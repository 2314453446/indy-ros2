#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import copy
from typing import List

# 直接使用你包里的 DCP 客户端
from indy_dcp_bridge.indydcp_client import IndyDCPClient

# ========= 用户可配置参数 =========
ROBOT_IP   = "192.168.0.101"
ROBOT_NAME = "NRMK-Indy7"

# 自定义 TCP：在法兰坐标系下的位置(毫米)与姿态欧拉角(度)；默认仅Z向100mm
TCP_OFFSET_MM  = [0.0, 0.0, 100.0]   # x y z (mm)
TCP_ROT_DEG    = [0.0, 0.0, 0.0]     # u v w (deg), 如需绕法兰自带姿态再旋转

# 基于 TCP 的摆动幅度（可修改）
TRANS_STEP_MM  = 1.0    # 平移幅度（mm），逐轴 ±5 mm
ROT_STEP_DEG   = 1.0    # 旋转幅度（deg），逐轴 ±5 deg

# 运动与判定参数
TIMEOUT     = 20.0      # 单步最大等待时间（s）
POLL_DT     = 0.05      # 轮询周期（s）
POS_TOL_DEG = 0.5       # 关节角到位容差（deg）
PRINT_TASK  = True      # 每步是否打印任务空间目标/实际

# ========== 数学与变换工具 ==========
def deg2rad(x): return x * math.pi / 180.0
def rad2deg(x): return x * 180.0 / math.pi

def euler_uvw_to_R(u_deg, v_deg, w_deg):
    # 约定：绕 x(u)、y(v)、z(w) 轴的内禀欧拉角，顺序 X-Y-Z
    ux, vy, wz = map(deg2rad, [u_deg, v_deg, w_deg])
    Rx = [[1,0,0],
          [0,math.cos(ux),-math.sin(ux)],
          [0,math.sin(ux), math.cos(ux)]]
    Ry = [[ math.cos(vy),0,math.sin(vy)],
          [0,1,0],
          [-math.sin(vy),0,math.cos(vy)]]
    Rz = [[math.cos(wz),-math.sin(wz),0],
          [math.sin(wz), math.cos(wz),0],
          [0,0,1]]
    return matmul(matmul(Rx,Ry),Rz)

def R_to_euler_uvw(R):
    # 与上面 euler 顺序匹配的反解（简化版，假设无奇异大角度）
    # 参考标准 XYZ 内禀欧拉
    r20 = R[2][0]
    if abs(r20) < 1.0:
        vy = math.asin(-r20)
        cy = math.cos(vy)
        ux = math.atan2(R[2][1]/cy, R[2][2]/cy)
        wz = math.atan2(R[1][0]/cy, R[0][0]/cy)
    else:
        # 奇异近似处理
        vy = math.pi/2 if r20 <= -1.0 else -math.pi/2
        ux = 0.0
        wz = math.atan2(-R[0][1], R[1][1])
    return list(map(rad2deg, [ux, vy, wz]))

def matmul(A,B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def v_add(a,b): return [a[i]+b[i] for i in range(3)]
def v_sub(a,b): return [a[i]-b[i] for i in range(3)]
def v_scale(a,s): return [a[i]*s for i in range(3)]

def apply_transform(p_world_flange, R_world_flange, p_flange_tcp, R_flange_tcp):
    # 计算世界系下 TCP 位姿： p_w_tcp = p_w_f + R_w_f * p_f_tcp
    # R_w_tcp = R_w_f * R_f_tcp
    p_w_tcp = [
        p_world_flange[0] + sum(R_world_flange[0][k]*p_flange_tcp[k] for k in range(3)),
        p_world_flange[1] + sum(R_world_flange[1][k]*p_flange_tcp[k] for k in range(3)),
        p_world_flange[2] + sum(R_world_flange[2][k]*p_flange_tcp[k] for k in range(3)),
    ]
    R_w_tcp = matmul(R_world_flange, R_flange_tcp)
    return p_w_tcp, R_w_tcp

def compose_target_flange_from_tcp(target_tcp_pos_w, target_tcp_rot_w, p_flange_tcp, R_flange_tcp):
    # 已知世界系目标 TCP 位姿，反求目标法兰位姿：
    # T_w_f = T_w_tcp * inv(T_f_tcp) => p_w_f = p_w_tcp - R_w_f * p_f_tcp 需要先解 R_w_f：
    # R_w_f = R_w_tcp * inv(R_f_tcp)
    # 注意 inv(R) = R^T
    R_f_tcp_T = [[R_flange_tcp[j][i] for j in range(3)] for i in range(3)]
    R_w_f = matmul(target_tcp_rot_w, R_f_tcp_T)
    # p_w_f = p_w_tcp - R_w_f * p_f_tcp
    p_w_f = [
        target_tcp_pos_w[0] - sum(R_w_f[0][k]*p_flange_tcp[k] for k in range(3)),
        target_tcp_pos_w[1] - sum(R_w_f[1][k]*p_flange_tcp[k] for k in range(3)),
        target_tcp_pos_w[2] - sum(R_w_f[2][k]*p_flange_tcp[k] for k in range(3)),
    ]
    return p_w_f, R_w_f

# ========== DCP 执行工具 ==========
def wait_motion_done(indy: IndyDCPClient, timeout=TIMEOUT):
    t0 = time.time()
    while True:
        st = indy.get_robot_status()
        if st.get("emergency") or st.get("error"):
            raise RuntimeError(f"Robot error/emergency: {st}")
        if (not st.get("busy")) and st.get("movedone"):
            return True
        if time.time() - t0 > timeout:
            raise TimeoutError("Motion timeout")
        time.sleep(POLL_DT)

def joint_to(indy: IndyDCPClient, q_deg: List[float]):
    indy.joint_move_to(q_deg)
    wait_motion_done(indy)

def near(q_now, q_goal, tol=POS_TOL_DEG):
    return all(abs(a-b) < tol for a,b in zip(q_now,q_goal))

# ========== 主流程 ==========
def main():
    # 连接
    indy = IndyDCPClient(ROBOT_IP, ROBOT_NAME)
    print("Connecting...")
    indy.connect()

    try:
        # 读取当前关节与法兰位姿
        st = indy.get_robot_status()
        if not st.get("ready") or st.get("emergency") or st.get("error"):
            raise RuntimeError(f"Robot not ready: {st}")

        q_base = indy.get_joint_pos()  # deg
        t_flange = indy.get_task_pos() # [x(m), y(m), z(m), u(deg), v(deg), w(deg)]
        p_w_f = t_flange[0:3]
        R_w_f = euler_uvw_to_R(t_flange[3], t_flange[4], t_flange[5])

        # 构造法兰->TCP 变换
        p_f_tcp = [TCP_OFFSET_MM[0]/1000.0, TCP_OFFSET_MM[1]/1000.0, TCP_OFFSET_MM[2]/1000.0]
        R_f_tcp = euler_uvw_to_R(TCP_ROT_DEG[0], TCP_ROT_DEG[1], TCP_ROT_DEG[2])

        # 基准 TCP 位姿（世界系）
        p_w_tcp0, R_w_tcp0 = apply_transform(p_w_f, R_w_f, p_f_tcp, R_f_tcp)
        uvw_tcp0 = R_to_euler_uvw(R_w_tcp0)

        print(f"Base flange pose (m,deg): pos={p_w_f}, uvw={t_flange[3:6]}")
        print(f"Base TCP pose (m,deg):    pos={['%.3f'%v for v in p_w_tcp0]}, uvw={['%.2f'%v for v in uvw_tcp0]}")

        # 生成“逐轴摆动”的目标（平移与旋转），采用“+Δ→回→-Δ→回”的四步模式
        def make_targets_trans(axis, step_m):
            # 平移：仅改 TCP 的平移
            offset = [0.0,0.0,0.0]
            offset[axis] = step_m
            # +Δ
            yield v_add(p_w_tcp0, offset), R_w_tcp0, f"TRANS axis {axis} +{step_m*1000:.1f}mm"
            # 回
            yield p_w_tcp0, R_w_tcp0, f"TRANS axis {axis} back"
            # -Δ
            yield v_sub(p_w_tcp0, offset), R_w_tcp0, f"TRANS axis {axis} -{step_m*1000:.1f}mm"
            # 回
            yield p_w_tcp0, R_w_tcp0, f"TRANS axis {axis} back"

        def make_targets_rot(axis, step_deg):
            # 旋转：仅改 TCP 的旋转
            duvw = [0.0,0.0,0.0]
            duvw[axis] = step_deg
            R_plus = matmul(R_w_tcp0, euler_uvw_to_R(duvw[0], duvw[1], duvw[2]))
            R_minus = matmul(R_w_tcp0, euler_uvw_to_R(-duvw[0], -duvw[1], -duvw[2]))
            # +Δ
            yield p_w_tcp0, R_plus, f"ROT axis {axis} +{step_deg:.1f}deg"
            # 回
            yield p_w_tcp0, R_w_tcp0, f"ROT axis {axis} back"
            # -Δ
            yield p_w_tcp0, R_minus, f"ROT axis {axis} -{step_deg:.1f}deg"
            # 回
            yield p_w_tcp0, R_w_tcp0, f"ROT axis {axis} back"

        # 执行器：把“目标 TCP 位姿”变回“目标法兰位姿”，再逆解成关节并执行
        def execute_tcp_target(p_w_tcp, R_w_tcp, tag=""):
            # 目标法兰位姿
            p_w_f_tgt, R_w_f_tgt = compose_target_flange_from_tcp(p_w_tcp, R_w_tcp, p_f_tcp, R_f_tcp)
            uvw_f_tgt = R_to_euler_uvw(R_w_f_tgt)
            t_flange_tgt = [p_w_f_tgt[0], p_w_f_tgt[1], p_w_f_tgt[2], uvw_f_tgt[0], uvw_f_tgt[1], uvw_f_tgt[2]]

            if PRINT_TASK:
                print(f"[{tag}] target flange (m,deg): pos={['%.3f'%v for v in p_w_f_tgt]}, uvw={['%.2f'%v for v in uvw_f_tgt]}")

            # 逆解：用当前关节作为起点
            q_now = indy.get_joint_pos()
            q_sol = indy.get_inv_kin(task_pos=t_flange_tgt, init_q=q_now)
            if not q_sol or all(abs(v) < 1e-9 for v in q_sol):
                print(f"  IK failed or zero solution, skip.")
                return

            # 关节执行（到位与二次确认）
            try:
                joint_to(indy, q_sol)
            except Exception as e:
                print(f"  joint_move_to error: {e}")
                return

            # 校验
            q_reached = indy.get_joint_pos()
            ok = near(q_reached, q_sol, POS_TOL_DEG)
            print(f"  reached ok={ok}, q_reached(deg)={['%.3f'%v for v in q_reached]}")

            if PRINT_TASK:
                t_now = indy.get_task_pos()
                print(f"  current flange (m,deg): pos={['%.3f'%v for v in t_now[:3]]}, uvw={['%.2f'%v for v in t_now[3:6]]}")

        # 执行逐轴平移与逐轴旋转
        step_m = TRANS_STEP_MM / 1000.0
        for ax in range(3):    # 0:X 1:Y 2:Z
            for p_tcp, R_tcp, tag in make_targets_trans(ax, step_m):
                execute_tcp_target(p_tcp, R_tcp, tag)

        for ax in range(3):
            for p_tcp, R_tcp, tag in make_targets_rot(ax, ROT_STEP_DEG):
                execute_tcp_target(p_tcp, R_tcp, tag)

        print("Done all TCP dither tests.")

    finally:
        indy.disconnect()
        print("Disconnected.")
        

if __name__ == "__main__":
    main()
