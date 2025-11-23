from indy_dcp_bridge.indydcp_client import IndyDCPClient

indy = IndyDCPClient("192.168.0.101", "NRMK-Indy7")
indy.connect()
# 例如工具沿法兰 +Z 方向 0.1m
indy.set_default_tcp([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
print("Default TCP set to:", indy.get_default_tcp())
indy.disconnect()






#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from indy_dcp_bridge.indydcp_client import IndyDCPClient

ROBOT_IP   = "192.168.0.101"
ROBOT_NAME = "NRMK-Indy7"

ROT_STEP_DEG = 2.0      # 单次旋转角度，建议先 2 度，确认后改 5 度
WAIT_POLL_DT = 0.05
WAIT_TIMEOUT = 20.0


def wait_motion_done(indy: IndyDCPClient, tag: str = ""):
    t0 = time.time()
    while True:
        st = indy.get_robot_status()
        if st.get("emergency") or st.get("error"):
            raise RuntimeError(f"[{tag}] Robot error/emergency: {st}")
        if (not st.get("busy")) and st.get("movedone"):
            return
        if time.time() - t0 > WAIT_TIMEOUT:
            raise TimeoutError(f"[{tag}] Motion timeout, last status={st}")
        time.sleep(WAIT_POLL_DT)


def task_rot_by(indy: IndyDCPClient, du: float, dv: float, dw: float, tag: str):
    """在当前 TCP 姿态基础上做相对旋转 [du,dv,dw]（deg），不平移。"""
    print(f"\n[{tag}] 相对旋转 (du,dv,dw) = ({du},{dv},{dw})")
    t_before = indy.get_task_pos()
    print(f"[{tag}] 执行前 TCP: {t_before}")

    delta = [0.0, 0.0, 0.0, du, dv, dw]  # 仅旋转，不平移
    indy.task_move_by(delta)
    wait_motion_done(indy, tag)

    t_after = indy.get_task_pos()
    print(f"[{tag}] 执行后 TCP: {t_after}")


def main():
    indy = IndyDCPClient(ROBOT_IP, ROBOT_NAME)
    print("Connecting ...")
    indy.connect()
    print("Connect: Server IP", ROBOT_IP)

    st = indy.get_robot_status()
    print("初始状态:", st)
    print("初始关节:", indy.get_joint_pos())
    print("当前默认 TCP (系统层面):", indy.get_default_tcp())
    print("当前 TCP pose (task_pos):", indy.get_task_pos())

    print("\n假定上面显示的 default_tcp 就是你想要的工具坐标系。")
    print("接下来所有相对旋转都围绕当前 TCP 原点进行，不再修改 TCP。")

    # 三个轴：0->X 1->Y 2->Z；用增量序列 [+θ, -θ, -θ, +θ] 实现 +θ→0→-θ→0
    seq = [ROT_STEP_DEG, -ROT_STEP_DEG, -ROT_STEP_DEG, ROT_STEP_DEG]

    # 绕 TCP X 轴摆动（对 task_move_by 来说改的是 u 分量）
    print("\n############  绕 TCP X 轴摆动  ############")
    for i, d in enumerate(seq):
        task_rot_by(indy, du=d, dv=0.0, dw=0.0, tag=f"Axis X step {i} Δu={d}deg")

    # 绕 TCP Y 轴摆动（改 v）
    print("\n############  绕 TCP Y 轴摆动  ############")
    for i, d in enumerate(seq):
        task_rot_by(indy, du=0.0, dv=d, dw=0.0, tag=f"Axis Y step {i} Δv={d}deg")

    # 绕 TCP Z 轴摆动（改 w）
    print("\n############  绕 TCP Z 轴摆动  ############")
    for i, d in enumerate(seq):
        task_rot_by(indy, du=0.0, dv=0.0, dw=d, tag=f"Axis Z step {i} Δw={d}deg")

    print("\n[测试结束] 断开连接（不修改系统 TCP）")
    indy.disconnect()


if __name__ == "__main__":
    main()
