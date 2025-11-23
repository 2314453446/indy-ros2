from indy_utils import indydcp_client as client
import time
import copy

ROBOT_IP = "192.168.0.101"     # 按实际修改
ROBOT_NAME = "NRMK-Indy7"
DELTA_DEG = 5.0                # 每个关节摆动角度，你可以改
POS_TOL   = 0.5                # 到位容差（度）
TIMEOUT   = 20.0               # 单次运动最大等待时间（秒）
POLL_DT   = 0.05               # 状态轮询周期（秒）


def wait_motion_done(indy, goal_q=None):
    """等待当前运动完成；可选对比关节角二次确认。"""
    t0 = time.time()
    while True:
        st = indy.get_robot_status()
        if st.get("emergency") or st.get("error"):
            raise RuntimeError(f"Robot in error/emg state: {st}")
        if (not st.get("busy")) and st.get("movedone"):
            break
        if time.time() - t0 > TIMEOUT:
            raise TimeoutError("Motion timeout")
        time.sleep(POLL_DT)

    if goal_q is not None:
        q_now = indy.get_joint_pos()
        diffs = [abs(a - b) for a, b in zip(q_now, goal_q)]
        print("  q_now(deg):", q_now)
        print("  diff(deg):", diffs)
        if any(d > POS_TOL for d in diffs):
            print("  WARN: out of tolerance.")


def main():
    indy = client.IndyDCPClient(ROBOT_IP, ROBOT_NAME)

    try:
        print("Connecting...")
        indy.connect()

        st = indy.get_robot_status()
        print("Robot status:", st)
        if not st.get("ready"):
            raise RuntimeError("Robot not ready, check Conty / servo / error.")

        # 以当前姿态作为“基准姿态”，不回 Home
        base_q = indy.get_joint_pos()
        print("Base joint position (deg):", base_q)

        dof = len(base_q)
        print(f"Detected DOF: {dof}")

        # 确认当前就处在基准（只是再次下发一次，以防之前未完全收敛）
        print("Ensure base pose (small correction if needed)...")
        indy.joint_move_to(base_q)
        wait_motion_done(indy, base_q)

        # 逐关节依次 ±DELTA_DEG 摆动
        for j in range(dof):
            print(f"\n=== Joint {j} test (±{DELTA_DEG} deg) ===")

            # 回到基准（起点就是现在这个安全位姿）
            print("  -> Back to base pose")
            indy.joint_move_to(base_q)
            wait_motion_done(indy, base_q)

            # +DELTA
            q_plus = copy.deepcopy(base_q)
            q_plus[j] += DELTA_DEG
            print(f"  -> Joint {j} +{DELTA_DEG} deg")
            indy.joint_move_to(q_plus)
            wait_motion_done(indy, q_plus)

            # 回到基准
            print("  -> Back to base pose")
            indy.joint_move_to(base_q)
            wait_motion_done(indy, base_q)

            # -DELTA
            q_minus = copy.deepcopy(base_q)
            q_minus[j] -= DELTA_DEG
            print(f"  -> Joint {j} -{DELTA_DEG} deg")
            indy.joint_move_to(q_minus)
            wait_motion_done(indy, q_minus)

            # 回到基准
            print("  -> Back to base pose (joint done)")
            indy.joint_move_to(base_q)
            wait_motion_done(indy, base_q)

        print("\nAll joints finished, stay at original safe pose.")
        # 最终停在最初的基准姿态
        indy.joint_move_to(base_q)
        wait_motion_done(indy, base_q)

    finally:
        indy.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
