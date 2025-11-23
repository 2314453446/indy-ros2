import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from indy_dcp_bridge.indydcp_client import IndyDCPClient


ROBOT_IP = "192.168.0.101"        # 控制柜 IP，根据实际修改
ROBOT_NAME = "NRMK-Indy7"         # Indy7 名称
JOINT_NAMES_6DOF = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
PUBLISH_RATE = 20.0               # /joint_states 发布频率 Hz
WAYPOINT_DT = 0.05                # 简单等待间隔（秒）


def rads2degs(rad_list):
    return [math.degrees(r) for r in rad_list]


class IndyDcpBridgeNode(Node):
    def __init__(self):
        super().__init__('indy_dcp_bridge_node')

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

        # 1) 连接 Indy，仅此一个客户端
        self.get_logger().info(f'Connecting to Indy at {ROBOT_IP} ...')
        self.indy = IndyDCPClient(ROBOT_IP, ROBOT_NAME)
        self.indy.connect()
        self.get_logger().info('Connected to Indy (bridge node).')

        # 2) JointState 发布器（替代原 joint_state_node）
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', qos
        )
        self.joint_state_feedback = JointTrajectoryPoint()
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)

        # 3) FollowJointTrajectory Action Server（给 MoveIt 用）
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('FollowJointTrajectory Action Server started.')

    # ---------- JointState 发布 ----------

    def timer_callback(self):
        """周期读取 DCP 关节角并发布到 /joint_states，同时更新反馈用位置。"""
        try:
            q_deg = self.indy.get_joint_pos()  # [deg]
        except Exception as e:
            self.get_logger().warning(f'Failed to read joint_pos: {e}')
            return

        q_rad = [math.radians(d) for d in q_deg]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES_6DOF
        msg.position = q_rad
        msg.velocity = [0.0] * len(q_rad)
        msg.effort = [0.0] * len(q_rad)

        self.joint_state_pub.publish(msg)

        # 也用于 FollowJointTrajectory 的 actual 反馈
        self.joint_state_feedback.positions = q_rad

    # ---------- Action 回调 ----------

    def goal_callback(self, goal_request):
        jn = list(goal_request.trajectory.joint_names)
        self.get_logger().info(f'Received goal request. joint_names = {jn}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('--- Execute FollowJointTrajectory goal ---')

        traj = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)
        points = traj.points

        self.get_logger().info(f'  joint_names: {joint_names}')
        self.get_logger().info(f'  number of points: {len(points)}')

        result = FollowJointTrajectory.Result()
        feedback_msg = FollowJointTrajectory.Feedback()

        if not points:
            self.get_logger().warn('  Empty trajectory received.')
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.succeed()
            return result

        # 逐点执行
        for idx, pt in enumerate(points):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('  Goal canceled by client.')
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return result

            q_rad = list(pt.positions)
            q_deg = rads2degs(q_rad)

            self.get_logger().info(
                f'  Point {idx}: q_rad = {["%.3f" % v for v in q_rad]}'
            )
            self.get_logger().info(
                f'           q_deg = {["%.3f" % v for v in q_deg]}'
            )

            # 调用前后打印状态，便于调试 DCP 层
            try:
                status_before = self.indy.get_robot_status()
                self.get_logger().info(f'  status before move: {status_before}')
            except Exception as e:
                self.get_logger().warning(f'  Failed to get status before move: {e}')
                status_before = {}

            try:
                self.get_logger().info(f'  -> sending joint_move_to (point {idx})')
                self.indy.joint_move_to(q_deg)  # 这里是真正控制机械臂的地方
                self.get_logger().info(f'  <- joint_move_to done (point {idx})')
            except Exception as e:
                self.get_logger().error(f'  ERROR sending waypoint {idx}: {e}')
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                goal_handle.abort()
                return result

            try:
                status_after = self.indy.get_robot_status()
                self.get_logger().info(f'  status after move: {status_after}')
            except Exception as e:
                self.get_logger().warning(f'  Failed to get status after move: {e}')

            # 简单等待一小段时间，避免过密调用
            time.sleep(WAYPOINT_DT)

            # 构造反馈
            feedback_msg.desired = JointTrajectoryPoint()
            feedback_msg.desired.positions = q_rad

            feedback_msg.actual = JointTrajectoryPoint()
            feedback_msg.actual.positions = list(self.joint_state_feedback.positions)

            feedback_msg.error = JointTrajectoryPoint()
            if feedback_msg.actual.positions:
                errs = [
                    a - d
                    for a, d in zip(feedback_msg.actual.positions, feedback_msg.desired.positions)
                ]
                feedback_msg.error.positions = errs
                self.get_logger().info(
                    f'  feedback point {idx}: actual(rad) = {["%.3f" % v for v in feedback_msg.actual.positions]}, '
                    f'err(rad) = {["%.3f" % v for v in errs]}'
                )
            else:
                self.get_logger().info('  feedback: no /joint_states yet, skip error calc')

            goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info('--- FollowJointTrajectory goal finished ---')
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result

    def destroy_node(self):
        try:
            if self.indy is not None:
                self.indy.disconnect()
                self.get_logger().info('Disconnected from Indy (bridge node).')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IndyDcpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
