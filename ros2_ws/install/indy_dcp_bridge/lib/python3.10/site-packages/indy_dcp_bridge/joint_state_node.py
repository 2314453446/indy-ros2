# joint_state_node.py
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import JointState

# 直接从同一包里导入
from indy_dcp_bridge.indydcp_client import IndyDCPClient



ROBOT_IP = "192.168.0.101"       # 控制柜 IP，根据实际修改
ROBOT_NAME = "NRMK-Indy7"        # Indy7 的名称
PUBLISH_RATE = 20.0              # Hz
JOINT_NAMES_6DOF = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']


def degs_to_rads(deg_list):
    return [math.radians(d) for d in deg_list]


class IndyDcpJointStateNode(Node):
    def __init__(self):
        super().__init__('indy_dcp_joint_state_node')

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', qos_profile
        )

        self.get_logger().info(f'Connecting to Indy at {ROBOT_IP} ...')
        self.indy = IndyDCPClient(ROBOT_IP, ROBOT_NAME)

        try:
            self.indy.connect()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Indy: {e}')
            raise

        self.get_logger().info('Connected to Indy. Starting joint state publishing.')

        # 定时器：按 PUBLISH_RATE 发布关节状态
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)

    def timer_callback(self):
        try:
            # 你已验证过：get_joint_pos() 返回 [q0..q5]，单位是度
            q_deg = self.indy.get_joint_pos()
            q_rad = degs_to_rads(q_deg)
        except Exception as e:
            self.get_logger().warning(f'Failed to read joint position: {e}')
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES_6DOF
        msg.position = q_rad
        # 速度、力矩可以先不填或填零数组
        msg.velocity = [0.0] * len(q_rad)
        msg.effort = [0.0] * len(q_rad)

        self.joint_state_pub.publish(msg)

    def destroy_node(self):
        # 关闭连接
        try:
            if self.indy is not None:
                self.indy.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IndyDcpJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
