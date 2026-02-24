#!/usr/bin/env python3
"""
서버 사이드 ROS 2 브릿지 - Arm 17 / Arm 18
AImarket.py에서 발행한 ROS 2 토픽을 수신하여 로봇팔 스크립트를 실행합니다.

[ROS 2 토픽 정보]
- Arm 17: ROS_DOMAIN_ID=7,  Topic: /pick_items,          Message: std_msgs/Int32MultiArray
- Arm 18: ROS_DOMAIN_ID=19, Topic: /robot_arm_18/pick_items, Message: std_msgs/Int32MultiArray

[실행 방법]
  # Arm 17 브릿지 (서버에서 실행):
  export ROS_DOMAIN_ID=7
  python3 arm_server_ros2.py 17

  # Arm 18 브릿지 (서버에서 실행):
  export ROS_DOMAIN_ID=19
  python3 arm_server_ros2.py 18

[환경변수]
  ARM17_IP   : 로봇팔 17번 IP (기본값: 192.168.0.xxx)
  ARM17_USER : SSH 사용자명 (기본값: jetcobot)
  ARM17_PASS : SSH 비밀번호
  ARM18_IP   : 로봇팔 18번 IP
  ARM18_USER : SSH 사용자명
  ARM18_PASS : SSH 비밀번호
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import subprocess
import os


class RobotArmBridge(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_arm_{robot_id}_bridge')
        self.robot_id = str(robot_id)

        # 환경변수에서 SSH 접속 정보 읽기
        if self.robot_id == "17":
            self.ip   = os.environ.get("ARM17_IP", "192.168.0.xxx")
            self.user = os.environ.get("ARM17_USER", "jetcobot")
            self.pw   = os.environ.get("ARM17_PASS", "your_password")
            topic = '/pick_items'
        else:
            self.ip   = os.environ.get("ARM18_IP", "192.168.0.xxx")
            self.user = os.environ.get("ARM18_USER", "jetcobot")
            self.pw   = os.environ.get("ARM18_PASS", "your_password")
            topic = '/robot_arm_18/pick_items'

        self.subscription = self.create_subscription(
            Int32MultiArray,
            topic,
            self.pick_callback,
            10
        )
        self.get_logger().info(
            f"🚀 Robot Arm {self.robot_id} Bridge Ready | "
            f"Topic: {topic} | IP: {self.ip}"
        )

    def ssh_run(self, cmd: str):
        """SSH 명령 실행 헬퍼"""
        full_cmd = (
            f"sshpass -p '{self.pw}' ssh -o StrictHostKeyChecking=no "
            f"{self.user}@{self.ip} \"{cmd}\""
        )
        self.get_logger().info(f"SSH CMD: {full_cmd}")
        subprocess.run(full_cmd, shell=True, executable="/bin/bash")

    def pick_callback(self, msg):
        marker_ids = msg.data
        self.get_logger().info(f"📥 Received Pick Request: {marker_ids}")

        for mid in marker_ids:
            try:
                if self.robot_id == "18":
                    cmd = (
                        "source /home/{u}/venv/mycobot/bin/activate && "
                        "cd /home/{u}/aruco_pick && python3 check.py"
                    ).format(u=self.user)
                    self.ssh_run(cmd)

                elif self.robot_id == "17":
                    if str(mid) == "6":
                        cmd = (
                            "/home/{u}/photo_upload_bundle/client/.venv/bin/python "
                            "/home/{u}/aruco_pick/step1_aruco_check6.py"
                        ).format(u=self.user)
                    elif str(mid) == "37":
                        cmd = (
                            "/home/{u}/photo_upload_bundle/client/.venv/bin/python "
                            "/home/{u}/aruco_pick/step1_aruco_check37.py"
                        ).format(u=self.user)
                    else:
                        cmd = (
                            "source /home/{u}/mycobot_venv/bin/activate && "
                            "cd /home/{u}/aruco_pick && python3 step1_aruco_check.py"
                        ).format(u=self.user)
                    self.ssh_run(cmd)

                self.get_logger().info(f"✅ Marker {mid} done")
            except Exception as e:
                self.get_logger().error(f"❌ Failed for Marker {mid}: {e}")


def main():
    import sys
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "17"

    rclpy.init()
    node = RobotArmBridge(robot_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
