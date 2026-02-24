#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty


class AutoSpinLocalize(Node):

    def __init__(self):
        super().__init__('auto_spin_localize')

        # ===== 설정값 =====
        self.cmd_vel_topic = '/cmd_vel'
        self.spin_speed = 0.6          # rad/s
        self.max_time = 35.0           # sec
        self.settle_time = 1.5         # sec

        self.xy_cov_thresh = 0.05      # m^2
        self.yaw_cov_thresh = 0.20     # rad^2

        self.global_service = '/reinitialize_global_localization'
        # ==================

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_cb,
            qos_profile_sensor_data
        )

        self.gl_cli = self.create_client(Empty, self.global_service)

        self.last_cov_ok_time = None

        self.started = False
        self.start_time = None

        self.global_requested = False
        self.global_future = None
        self.global_done = False

        self.last_wait_log = 0.0

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info("초기위치 자동 찾기 시작! (서비스 1회 호출 + 회전 안정화)")

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        cov_x = cov[0]
        cov_y = cov[7]
        cov_yaw = cov[35]

        cov_ok = (
            cov_x < self.xy_cov_thresh and
            cov_y < self.xy_cov_thresh and
            cov_yaw < self.yaw_cov_thresh
        )

        now = self.get_clock().now()

        if cov_ok:
            if self.last_cov_ok_time is None:
                self.last_cov_ok_time = now
        else:
            self.last_cov_ok_time = None

    def spin_robot(self, on: bool):
        t = Twist()
        if on:
            t.angular.z = float(self.spin_speed)
        self.cmd_pub.publish(t)

    def maybe_request_global(self):

        if not self.gl_cli.service_is_ready():
            now_s = self.get_clock().now().nanoseconds / 1e9
            if now_s - self.last_wait_log > 1.0:
                self.get_logger().warn("global localization 서비스 대기중...")
                self.last_wait_log = now_s
            return

        if self.global_requested:
            return

        self.global_future = self.gl_cli.call_async(Empty.Request())
        self.global_requested = True
        self.get_logger().info("AMCL 전역 초기화 요청 전송!")

    def check_global_future(self):

        if not self.global_requested or self.global_done or self.global_future is None:
            return

        if self.global_future.done():
            try:
                _ = self.global_future.result()
                self.get_logger().info("AMCL 전역 초기화 응답 수신(완료)!")
            except Exception as e:
                self.get_logger().error(f"AMCL 전역 초기화 응답 예외: {e}")
            self.global_done = True

    def loop(self):

        if not self.started:
            self.started = True
            self.start_time = self.get_clock().now()
            self.get_logger().info("빙글빙글 회전 시작!")
            return

        # 1) 회전
        self.spin_robot(True)

        # 2) global localization 1회 요청
        self.maybe_request_global()
        self.check_global_future()

        # 3) 타임아웃
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.max_time:
            self.spin_robot(False)
            self.get_logger().warn("시간 초과! 정지 후 종료.")
            rclpy.shutdown()
            return

        # 4) 안정화 체크
        if self.last_cov_ok_time is not None:
            stable = (self.get_clock().now() - self.last_cov_ok_time).nanoseconds / 1e9
            if stable >= self.settle_time:
                self.spin_robot(False)
                self.get_logger().info("초기위치 안정화 완료! 정지 후 종료.")
                rclpy.shutdown()
                return


def main():
    rclpy.init()
    node = AutoSpinLocalize()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
