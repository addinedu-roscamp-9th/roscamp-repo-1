#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


# =========================
# 신호등 브레이크: 빨강이면 /cmd_vel 0,0 덮어쓰기
# =========================
class TrafficLightBrake(Node):
    def __init__(self):
        super().__init__("traffic_light_brake_for_nav2")

        self.stop_mode = False  # True면 강제 정지
        self.last_state = "unknown"
        self.last_state_time = time.time()

        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/traffic_light_state", self.on_state, 10)

        self.PUB_HZ = 100  # 필요하면 50으로 올려도 됨
        self.timer = self.create_timer(1.0 / self.PUB_HZ, self.tick)

    def on_state(self, msg: String):
        s = (msg.data or "").strip().lower()
        if s not in ("red", "green", "unknown"):
            return

        self.last_state = s
        self.last_state_time = time.time()

        if s == "red":
            self.stop_mode = True
        elif s == "green":
            self.stop_mode = False
        # unknown은 유지(흔들림 방지)

    def tick(self):
        # stop_mode일 때만 /cmd_vel을 0,0으로 계속 덮어쓰기
        if self.stop_mode:
            out = Twist()
            out.linear.x = 0.0
            out.angular.z = 0.0
            self.stop_pub.publish(out)


# =========================
# 네 고정 좌표 네비 (기존 그대로)
# =========================
class WaypointRunner(Node):
    def __init__(self):
        super().__init__("route_567")

        self.navigate_action_name = "/navigate_to_pose"

        self.waypoints = [
            # 5
            (-0.036990267955641636, 0.1805891094917717,
             0.6877474614552492, 0.7259500184323026),

            # 6
            (-0.06932109118161453, 0.626396195066818,
             0.6781489607828358, 0.7349244770649294),

            # 7
            (-0.11263832452398341, 1.4010683942030093,
             0.7180190522675184, 0.6960234482981552),
        ]

        self.frame_id = "map"
        self.goal_timeout_sec = 180.0

        self._ac = ActionClient(self, NavigateToPose, self.navigate_action_name)
        self._idx = 0
        self._inflight = False
        self._sent_time = None
        self._last_wait_log = 0.0

        self.timer = self.create_timer(0.2, self.loop)

    def loop(self):
        if not self._ac.server_is_ready():
            now = time.time()
            if now - self._last_wait_log > 2.0:
                self.get_logger().info("액션 서버 대기중...")
                self._last_wait_log = now
            return

        if self._idx >= len(self.waypoints):
            self.get_logger().info("✅ route_567 완료")
            rclpy.shutdown()
            return

        if self._inflight and (time.time() - self._sent_time > self.goal_timeout_sec):
            self.get_logger().error("⛔ 타임아웃 → 다음 경로")
            self._inflight = False
            self._idx += 1
            return

        if not self._inflight:
            x, y, qz, qw = self.waypoints[self._idx]
            goal = NavigateToPose.Goal()
            goal.pose = self.make_pose(x, y, qz, qw)

            self.get_logger().info(f"➡ Goal {self._idx+1}/{len(self.waypoints)}")
            future = self._ac.send_goal_async(goal)
            future.add_done_callback(self.goal_response_cb)

            self._inflight = True
            self._sent_time = time.time()

    def make_pose(self, x, y, qz, qw):
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("⛔ Goal 거부")
            self._inflight = False
            self._idx += 1
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        self.get_logger().info("✅ 도착 완료")
        self._inflight = False
        self._idx += 1


def main():
    rclpy.init()

    # yolo.py는 별도로 실행해서 /traffic_light_state를 뿌리고 있어야 함
    brake = TrafficLightBrake()
    nav = WaypointRunner()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(brake)
    executor.add_node(nav)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 안전 정지
        pub = nav.create_publisher(Twist, "/cmd_vel", 10)
        msg = Twist()
        for _ in range(5):
            pub.publish(msg)
            time.sleep(0.05)

        brake.destroy_node()
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
