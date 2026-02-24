#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class WaypointRunner(Node):
    def __init__(self):
        super().__init__("waypoint_runner_part1")

        self.navigate_action_name = "/navigate_to_pose"

        # 경로 1,2,3만
        self.waypoints = [
            (-0.7303844004651672, 0.816622402752207,  -0.7045300430399346, 0.709674163580828),   # 1
            (-0.7333716543681044, 0.21762430300100136, -0.6637323584344912, 0.7479701574040157), # 2
            (-0.49549180778724794, 0.003812789935692772, 0.018355280541727217, 0.9998315276466503), # 3
        ]

        self.frame_id = "map"
        self.goal_timeout_sec = 180.0

        self._ac = ActionClient(self, NavigateToPose, self.navigate_action_name)
        self._idx = 0
        self._sent_time = None
        self._inflight = False
        self._waiting_after_3 = False
        self._wait_until = 0.0

        self.get_logger().info("WaypointRunner PART1 시작 (/navigate_to_pose wait_for_server...)")
        self._ac.wait_for_server()

        self.timer = self.create_timer(0.2, self.loop)

    def loop(self):
        # 경로3 도착 후 10초 대기(블로킹 없이)
        if self._waiting_after_3:
            if time.time() >= self._wait_until:
                self.get_logger().info("대기 종료 → 종료합니다.")
                rclpy.shutdown()
            return

        # 모든 경로 완료
        if self._idx >= len(self.waypoints):
            self.get_logger().info("PART1 모든 경로 주행 완료. 종료합니다.")
            rclpy.shutdown()
            return

        # 타임아웃 체크
        if self._inflight and self._sent_time is not None:
            if (time.time() - self._sent_time) > self.goal_timeout_sec:
                self.get_logger().error("Goal 타임아웃 → 다음 경로로 이동")
                self._inflight = False
                self._sent_time = None
                self._idx += 1
                return

        # Goal 전송
        if not self._inflight:
            x, y, qz, qw = self.waypoints[self._idx]

            goal = NavigateToPose.Goal()
            goal.pose = self.make_pose_stamped(x, y, qz, qw)

            self.get_logger().info(
                f"[{self._idx+1}/{len(self.waypoints)}] Goal 전송: x={x:.3f}, y={y:.3f}"
            )

            future = self._ac.send_goal_async(goal)
            future.add_done_callback(self.goal_response_cb)

            self._inflight = True
            self._sent_time = time.time()

    def make_pose_stamped(self, x, y, qz, qw):
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()

        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0

        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)

        return ps

    def goal_response_cb(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal 거부됨 → 다음 경로")
            self._inflight = False
            self._sent_time = None
            self._idx += 1
            return

        self.get_logger().info("Goal 수락됨")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        try:
            res = future.result()
            status = res.status
        except Exception as e:
            self.get_logger().error(f"결과 수신 예외: {e}")
            status = None

        # 성공일 때만 다음 waypoint로
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("도착 성공 (SUCCEEDED)")

            # 경로3(인덱스2) 도착이면 10초 대기 후 종료
            if self._idx == 2:
                self.get_logger().info("경로3 도착 → 10초 대기 시작")
                self._waiting_after_3 = True
                self._wait_until = time.time() + 10.0
                self._inflight = False
                self._sent_time = None
                return

            self._idx += 1
        else:
            self.get_logger().error(f"도착 실패/중단 (status={status}) → 다음 경로로 넘김")
            self._idx += 1

        self._inflight = False
        self._sent_time = None


def main():
    rclpy.init()
    node = WaypointRunner()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
