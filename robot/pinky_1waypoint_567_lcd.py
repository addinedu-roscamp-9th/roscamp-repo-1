#!/usr/bin/env python3
import time
import subprocess
import os
import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from pinky_interfaces.srv import SetLed

# LCD
from pinky_lcd import LCD
from PIL import Image, ImageDraw, ImageFont


class WaypointRunner(Node):
    def __init__(self):
        super().__init__("waypoint_runner_with_lcd")

        # led_server 자동 실행
        self.led_proc = subprocess.Popen(
            ["ros2", "run", "pinky_led", "led_server"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )

        self.navigate_action_name = "/navigate_to_pose"

        self.waypoints = [
            (-0.02303434895238122, 0.1364322663572329, 0.044065607282374,  0.9990286393566681),
            (-0.06635784676736996, 0.5782167513155609, 0.7195170305099051, 0.6944747963793995),
            (-0.05697984997934965, 1.4278316079965387, 0.7210638963950985, 0.6928685714589158),
        ]

        self._ac = ActionClient(self, NavigateToPose, self.navigate_action_name)
        self._ac.wait_for_server()

        self.led_cli = self.create_client(SetLed, "/set_led")

        self._idx = 0
        self._inflight = False
        self._waiting_at_2 = False
        self._wait_until = 0.0

        # 🔥 추가: 최종 도착 후 대기 상태
        self._waiting_pickup = False

        self.timer = self.create_timer(0.2, self.loop)

    # ---------------- LED ----------------
    def set_led_fill(self, r, g, b):
        req = SetLed.Request()
        req.command = "fill"
        req.r = r
        req.g = g
        req.b = b
        self.led_cli.call_async(req)

    # ---------------- LCD 출력 ----------------
    def show_pickup_message(self):
        try:
            lcd = LCD()

            w = int(getattr(lcd, "width", 240))
            h = int(getattr(lcd, "height", 240))

            img = Image.new("RGB", (w, h), (0, 0, 0))
            draw = ImageDraw.Draw(img)

            font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

            try:
                font = ImageFont.truetype(font_path, 26)
            except Exception as e:
                self.get_logger().error(f"폰트 로드 실패: {e}")
                font = None

            text = "물건을\n회수해주세요."

            draw.multiline_text(
                (20, h//2 - 30),
                text,
                fill=(255, 255, 255),
                font=font,
                spacing=8
            )

            lcd.img_show(img)

            self.get_logger().info("LCD 표시 완료 → 15초 대기 시작")

        except Exception as e:
            self.get_logger().error(f"LCD 출력 실패: {e}")

    # ------------------------------------------------

    def loop(self):

        # 🔴 2번 waypoint (횡단보도) 대기
        if self._waiting_at_2:
            if time.time() >= self._wait_until:
                self.set_led_fill(0, 0, 0)
                self._waiting_at_2 = False
                self._idx += 1
            return

        # 🔵 최종 목적지 수령 대기
        if self._waiting_pickup:
            if time.time() >= self._wait_until:
                self.get_logger().info("15초 경과 → 자동 복귀 시작")
                self._waiting_pickup = False
                self._idx = 0          # 🔥 처음 waypoint로 복귀
                self._inflight = False
            return

        # 🟢 모든 경로 완료 → LCD 출력 + 15초 대기
        if self._idx >= len(self.waypoints):
            self.get_logger().info("최종 목적지 도착 → LCD 출력")
            self.show_pickup_message()
            self._waiting_pickup = True
            self._wait_until = time.time() + 15.0   # 🔥 15초 설정
            return

        # 🟡 일반 waypoint 이동
        if not self._inflight:
            x, y, qz, qw = self.waypoints[self._idx]

            goal = NavigateToPose.Goal()
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = self.get_clock().now().to_msg()

            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw

            goal.pose = ps

            fut = self._ac.send_goal_async(goal)
            fut.add_done_callback(self.goal_response_cb)

            self._inflight = True

    def goal_response_cb(self, future):
        goal_handle = future.result()
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        self._inflight = False

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error("도착 실패 → 다음 경로")
            self._idx += 1
            return

        # 🔴 2번에서 LED + 6초 정지
        if self._idx == 1:
            self.set_led_fill(255, 0, 0)
            self._waiting_at_2 = True
            self._wait_until = time.time() + 6.0
            return

        self._idx += 1


def main():
    rclpy.init()
    node = WaypointRunner()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
