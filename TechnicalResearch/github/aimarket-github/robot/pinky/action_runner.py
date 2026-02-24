#!/usr/bin/env python3
"""
Pinky AMR - Action Runner (로봇 사이드 실행 파일)
LLM이 생성한 JSON 명령을 받아 실제 ROS 2 토픽 발행 및 서비스 호출

[ROS 2 토픽/서비스 정보]
- 이동:  /cmd_vel        | geometry_msgs/Twist
- LED:   set_led (srv)  | pinky_interfaces/SetLed
- LCD:   pinky_lcd 라이브러리 직접 사용

[실행 방법 - Pinky 로봇 내부에서]
  export ROS_DOMAIN_ID=0
  source /opt/ros/jazzy/setup.bash
  source ~/pinky_pro/install/local_setup.bash
  python3 action_runner.py '{"type":"set_led","params":{"r":255,"g":0,"b":0}}'

[지원 액션 타입]
  move_twist  : /cmd_vel 토픽으로 주행 (linear_x, angular_z, duration)
  set_led     : LED 색상 설정 (r, g, b)
  set_emotion : LCD 감정 표시 (emotion: happy/sad/angry/normal 등)
"""
import sys
import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pinky_interfaces.srv import SetLed

try:
    from pinky_lcd import LCD
    from PIL import Image, ImageDraw
    HAS_LCD = True
except ImportError:
    HAS_LCD = False


class ActionRunner(Node):
    def __init__(self):
        super().__init__('action_runner')
        # 이동 명령 퍼블리셔 → /cmd_vel 토픽
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # LED 서비스 클라이언트
        self.led_cli = self.create_client(SetLed, 'set_led')

    def move_twist(self, linear_x, angular_z, duration):
        self.get_logger().info(f"Moving: x={linear_x}, z={angular_z} for {duration}s")
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)

        # 정지
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Movement completed")

    def set_led(self, r, g, b):
        self.get_logger().info(f"Setting LED: R={r}, G={g}, B={b}")
        if not self.led_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("LED service not available")
            return

        req = SetLed.Request()
        req.command = "fill"
        req.r = int(r)
        req.g = int(g)
        req.b = int(b)
        self.led_cli.call_async(req)

    def set_emotion(self, emotion):
        self.get_logger().info(f"Setting Emotion: {emotion}")
        if not HAS_LCD:
            self.get_logger().error("LCD library not found")
            return

        try:
            lcd = LCD()
            w = int(getattr(lcd, "width", 240))
            h = int(getattr(lcd, "height", 240))
            img = Image.new("RGB", (w, h), (0, 0, 0))
            draw = ImageDraw.Draw(img)

            color = (255, 255, 0)
            if emotion == "happy":
                draw.ellipse([w//4, h//4, 3*w//4, 3*h//4], outline=color, width=5)
                draw.arc([w//3, h//2, 2*w//3, 3*h//4], 0, 180, fill=color, width=5)
            elif emotion == "sad":
                draw.ellipse([w//4, h//4, 3*w//4, 3*h//4], outline=color, width=5)
                draw.arc([w//3, 3*h//4, 2*w//3, h], 180, 0, fill=color, width=5)
            else:
                draw.text((w//2-20, h//2), emotion, fill=(255, 255, 255))

            lcd.display(img)
        except Exception as e:
            self.get_logger().error(f"LCD error: {e}")


def main():
    if len(sys.argv) < 2:
        print("Usage: action_runner.py '<json_string>'")
        return

    try:
        data = json.loads(sys.argv[1])
    except Exception as e:
        print(f"Invalid JSON: {e}")
        return

    rclpy.init()
    node = ActionRunner()

    action_type = data.get("type")
    params = data.get("params", {})

    if action_type == "move_twist":
        node.move_twist(
            params.get("linear_x", 0),
            params.get("angular_z", 0),
            params.get("duration", 1)
        )
    elif action_type == "set_led":
        node.set_led(params.get("r", 0), params.get("g", 0), params.get("b", 0))
    elif action_type == "set_emotion":
        node.set_emotion(params.get("emotion", "normal"))
    else:
        print(f"Unknown action: {action_type}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
