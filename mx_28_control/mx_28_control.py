#!/usr/bin/env python3

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from client_vision_interfaces.msg import TurtlebotDetection


def mx_28_controller(
    vision_msg: TurtlebotDetection,
    current_angle: float,
) -> Optional[float]:
    # 제어 상수
    target_class_id = -1
    min_score = 0.5
    image_width = 640.0
    middle_left = image_width / 3.0
    middle_right = image_width * 2.0 / 3.0
    edge_margin = 5.0
    shelf_y_threshold = 400.0
    travel_angle = 175.0
    shelf_angle = 210.0
    angle_speed_ratio = 0.75

    #---------------- 입력 검사
    sizes = [
        len(vision_msg.class_ids),
        len(vision_msg.score),
        len(vision_msg.x1),
        len(vision_msg.y1),
        len(vision_msg.x2),
        len(vision_msg.y2),
    ]
    if len(set(sizes)) != 1 or sizes[0] == 0:
        return None

    best_index = None
    best_score = -math.inf

    #---------------- 박스 선택
    for index, score in enumerate(vision_msg.score):
        if score < min_score:
            continue
        if target_class_id != -1 and vision_msg.class_ids[index] != target_class_id:
            continue
        if score > best_score:
            best_score = score
            best_index = index

    if best_index is None:
        return None

    #---------------- 위치 계산
    x1 = float(vision_msg.x1[best_index])
    x2 = float(vision_msg.x2[best_index])
    y1 = float(vision_msg.y1[best_index])
    y2 = float(vision_msg.y2[best_index])

    #---------------- 규칙 적용
    target_angle = travel_angle

    if current_angle < shelf_angle - 10.0: #낮은 앵글일 시
        if y1 ==0 and y2 < 440:
            target_angle = shelf_angle
        else:
            target_angle = travel_angle
        
    else:#높은 앵글일 시
        if x2 - x1 < 160:
            target_angle = travel_angle
        else:
            target_angle = shelf_angle
    if target_class_id == 2:
        target_angle = travel_angle

    return current_angle + (target_angle - current_angle) * angle_speed_ratio


class Mx28Node(Node):
    def __init__(self):
        super().__init__('mx_28_control')

        self.declare_parameter('vision_topic', '/detection')
        self.declare_parameter('output_topic', '/gripper/mx28_angle')
        self.declare_parameter('tick_hz', 20.0)
        self.declare_parameter('target_class_id', -1)
        self.declare_parameter('min_score', 0.5)
        self.declare_parameter('target_y', 240.0)
        self.declare_parameter('deadband_y', 20.0)
        self.declare_parameter('initial_angle', 150.0)
        self.declare_parameter('min_angle', 80.0)
        self.declare_parameter('max_angle', 220.0)
        self.declare_parameter('kp', 0.05)
        self.declare_parameter('max_step_per_tick', 2.0)
        self.declare_parameter('invert_direction', False)
        self.declare_parameter('stale_timeout_sec', 0.5)
        self.declare_parameter('publish_hold_angle_when_lost', True)

        self.vision_topic = str(self.get_parameter('vision_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.tick_hz = float(self.get_parameter('tick_hz').value)
        self.target_class_id = int(self.get_parameter('target_class_id').value)
        self.min_score = float(self.get_parameter('min_score').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.deadband_y = float(self.get_parameter('deadband_y').value)
        self.initial_angle = float(self.get_parameter('initial_angle').value)
        self.min_angle = float(self.get_parameter('min_angle').value)
        self.max_angle = float(self.get_parameter('max_angle').value)
        self.kp = float(self.get_parameter('kp').value)
        self.max_step_per_tick = float(self.get_parameter('max_step_per_tick').value)
        self.invert_direction = bool(self.get_parameter('invert_direction').value)
        self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
        self.publish_hold_angle_when_lost = bool(
            self.get_parameter('publish_hold_angle_when_lost').value
        )

        self.current_angle = max(self.min_angle, min(self.max_angle, self.initial_angle))
        self._latest_vision_msg = None
        self._latest_vision_monotonic_ns = None
        self._vision_mutex = threading.Lock()
        self._has_logged_first_vision = False
        self._last_warn_monotonic_ns = 0
        self._last_published_angle = None

        self.angle_pub = self.create_publisher(Float64, self.output_topic, 10)
        self.vision_sub = self.create_subscription(
            TurtlebotDetection,
            self.vision_topic,
            self.vision_callback,
            10,
        )

        timer_period = 1.0 / self.tick_hz if self.tick_hz > 0.0 else 0.05
        self.timer = self.create_timer(timer_period, self.tick)

        self.get_logger().info(
            'mx_28_control started: '
            f'vision_topic={self.vision_topic}, output_topic={self.output_topic}, '
            f'tick_hz={self.tick_hz:.1f}, target_class_id={self.target_class_id}, '
            f'target_y={self.target_y:.1f}, deadband_y={self.deadband_y:.1f}, '
            f'angle_range=[{self.min_angle:.1f}, {self.max_angle:.1f}], '
            f'initial_angle={self.current_angle:.1f}, invert_direction={self.invert_direction}'
        )

    def vision_callback(self, msg: TurtlebotDetection):
        # 최신 비전만 저장
        with self._vision_mutex:
            self._latest_vision_msg = msg
            self._latest_vision_monotonic_ns = self.get_clock().now().nanoseconds

        if not self._has_logged_first_vision:
            self._has_logged_first_vision = True
            self.get_logger().info(f'Vision stream detected on {self.vision_topic}')

    def tick(self):
        # 저장된 비전으로 주기 제어
        with self._vision_mutex:
            vision_msg = self._latest_vision_msg
            vision_time_ns = self._latest_vision_monotonic_ns
        now_ns = self.get_clock().now().nanoseconds
        hold_msg = Float64()
        hold_msg.data = float(self.current_angle)

        if vision_msg is None or vision_time_ns is None:
            if now_ns - self._last_warn_monotonic_ns >= int(1.0 * 1e9):
                self.get_logger().warn('No vision data received yet')
                self._last_warn_monotonic_ns = now_ns
            if self.publish_hold_angle_when_lost:
                self.angle_pub.publish(hold_msg)
            return

        age_sec = (self.get_clock().now().nanoseconds - vision_time_ns) / 1e9
        if age_sec > self.stale_timeout_sec:
            if now_ns - self._last_warn_monotonic_ns >= int(1.0 * 1e9):
                self.get_logger().warn(
                    f'Vision data stale: age={age_sec:.2f}s > timeout={self.stale_timeout_sec:.2f}s'
                )
                self._last_warn_monotonic_ns = now_ns
            if self.publish_hold_angle_when_lost:
                self.angle_pub.publish(hold_msg)
            return

        new_angle = mx_28_controller(vision_msg, self.current_angle)
        if new_angle is None:
            if now_ns - self._last_warn_monotonic_ns >= int(1.0 * 1e9):
                self.get_logger().warn('No valid detection selected for MX-28 control')
                self._last_warn_monotonic_ns = now_ns
            if self.publish_hold_angle_when_lost:
                self.angle_pub.publish(hold_msg)
            return

        self.current_angle = new_angle
        msg = Float64()
        msg.data = float(new_angle)
        self.angle_pub.publish(msg)

        if self._last_published_angle is None or abs(self._last_published_angle - new_angle) >= 0.5:
            self.get_logger().info(f'Publishing MX-28 angle: {new_angle:.2f}')
            self._last_published_angle = new_angle


def main(args=None):
    rclpy.init(args=args)
    node = Mx28Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
