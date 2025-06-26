#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
esp_bridge.py  •  LiDAR (+ yaw / mirror / static TF)  •  /cmd_vel → ESP  •  /odom  (ROS 2 Jazzy)

* WebSocket-скан от ESP32  →  /lidar_scan   (sensor_msgs/LaserScan)
* REST-команда /setSpeed   ← /cmd_vel (geometry_msgs/Twist)
* HTTP /state → /odom + TF (nav_msgs/Odometry + динамический odom→base_link)

Дополнительные параметры конфигурируют датчик:

  lidar_yaw_deg     (double, ° CW) – повернуть облако
  lidar_mirror      (bool)         – отразить (лево↔право)
  lidar_xyz         (double[3])    – смещение лидара в базе  [м]
  lidar_rpy_deg     (double[3])    – ориентация лидара       [° R P Y]

При старте узел публикует *статический* трансформ base_link→laser,
поэтому RViz/ Nav2 сразу видят полную цепочку `odom → base_link → laser`.
"""

import asyncio
import math
import struct
import threading
import socket
import urllib.parse
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    TransformStamped,
)
import websockets

# ――― константы LDS пакетов ―――――――――――――――――――――――――――――――――――――――――――――――
FRAME_FMT = "<HHHHHHHHHH"          # 20 B – угол начала, угол конца, 8×дистанция
FRAME_SZ = 20
CRC_SZ = 2
MAX_PKTS = 64                      # макс. пакетов на оборот
MAX_SIZE = MAX_PKTS * FRAME_SZ + CRC_SZ

BASE_MM = 96.0                     # база робота (колёс), мм


def crc16(buf: bytes, crc: int = 0xFFFF) -> int:
    """Modbus-совместимый CRC-16"""
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc & 0xFFFF


def q_yaw(yaw: float) -> Quaternion:
    """Quaternion из Yaw (рад)"""
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """RPY (рад) → Quaternion"""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


# ―――― основной узел ―――――――――――――――――――――――――――――――――――――――――――――――――――
class ESPBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp_bridge")

        # ── параметры подключения ──────────────────────────────────
        self.declare_parameter("host", "192.168.0.105")
        self.declare_parameter("ws_port", 80)
        self.declare_parameter("ws_path", "/ws")

        # ── имена фреймов и диапазон LIDAR-a ───────────────────────
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_range_min", 0.05)
        self.declare_parameter("lidar_range_max", 2.0)

        # ── дальномер ───────────────────────
        self.declare_parameter("rangefinder_range_min", 0.05)
        self.declare_parameter("rangefinder_range_max", 1.2)
        self.declare_parameter("rangefinder_id", "range")
        self.declare_parameter("rangefinder_xyz", [0.04, 0.0, 0.02])
        self.declare_parameter("rangefinder_rpy_deg", [0.0, 0.0, 0.0])

        # ── настройка ориентации LiDAR-а ───────────────────────────
        self.declare_parameter("lidar_yaw_deg", 180.0)        # сдвиг облака CW
        self.declare_parameter("lidar_mirror", True)          # зеркально (True/False)
        self.declare_parameter("lidar_xyz", [0.0, 0.0, 0.10]) # положение на раме
        self.declare_parameter("lidar_rpy_deg", [0.0, 0.0, 0.0])  # ориентация
        

        # ── формируем URL WebSocket ――
        host = self.get_parameter("host").value
        port = self.get_parameter("ws_port").value
        path = self.get_parameter("ws_path").value
        self.ws_url = f"ws://{host}:{port}{path}"

        # ── ROS паблишеры/сабскрайберы ────────────────────────────
        self.scan_pub = self.create_publisher(LaserScan, "/lidar_scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.range_pub = self.create_publisher(LaserScan, "/range", 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

        # динамический TF (odom→base_link) + статический (base_link→laser)
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self.static_br2 = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # ― запускаем отдельный event-loop для WebSocket (лидар) ―
        self.ws_loop = asyncio.new_event_loop()
        threading.Thread(
            target=self._run_ws_loop, args=(self.ws_loop,), daemon=True
        ).start()

        # ― периодический HTTP-опрос /state ―
        self.create_timer(0.10, self.poll_state)

        self.get_logger().info(f"LiDAR WebSocket →  {self.ws_url}")

    # =============================================================
    #                 статический TF  base_link → laser
    # =============================================================
    def _publish_static_tf(self) -> None:
        xyz = self.get_parameter("lidar_xyz").value
        rpy_deg = self.get_parameter("lidar_rpy_deg").value

        roll, pitch, yaw = [math.radians(a) for a in rpy_deg]
        q = rpy_to_quat(roll, pitch, yaw)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.get_parameter("base_frame").value
        tf.child_frame_id = self.get_parameter("frame_id").value
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation = q
        # latched – публикуем один раз
        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF base_link → laser")

        xyz = self.get_parameter("rangefinder_xyz").value
        rpy_deg = self.get_parameter("rangefinder_rpy_deg").value
        roll, pitch, yaw = [math.radians(a) for a in rpy_deg]
        q = rpy_to_quat(roll, pitch, yaw)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.get_parameter("base_frame").value
        tf.child_frame_id = self.get_parameter("rangefinder_id").value
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation = q
        # latched – публикуем один раз
        self.static_br2.sendTransform(tf)
        self.get_logger().info("Published static TF base_link → range")

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = rpy_to_quat(0.0,0.0,0.0)

        # latched – публикуем один раз
        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF map → odom")

    # =============================================================
    #                    WebSocket (LiDAR поток)
    # =============================================================
    def _run_ws_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.ws_task())

    async def ws_task(self) -> None:
        while rclpy.ok():
            try:
                async with websockets.connect(
                    self.ws_url, max_size=None, ping_interval=None
                ) as ws:
                    # disable Nagle
                    sock: socket.socket = ws.transport.get_extra_info("socket")
                    if sock:
                        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.get_logger().info("WS connected")

                    while rclpy.ok():
                        buf = await ws.recv()
                        if not isinstance(buf, (bytes, bytearray)):
                            continue
                        if not (FRAME_SZ + CRC_SZ <= len(buf) <= MAX_SIZE):
                            continue
                        if crc16(buf[:-2]) != int.from_bytes(buf[-2:], "little"):
                            continue
                        self.publish_scan(buf[:-2])

            except Exception as e:
                self.get_logger().warn(f"WS error: {e}")
                await asyncio.sleep(1.0)

    # =============================================================
    #               /cmd_vel  →  HTTP /setSpeed
    # =============================================================
    def cmd_cb(self, msg: Twist) -> None:
        v_mm = msg.linear.x * 1000.0
        w = msg.angular.z
        left = int(v_mm - w * BASE_MM / 2.0)
        right = int(v_mm + w * BASE_MM / 2.0)

        host = self.get_parameter("host").value
        qs = "/setSpeed?" + urllib.parse.urlencode({"l": left, "r": right})
        try:
            requests.get(f"http://{host}{qs}", timeout=0.15)
        except Exception as e:
            self.get_logger().warn(f"/setSpeed fail: {e}")

    # =============================================================
    #               преобразуем пакеты LDS → LaserScan
    # =============================================================
    def publish_scan(self, raw: bytes) -> None:
        pkt_cnt = len(raw) // FRAME_SZ
        if pkt_cnt < 38:  # < ~270°
            return

        ang, rng, inten = [], [], []
        prev = None
        off = 0.0

        for i in range(pkt_cnt):
            s, e, *dist = struct.unpack_from(FRAME_FMT, raw, i * FRAME_SZ)
            s /= 100.0
            e /= 100.0
            if e < s:
                e += 360.0

            for j, d in enumerate(dist):
                a = s + (e - s) * j / 7
                if prev is not None and a + off < prev - 300:
                    off += 360.0
                a += off
                prev = a

                ang.append(a)
                rng.append(d / 1000.0 if d else float("inf"))
                inten.append(1.0 if d else 0.0)

        # зеркалим?
        if self.get_parameter("lidar_mirror").value:
            ang.reverse()
            rng.reverse()
            inten.reverse()
            ang = [-a for a in ang]

        # добавляем поворот
        yaw_deg = self.get_parameter("lidar_yaw_deg").value
        if yaw_deg:
            ang = [a + yaw_deg for a in ang]

        # нормализуем, чтобы начало ≈ −π //
        if ang[0] > 180.0:
            ang = [a - 360.0 for a in ang]

        rad = [math.radians(a) for a in ang]

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.get_parameter("frame_id").value
        scan.angle_min = rad[0]
        scan.angle_max = rad[-1]
        scan.angle_increment = (rad[-1] - rad[0]) / (len(rad) - 1)
        scan.range_min = self.get_parameter("lidar_range_min").value
        scan.range_max = self.get_parameter("lidar_range_max").value
        scan.ranges = rng
        scan.intensities = inten
        self.scan_pub.publish(scan)

    # =============================================================
    #                 HTTP /state → /odom  (+ TF)
    # =============================================================
    def poll_state(self) -> None:
        host = self.get_parameter("host").value
        try:
            js = requests.get(f"http://{host}/state", timeout=0.25).json()
        except Exception:
            return

        x, y, th = js["odom"]["x"], js["odom"]["y"], js["odom"]["th"]
        rng = js["rangefinder"]["value"]/1000

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = self.get_parameter("odom_frame").value
        od.child_frame_id = self.get_parameter("base_frame").value
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation = q_yaw(th)
        od.twist.twist.linear.x = js["speed"]["left"] / 1000.0
        od.twist.twist.angular.z = (
            js["speed"]["right"] - js["speed"]["left"]
        ) / BASE_MM * 1000.0
        self.odom_pub.publish(od)

        tf = TransformStamped()
        tf.header.stamp = od.header.stamp
        tf.header.frame_id = od.header.frame_id
        tf.child_frame_id = od.child_frame_id
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = od.pose.pose.orientation
        self.tf_br.sendTransform(tf)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.get_parameter("rangefinder_id").value
        scan.angle_min = 0.0
        scan.angle_max = 0.001
        scan.angle_increment = 0.001
        scan.range_min = self.get_parameter("rangefinder_range_min").value
        scan.range_max = self.get_parameter("rangefinder_range_max").value
        scan.ranges = [rng] * 2
        scan.intensities = [1.0 if rng<1.1 else 0.0] * 2
        self.range_pub.publish(scan)


# ――― точка входа ――――――――――――――――――――――――――――――――――――――――――――――――――
def main(args=None) -> None:
    rclpy.init(args=args)
    rclpy.spin(ESPBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
