#!/usr/bin/env python3
# Copyright 2026 wolf26_nav
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""周期性把当前地图存盘。

建图或长时间调试时防止崩溃丢图：每 ``save_interval`` 秒调用一次 nav2 map_saver
的 ``save_map`` 服务，把当前地图写成 ``<map_save_path>.pgm`` + ``<map_save_path>.yaml``。

``map_save_path`` 是**输出前缀**（不含扩展名），同名覆盖，所以磁盘占用不会随时
间增长——它保的是「最近一次快照」，不是历史版本。

存图服务还没就绪（``map_saver_server`` 没起来或没被 lifecycle manager 激活）时
只告警、不退出，等下一轮再试。
"""

import os

import rclpy
from nav2_msgs.srv import SaveMap
from rclpy.node import Node


class PeriodicMapSaver(Node):
    """定时调用 ``map_saver/save_map`` 的前缀存图器。"""

    def __init__(self):
        super().__init__("periodic_map_saver")

        self.declare_parameter("map_save_path", "~/sentry26_maps/map")
        self.declare_parameter("save_interval", 60.0)
        self.declare_parameter("save_map_service", "map_saver/save_map")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("image_format", "pgm")
        self.declare_parameter("map_mode", "trinary")
        self.declare_parameter("free_thresh", 0.25)
        self.declare_parameter("occupied_thresh", 0.65)

        self._path_prefix = os.path.expanduser(
            str(self.get_parameter("map_save_path").value)
        )
        self._in_flight = False

        self._client = self.create_client(
            SaveMap, str(self.get_parameter("save_map_service").value)
        )

        interval = float(self.get_parameter("save_interval").value)
        self._timer = self.create_timer(interval, self._save_once)
        self.get_logger().info(
            f"每 {interval:.0f} s 存一次图：{self._path_prefix}.pgm / .yaml"
        )

    def _save_once(self):
        if self._in_flight:
            # 上一轮还没回来（存大图可能比 save_interval 还慢），跳过本轮。
            return

        if not self._client.service_is_ready():
            self.get_logger().warn(
                f"存图服务 {self._client.srv_name} 未就绪"
                "（map_saver_server 没起来或未被 lifecycle manager 激活），本轮跳过",
                throttle_duration_sec=120.0,
            )
            return

        try:
            os.makedirs(os.path.dirname(self._path_prefix) or ".", exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f"创建存图目录失败：{exc}")
            return

        request = SaveMap.Request()
        request.map_topic = str(self.get_parameter("map_topic").value)
        request.map_url = self._path_prefix
        request.image_format = str(self.get_parameter("image_format").value)
        request.map_mode = str(self.get_parameter("map_mode").value)
        request.free_thresh = float(self.get_parameter("free_thresh").value)
        request.occupied_thresh = float(self.get_parameter("occupied_thresh").value)

        self._in_flight = True
        self._client.call_async(request).add_done_callback(self._on_saved)

    def _on_saved(self, future):
        self._in_flight = False
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - 服务端异常不该带崩存图定时器
            self.get_logger().error(f"存图失败：{exc}")
            return

        if result.result:
            self.get_logger().info(f"已存图：{self._path_prefix}.pgm / .yaml")
        else:
            self.get_logger().warn("存图服务返回失败（多半是这一轮没收到地图消息）")


def main(args=None):
    rclpy.init(args=args)
    node = PeriodicMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
