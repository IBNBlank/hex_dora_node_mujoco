#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-04-14
################################################################

import os
import cv2
from dora import Node
from hex_util_runtime import get_dora_node_name
from hex_util_runtime import dict_decode, color_decode, depth_decode
from hex_util_robot import depth_to_cmap


def main():
    node_name = get_dora_node_name(os.getenv('NODE_NAME', ""))
    os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu/"

    show_cnt = 0
    color_frame, depth_cmap = None, None
    node = Node(node_name)
    for event in node:
        if event["type"] == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                show_cnt += 1
                if show_cnt >= 8:
                    show_cnt = 0
                    if color_frame is not None:
                        cv2.imshow("color", color_frame)
                    if depth_cmap is not None:
                        cv2.imshow("depth", depth_cmap)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break

            elif event_id == "arm_motor":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[archer_y6_test] arm_motor: "
                      f"pos={state['pos']}, vel={state['vel']}, "
                      f"eff={state['eff']}")

            elif event_id == "arm_end":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[archer_y6_test] arm_end: "
                      f"pos={state['pos']}, quat={state['quat']}")

            elif event_id == "grip_motor":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[archer_y6_test] grip_motor: "
                      f"pos={state['pos']}, vel={state['vel']}, "
                      f"eff={state['eff']}")

            elif event_id == "obj_pose":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[archer_y6_test] obj_pose: "
                      f"pos={state['pos']}, quat={state['quat']}")

            elif event_id == "color":
                color_frame = color_decode(event['value'], event['metadata'])

            elif event_id == "depth":
                depth_frame = depth_decode(event['value'], event['metadata'])
                depth_cmap = depth_to_cmap(depth_frame)

        elif event["type"] == "ERROR":
            raise RuntimeError(event["error"])

    cv2.destroyAllWindows()
