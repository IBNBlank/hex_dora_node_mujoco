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
    color_frames = {"head": None, "left": None, "right": None}
    depth_cmaps = {"head": None, "left": None, "right": None}
    node = Node(node_name)
    for event in node:
        if event["type"] == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                show_cnt += 1
                if show_cnt >= 8:
                    show_cnt = 0
                    for side in ("head", "left", "right"):
                        if color_frames[side] is not None:
                            cv2.imshow(f"{side}_color", color_frames[side])
                        if depth_cmaps[side] is not None:
                            cv2.imshow(f"{side}_depth", depth_cmaps[side])
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break

            elif event_id == "left_arm_motor":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[e3_desktop_test] left_arm_motor: "
                      f"pos={state['pos']}, vel={state['vel']}, "
                      f"eff={state['eff']}")

            elif event_id == "right_arm_motor":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[e3_desktop_test] right_arm_motor: "
                      f"pos={state['pos']}, vel={state['vel']}, "
                      f"eff={state['eff']}")

            elif event_id == "left_arm_end":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[e3_desktop_test] left_arm_end: "
                      f"pos={state['pos']}, quat={state['quat']}")

            elif event_id == "right_arm_end":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[e3_desktop_test] right_arm_end: "
                      f"pos={state['pos']}, quat={state['quat']}")

            elif event_id == "left_grip_motor":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[e3_desktop_test] left_grip_motor: "
                      f"pos={state['pos']}, vel={state['vel']}, "
                      f"eff={state['eff']}")

            elif event_id == "right_grip_motor":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[e3_desktop_test] right_grip_motor: "
                      f"pos={state['pos']}, vel={state['vel']}, "
                      f"eff={state['eff']}")

            elif event_id == "obj_pose":
                state = dict_decode(event["value"], event["metadata"])
                print(f"[e3_desktop_test] obj_pose: "
                      f"pos={state['pos']}, quat={state['quat']}")

            elif event_id == "head_color":
                color_frames["head"] = color_decode(event['value'],
                                                    event['metadata'])

            elif event_id == "head_depth":
                depth_frame = depth_decode(event['value'], event['metadata'])
                depth_cmaps["head"] = depth_to_cmap(depth_frame)

            elif event_id == "left_color":
                color_frames["left"] = color_decode(event['value'],
                                                    event['metadata'])

            elif event_id == "left_depth":
                depth_frame = depth_decode(event['value'], event['metadata'])
                depth_cmaps["left"] = depth_to_cmap(depth_frame)

            elif event_id == "right_color":
                color_frames["right"] = color_decode(event['value'],
                                                     event['metadata'])

            elif event_id == "right_depth":
                depth_frame = depth_decode(event['value'], event['metadata'])
                depth_cmaps["right"] = depth_to_cmap(depth_frame)

        elif event["type"] == "ERROR":
            raise RuntimeError(event["error"])

    cv2.destroyAllWindows()
