#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-04-14
################################################################

import os
import dora
import cv2
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from hex_util_runtime import get_dora_node_name
from hex_util_runtime import dict_encode, dict_decode, color_decode, depth_decode
from hex_util_robot import depth_to_cmap

ARM_CTRL_CMDS = {
    "mit": {
        "type": "mit",
        "q_tar": np.array([0.0, -1.5, 3.0, 0.07, 0.0, 0.0]),
        "dq_tar": np.zeros(6),
        "tau": np.zeros(6),
        "kp": np.array([400.0, 400.0, 500.0, 200.0, 100.0, 100.0]),
        "kd": np.array([5.0, 5.0, 5.0, 5.0, 2.0, 2.0]),
    },
    "mit_comp": {
        "type": "mit_comp",
        "q_tar": np.zeros(6),
        "dq_tar": np.zeros(6),
        "tau": np.zeros(6),
        "kp": np.zeros(6),
        "kd": np.zeros(6),
    },
    "pos": {
        "type": "pos",
        "q_tar": np.array([0.0, -1.5, 3.0, 0.07, 0.0, 0.0]),
        "kp": np.array([400.0, 400.0, 500.0, 200.0, 100.0, 100.0]),
        "kd": np.array([5.0, 5.0, 5.0, 5.0, 2.0, 2.0]),
        "err_limit": 0.03,
    },
    "pose": {
        "type": "pose",
        "pos": np.array([0.3, 0.0, 0.3]),
        "quat": np.array([1.0, 0.0, 0.0, 0.0]),
        "kp": np.array([400.0, 400.0, 500.0, 200.0, 100.0, 100.0]),
        "kd": np.array([5.0, 5.0, 5.0, 5.0, 2.0, 2.0]),
        "err_limit": 0.03,
    },
}

GRIP_CTRL_CMDS = {
    "mit": {
        "type": "mit",
        "q_tar": np.array([0.5]),
        "dq_tar": np.array([0.0]),
        "tau": np.array([0.0]),
        "kp": np.array([10.0]),
        "kd": np.array([0.5]),
    },
    "mit_comp": {
        "type": "mit_comp",
        "q_tar": np.zeros(1),
        "dq_tar": np.zeros(1),
        "tau": np.zeros(1),
        "kp": np.zeros(1),
        "kd": np.zeros(1),
    },
    "pos": {
        "type": "pos",
        "q_tar": np.array([0.5]),
        "kp": np.array([10.0]),
        "kd": np.array([0.5]),
        "err_limit": 0.01,
    },
}


def build_zero_cmd(fields):
    keys = [f[0] for f in fields]
    data = {f[0]: np.zeros(f[1]) for f in fields}
    return keys, data


def main():
    node_name = get_dora_node_name(os.getenv('NODE_NAME', ""))
    os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu/"

    arm_ctrl_mode = os.getenv('ARM_CTRL_MODE', 'pos')
    grip_ctrl_mode = os.getenv('GRIP_CTRL_MODE', 'pos')
    arm_cmd = ARM_CTRL_CMDS.get(arm_ctrl_mode, ARM_CTRL_CMDS["pos"])
    grip_cmd = GRIP_CTRL_CMDS.get(grip_ctrl_mode, GRIP_CTRL_CMDS["pos"])
    print(f"[archer_y6_test] arm_ctrl_mode={arm_ctrl_mode}, "
          f"grip_ctrl_mode={grip_ctrl_mode}")

    show_cnt = 0
    print_cnt = 0
    color_frame, depth_cmap = None, None
    color_future, depth_future = None, None
    arm_motor_state, arm_end_state = None, None
    grip_motor_state, obj_pose_state = None, None
    node = dora.Node(node_name)
    executor = ThreadPoolExecutor(max_workers=2)
    for event in node:
        if event["type"] == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                raw_meta = event["metadata"]
                storage, metadata = dict_encode(arm_cmd.keys(), arm_cmd,
                                                dict(raw_meta))
                node.send_output("arm_cmd", storage, metadata)
                storage, metadata = dict_encode(grip_cmd.keys(), grip_cmd,
                                                dict(raw_meta))
                node.send_output("grip_cmd", storage, metadata)

                print_cnt += 1
                if print_cnt >= 100:
                    print_cnt = 0
                    if arm_motor_state is not None:
                        print(f"[archer_y6_test] arm_motor: "
                              f"pos={arm_motor_state['pos']}, "
                              f"vel={arm_motor_state['vel']}, "
                              f"eff={arm_motor_state['eff']}")
                    if arm_end_state is not None:
                        print(f"[archer_y6_test] arm_end: "
                              f"pos={arm_end_state['pos']}, "
                              f"quat={arm_end_state['quat']}")
                    if grip_motor_state is not None:
                        print(f"[archer_y6_test] grip_motor: "
                              f"pos={grip_motor_state['pos']}, "
                              f"vel={grip_motor_state['vel']}, "
                              f"eff={grip_motor_state['eff']}")
                    if obj_pose_state is not None:
                        print(f"[archer_y6_test] obj_pose: "
                              f"pos={obj_pose_state['pos']}, "
                              f"quat={obj_pose_state['quat']}")

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
                arm_motor_state = dict_decode(event["value"],
                                              event["metadata"])

            elif event_id == "arm_end":
                arm_end_state = dict_decode(event["value"], event["metadata"])

            elif event_id == "grip_motor":
                grip_motor_state = dict_decode(event["value"],
                                               event["metadata"])

            elif event_id == "obj_pose":
                obj_pose_state = dict_decode(event["value"], event["metadata"])

            elif event_id == "color":
                color_future = executor.submit(color_decode, event['value'],
                                               event['metadata'])

            elif event_id == "depth":
                depth_future = executor.submit(depth_decode, event['value'],
                                               event['metadata'])

            if color_future is not None and color_future.done():
                color_frame = color_future.result()
                color_future = None
            if depth_future is not None and depth_future.done():
                depth_frame = depth_future.result()
                depth_cmap = depth_to_cmap(depth_frame)
                depth_future = None

        elif event["type"] == "ERROR":
            raise RuntimeError(event["error"])

    executor.shutdown(wait=False)
    cv2.destroyAllWindows()
