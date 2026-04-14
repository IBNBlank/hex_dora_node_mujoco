#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2026-04-14
################################################################

import os, traceback
from dora import Node
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from hex_util_runtime import get_dora_node_name, get_dora_bool
from hex_util_runtime import dict_encode, dict_decode
from hex_util_runtime import color_encode, depth_encode
from hex_driver_mujoco import HexMujocoArcherY6, HexMujocoArcherY6Params


def state_func(sim, raw_meta, node):
    arm_motor = sim.get_arm_motor(latest=True)
    if arm_motor is not None:
        arm_motor_storage, arm_motor_metadata = dict_encode(
            ["pos", "vel", "eff"],
            arm_motor,
            raw_meta,
        )
        node.send_output("arm_motor", arm_motor_storage, arm_motor_metadata)

    arm_end = sim.get_arm_end(latest=True)
    if arm_end is not None:
        arm_end_storage, arm_end_metadata = dict_encode(
            ["pos", "quat"],
            arm_end,
            raw_meta,
        )
        node.send_output("arm_end", arm_end_storage, arm_end_metadata)

    grip_motor = sim.get_grip_motor(latest=True)
    if grip_motor is not None:
        grip_motor_storage, grip_motor_metadata = dict_encode(
            ["pos", "vel", "eff"],
            grip_motor,
            raw_meta,
        )
        node.send_output("grip_motor", grip_motor_storage,
                         grip_motor_metadata)

    obj_pose = sim.get_obj_pose(latest=True)
    if obj_pose is not None:
        obj_pose_storage, obj_pose_metadata = dict_encode(
            ["pos", "quat"],
            obj_pose,
            raw_meta,
        )
        node.send_output("obj_pose", obj_pose_storage, obj_pose_metadata)


def cam_encode_func(sim, raw_meta, color_encoding, depth_encoding):
    results = []
    color_frame = sim.get_color_img(latest=True)
    if color_frame is not None:
        color = color_frame["data"]
        storage, metadata = color_encode(color, color_encoding, raw_meta)
        if storage is not None:
            results.append(("color", storage, metadata))

    depth_frame = sim.get_depth_img(latest=True)
    if depth_frame is not None:
        depth = depth_frame["data"]
        storage, metadata = depth_encode(depth, depth_encoding, raw_meta)
        if storage is not None:
            results.append(("depth", storage, metadata))
    return results


def cmd_func(sim, arm_deque, grip_deque):
    last_arm_cmd = None
    while True:
        try:
            last_arm_cmd = arm_deque.popleft()
        except IndexError:
            break
    if last_arm_cmd is not None:
        if last_arm_cmd[0] == "mit":
            sim.set_arm_mit_cmd(last_arm_cmd[1])
        elif last_arm_cmd[0] == "mit_comp":
            sim.set_arm_mit_comp_cmd(last_arm_cmd[1])
        elif last_arm_cmd[0] == "pos":
            sim.set_arm_pos_cmd(last_arm_cmd[1])
        elif last_arm_cmd[0] == "pose":
            sim.set_arm_pose_cmd(last_arm_cmd[1])

    last_grip_cmd = None
    while True:
        try:
            last_grip_cmd = grip_deque.popleft()
        except IndexError:
            break
    if last_grip_cmd is not None:
        if last_grip_cmd[0] == "grip_mit":
            sim.set_grip_mit_cmd(last_grip_cmd[1])
        elif last_grip_cmd[0] == "grip_mit_comp":
            sim.set_grip_mit_comp_cmd(last_grip_cmd[1])
        elif last_grip_cmd[0] == "grip_pos":
            sim.set_grip_pos_cmd(last_grip_cmd[1])


def main():
    node_name = get_dora_node_name(os.getenv('NODE_NAME', ""))
    params = HexMujocoArcherY6Params(
        state_rate=float(os.getenv('STATE_RATE', '1000')),
        cam_rate=float(os.getenv('CAM_RATE', '30')),
        headless=get_dora_bool(os.getenv('HEADLESS', 'False')),
        state_buffer_size=int(os.getenv('STATE_BUFFER_SIZE', '200')),
        cam_buffer_size=int(os.getenv('CAM_BUFFER_SIZE', '8')),
        sens_ts=get_dora_bool(os.getenv('SEN_TS', 'False')),
        camera_type=os.getenv('CAMERA_TYPE', 'usb'),
    )
    color_encoding = os.getenv('COLOR_ENCODING', 'bgr8')
    depth_encoding = os.getenv('DEPTH_ENCODING', 'uint16')
    sim = None
    arm_deque = deque(maxlen=10)
    grip_deque = deque(maxlen=10)
    cam_executor = ThreadPoolExecutor(max_workers=1)
    cam_future = None
    try:
        sim = HexMujocoArcherY6(params)
        sim.start()

        node = Node(node_name)
        for event in node:
            if event["type"] == "INPUT":
                event_id = event["id"]

                if event_id == "tick":
                    raw_meta = event["metadata"]

                    if cam_future is not None and cam_future.done():
                        try:
                            for out_id, storage, metadata in cam_future.result():
                                node.send_output(out_id, storage, metadata)
                        except Exception:
                            traceback.print_exc()
                        cam_future = None

                    state_func(sim, raw_meta, node)
                    if cam_future is None:
                        cam_future = cam_executor.submit(
                            cam_encode_func, sim, raw_meta,
                            color_encoding, depth_encoding)
                    cmd_func(sim, arm_deque, grip_deque)

                elif event_id == "arm_mit_cmd":
                    cmd = dict_decode(event["value"], event["metadata"])
                    arm_deque.append(("mit", cmd))

                elif event_id == "arm_mit_comp_cmd":
                    cmd = dict_decode(event["value"], event["metadata"])
                    arm_deque.append(("mit_comp", cmd))

                elif event_id == "arm_pos_cmd":
                    cmd = dict_decode(event["value"], event["metadata"])
                    arm_deque.append(("pos", cmd))

                elif event_id == "arm_pose_cmd":
                    cmd = dict_decode(event["value"], event["metadata"])
                    arm_deque.append(("pose", cmd))

                elif event_id == "grip_mit_cmd":
                    cmd = dict_decode(event["value"], event["metadata"])
                    grip_deque.append(("grip_mit", cmd))

                elif event_id == "grip_mit_comp_cmd":
                    cmd = dict_decode(event["value"], event["metadata"])
                    grip_deque.append(("grip_mit_comp", cmd))

                elif event_id == "grip_pos_cmd":
                    cmd = dict_decode(event["value"], event["metadata"])
                    grip_deque.append(("grip_pos", cmd))

                elif event_id == "reset":
                    sim.reset()

            elif event["type"] == "ERROR":
                raise RuntimeError(event["error"])

    except Exception:
        traceback.print_exc()
    finally:
        cam_executor.shutdown(wait=False)
        if sim is not None:
            sim.stop()
            sim.deinit_mujoco()
