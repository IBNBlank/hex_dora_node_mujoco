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
from hex_driver_mujoco import HexMujocoE3Desktop, HexMujocoE3DesktopParams


def state_func(sim, raw_meta, node):
    for side in ("left", "right"):
        arm_motor = getattr(sim, f"get_{side}_arm_motor")(latest=True)
        if arm_motor is not None:
            storage, metadata = dict_encode(
                ["pos", "vel", "eff"],
                arm_motor,
                dict(raw_meta),
            )
            node.send_output(f"{side}_arm_motor", storage, metadata)

        arm_end = getattr(sim, f"get_{side}_arm_end")(latest=True)
        if arm_end is not None:
            storage, metadata = dict_encode(
                ["pos", "quat"],
                arm_end,
                dict(raw_meta),
            )
            node.send_output(f"{side}_arm_end", storage, metadata)

        grip_motor = getattr(sim, f"get_{side}_grip_motor")(latest=True)
        if grip_motor is not None:
            storage, metadata = dict_encode(
                ["pos", "vel", "eff"],
                grip_motor,
                dict(raw_meta),
            )
            node.send_output(f"{side}_grip_motor", storage, metadata)

    obj_pose = sim.get_obj_pose(latest=True)
    if obj_pose is not None:
        obj_pose_storage, obj_pose_metadata = dict_encode(
            ["pos", "quat"],
            obj_pose,
            dict(raw_meta),
        )
        node.send_output("obj_pose", obj_pose_storage, obj_pose_metadata)


def cam_encode_func(sim, raw_meta, color_encoding, depth_encoding):
    results = []
    for side in ("head", "left", "right"):
        color_frame = getattr(sim, f"get_{side}_color_img")(latest=True)
        if color_frame is not None:
            color = color_frame["data"]
            storage, metadata = color_encode(color, color_encoding, dict(raw_meta))
            if storage is not None:
                results.append((f"{side}_color", storage, metadata))

        depth_frame = getattr(sim, f"get_{side}_depth_img")(latest=True)
        if depth_frame is not None:
            depth = depth_frame["data"]
            storage, metadata = depth_encode(depth, depth_encoding, dict(raw_meta))
            if storage is not None:
                results.append((f"{side}_depth", storage, metadata))
    return results


def cmd_func(sim, arm_deque_dict, grip_deque_dict):
    for side in ("left", "right"):
        last_arm_cmd = None
        while True:
            try:
                last_arm_cmd = arm_deque_dict[side].popleft()
            except IndexError:
                break
        if last_arm_cmd is not None:
            if last_arm_cmd["type"] == "mit":
                getattr(sim, f"set_{side}_arm_mit_cmd")(last_arm_cmd)
            elif last_arm_cmd["type"] == "mit_comp":
                getattr(sim, f"set_{side}_arm_mit_comp_cmd")(last_arm_cmd)
            elif last_arm_cmd["type"] == "pos":
                getattr(sim, f"set_{side}_arm_pos_cmd")(last_arm_cmd)
            elif last_arm_cmd["type"] == "pose":
                getattr(sim, f"set_{side}_arm_pose_cmd")(last_arm_cmd)

        last_grip_cmd = None
        while True:
            try:
                last_grip_cmd = grip_deque_dict[side].popleft()
            except IndexError:
                break
        if last_grip_cmd is not None:
            if last_grip_cmd["type"] == "mit":
                getattr(sim, f"set_{side}_grip_mit_cmd")(last_grip_cmd)
            elif last_grip_cmd["type"] == "mit_comp":
                getattr(sim, f"set_{side}_grip_mit_comp_cmd")(last_grip_cmd)
            elif last_grip_cmd["type"] == "pos":
                getattr(sim, f"set_{side}_grip_pos_cmd")(last_grip_cmd)


def main():
    node_name = get_dora_node_name(os.getenv('NODE_NAME', ""))
    params = HexMujocoE3DesktopParams(
        state_rate=float(os.getenv('STATE_RATE', '1000')),
        cam_rate=float(os.getenv('CAM_RATE', '30')),
        headless=get_dora_bool(os.getenv('HEADLESS', 'False')),
        state_buffer_size=int(os.getenv('STATE_BUFFER_SIZE', '200')),
        cam_buffer_size=int(os.getenv('CAM_BUFFER_SIZE', '8')),
        sens_ts=get_dora_bool(os.getenv('SEN_TS', 'False')),
        cam_type={
            "head": os.getenv('HEAD_CAM_TYPE', 'empty'),
            "left": os.getenv('LEFT_CAM_TYPE', 'empty'),
            "right": os.getenv('RIGHT_CAM_TYPE', 'empty'),
        },
    )
    color_encoding = os.getenv('COLOR_ENCODING', 'bgr8')
    depth_encoding = os.getenv('DEPTH_ENCODING', 'uint16')
    sim = None
    arm_deque_dict = {
        "left": deque(maxlen=10),
        "right": deque(maxlen=10),
    }
    grip_deque_dict = {
        "left": deque(maxlen=10),
        "right": deque(maxlen=10),
    }
    cam_executor = ThreadPoolExecutor(max_workers=1)
    cam_future = None
    try:
        sim = HexMujocoE3Desktop(params)
        sim.start()

        node = Node(node_name)
        for event in node:
            if event["type"] == "INPUT":
                event_id = event["id"]

                if event_id == "tick":
                    raw_meta = event["metadata"]

                    if cam_future is not None and cam_future.done():
                        try:
                            for out_id, storage, metadata in cam_future.result(
                            ):
                                node.send_output(out_id, storage, metadata)
                        except Exception:
                            traceback.print_exc()
                        cam_future = None

                    state_func(sim, raw_meta, node)
                    if cam_future is None:
                        cam_future = cam_executor.submit(
                            cam_encode_func, sim, raw_meta, color_encoding,
                            depth_encoding)
                    cmd_func(sim, arm_deque_dict, grip_deque_dict)

                elif event_id == "left_arm_cmd":
                    arm_deque_dict["left"].append(
                        dict_decode(event["value"], event["metadata"]))

                elif event_id == "right_arm_cmd":
                    arm_deque_dict["right"].append(
                        dict_decode(event["value"], event["metadata"]))

                elif event_id == "left_grip_cmd":
                    grip_deque_dict["left"].append(
                        dict_decode(event["value"], event["metadata"]))

                elif event_id == "right_grip_cmd":
                    grip_deque_dict["right"].append(
                        dict_decode(event["value"], event["metadata"]))

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
