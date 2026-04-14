# hex_dora_node_mujoco

Dora nodes for running MuJoCo robot simulations. Supports Archer Y6 (single-arm) and E3 Desktop (dual-arm) models with state reading, command sending, and camera image streaming over the Dora dataflow.

## Nodes

| Node | Description | Inputs | Outputs |
| --- | --- | --- | --- |
| `hex-dora-mujoco-archer-y6` | Archer Y6 MuJoCo simulation (6-DOF arm with grip and camera) | `tick`, `arm_mit_cmd`, `arm_mit_comp_cmd`, `arm_pos_cmd`, `arm_pose_cmd`, `grip_mit_cmd`, `grip_mit_comp_cmd`, `grip_pos_cmd`, `reset` | `arm_motor`, `arm_end`, `grip_motor`, `obj_pose`, `color`, `depth` |
| `hex-dora-mujoco-e3-desktop` | E3 Desktop MuJoCo simulation (dual 6-DOF arm with grips and 3 cameras) | `tick`, `left_arm_*_cmd`, `right_arm_*_cmd`, `left_grip_*_cmd`, `right_grip_*_cmd`, `reset` | `left_arm_motor`, `right_arm_motor`, `left_arm_end`, `right_arm_end`, `left_grip_motor`, `right_grip_motor`, `obj_pose`, `head_color`, `head_depth`, `left_color`, `left_depth`, `right_color`, `right_depth` |
| `hex-dora-mujoco-test-archer-y6` | Archer Y6 state printer and image viewer for testing | `tick`, `arm_motor`, `arm_end`, `grip_motor`, `obj_pose`, `color`, `depth` | - |
| `hex-dora-mujoco-test-e3-desktop` | E3 Desktop state printer and image viewer for testing | `tick`, `left_arm_motor`, `right_arm_motor`, `left_arm_end`, `right_arm_end`, `left_grip_motor`, `right_grip_motor`, `obj_pose`, `head_color`, `head_depth`, `left_color`, `left_depth`, `right_color`, `right_depth` | - |

## Installation

```bash
pip install hex_dora_node_mujoco
```

## YAML Examples

### Archer Y6

```yaml
nodes:
  - id: mujoco_archer_y6
    build: pip install hex_dora_node_mujoco
    path: hex-dora-mujoco-archer-y6
    inputs:
      tick: dora/timer/millis/2
      arm_pos_cmd: planner/arm_pos_cmd
      grip_pos_cmd: planner/grip_pos_cmd
    outputs:
      - arm_motor
      - arm_end
      - grip_motor
      - obj_pose
      - color
      - depth
    env:
      NODE_NAME: mujoco_archer_y6
      STATE_RATE: 1000
      CAM_RATE: 30
      HEADLESS: False
      STATE_BUFFER_SIZE: 200
      CAM_BUFFER_SIZE: 8
      SEN_TS: False
      CAMERA_TYPE: usb
      COLOR_ENCODING: bgr8
      DEPTH_ENCODING: uint16

  - id: test_mujoco_archer_y6
    build: pip install hex_dora_node_mujoco
    path: hex-dora-mujoco-test-archer-y6
    inputs:
      tick: dora/timer/millis/2
      arm_motor: mujoco_archer_y6/arm_motor
      arm_end: mujoco_archer_y6/arm_end
      grip_motor: mujoco_archer_y6/grip_motor
      obj_pose: mujoco_archer_y6/obj_pose
      color: mujoco_archer_y6/color
      depth: mujoco_archer_y6/depth
    env:
      NODE_NAME: test_mujoco_archer_y6
```

### E3 Desktop

```yaml
nodes:
  - id: mujoco_e3_desktop
    build: pip install hex_dora_node_mujoco
    path: hex-dora-mujoco-e3-desktop
    inputs:
      tick: dora/timer/millis/2
      left_arm_pos_cmd: planner/left_arm_pos_cmd
      right_arm_pos_cmd: planner/right_arm_pos_cmd
      left_grip_pos_cmd: planner/left_grip_pos_cmd
      right_grip_pos_cmd: planner/right_grip_pos_cmd
    outputs:
      - left_arm_motor
      - right_arm_motor
      - left_arm_end
      - right_arm_end
      - left_grip_motor
      - right_grip_motor
      - obj_pose
      - head_color
      - head_depth
      - left_color
      - left_depth
      - right_color
      - right_depth
    env:
      NODE_NAME: mujoco_e3_desktop
      STATE_RATE: 1000
      CAM_RATE: 30
      HEADLESS: False
      STATE_BUFFER_SIZE: 200
      CAM_BUFFER_SIZE: 8
      SEN_TS: False
      HEAD_CAM_TYPE: usb
      LEFT_CAM_TYPE: empty
      RIGHT_CAM_TYPE: empty
      COLOR_ENCODING: bgr8
      DEPTH_ENCODING: uint16

  - id: test_mujoco_e3_desktop
    build: pip install hex_dora_node_mujoco
    path: hex-dora-mujoco-test-e3-desktop
    inputs:
      tick: dora/timer/millis/2
      left_arm_motor: mujoco_e3_desktop/left_arm_motor
      right_arm_motor: mujoco_e3_desktop/right_arm_motor
      left_arm_end: mujoco_e3_desktop/left_arm_end
      right_arm_end: mujoco_e3_desktop/right_arm_end
      left_grip_motor: mujoco_e3_desktop/left_grip_motor
      right_grip_motor: mujoco_e3_desktop/right_grip_motor
      obj_pose: mujoco_e3_desktop/obj_pose
      head_color: mujoco_e3_desktop/head_color
      head_depth: mujoco_e3_desktop/head_depth
      left_color: mujoco_e3_desktop/left_color
      left_depth: mujoco_e3_desktop/left_depth
      right_color: mujoco_e3_desktop/right_color
      right_depth: mujoco_e3_desktop/right_depth
    env:
      NODE_NAME: test_mujoco_e3_desktop
```

## Inputs

### Timer

| Input | Type | Description |
| --- | --- | --- |
| `tick` | `dora/timer/millis/*` | Timer tick to trigger state reading and image capture |

### Reset

| Input | Type | Description |
| --- | --- | --- |
| `reset` | any | Reset simulation to initial state |

### Arm Commands (Archer Y6)

| Input | Fields | Description |
| --- | --- | --- |
| `arm_mit_cmd` | `q_tar(6)`, `dq_tar(6)`, `tau(6)`, `kp(6)`, `kd(6)` | MIT torque command for arm |
| `arm_mit_comp_cmd` | `q_tar(6)`, `dq_tar(6)`, `tau(6)`, `kp(6)`, `kd(6)` | MIT command with gravity/friction compensation |
| `arm_pos_cmd` | `q_tar(6)`, `kp(6)`, `kd(6)`, `err_limit(1)` | Joint position command for arm |
| `arm_pose_cmd` | `pos(3)`, `quat(4)`, `kp(6)`, `kd(6)`, `err_limit(1)` | Cartesian pose command for arm |

### Grip Commands (Archer Y6)

| Input | Fields | Description |
| --- | --- | --- |
| `grip_mit_cmd` | `q_tar(1)`, `dq_tar(1)`, `tau(1)`, `kp(1)`, `kd(1)` | MIT torque command for grip |
| `grip_mit_comp_cmd` | `q_tar(1)`, `dq_tar(1)`, `tau(1)`, `kp(1)`, `kd(1)` | MIT command with friction compensation |
| `grip_pos_cmd` | `q_tar(1)`, `kp(1)`, `kd(1)`, `err_limit(1)` | Joint position command for grip |

### Arm Commands (E3 Desktop)

| Input | Fields | Description |
| --- | --- | --- |
| `left_arm_mit_cmd` / `right_arm_mit_cmd` | `q_tar(6)`, `dq_tar(6)`, `tau(6)`, `kp(6)`, `kd(6)` | MIT torque command for left/right arm |
| `left_arm_mit_comp_cmd` / `right_arm_mit_comp_cmd` | `q_tar(6)`, `dq_tar(6)`, `tau(6)`, `kp(6)`, `kd(6)` | MIT command with gravity/friction compensation |
| `left_arm_pos_cmd` / `right_arm_pos_cmd` | `q_tar(6)`, `kp(6)`, `kd(6)`, `err_limit(1)` | Joint position command for left/right arm |
| `left_arm_pose_cmd` / `right_arm_pose_cmd` | `pos(3)`, `quat(4)`, `kp(6)`, `kd(6)`, `err_limit(1)` | Cartesian pose command for left/right arm |

### Grip Commands (E3 Desktop)

| Input | Fields | Description |
| --- | --- | --- |
| `left_grip_mit_cmd` / `right_grip_mit_cmd` | `q_tar(1)`, `dq_tar(1)`, `tau(1)`, `kp(1)`, `kd(1)` | MIT torque command for left/right grip |
| `left_grip_mit_comp_cmd` / `right_grip_mit_comp_cmd` | `q_tar(1)`, `dq_tar(1)`, `tau(1)`, `kp(1)`, `kd(1)` | MIT command with friction compensation |
| `left_grip_pos_cmd` / `right_grip_pos_cmd` | `q_tar(1)`, `kp(1)`, `kd(1)`, `err_limit(1)` | Joint position command for left/right grip |

### Test Nodes (Archer Y6)

| Input | Type | Description |
| --- | --- | --- |
| `tick` | `dora/timer/millis/*` | Timer tick to refresh display |
| `arm_motor` | Arrow array | Arm motor state |
| `arm_end` | Arrow array | Arm end-effector state |
| `grip_motor` | Arrow array | Grip motor state |
| `obj_pose` | Arrow array | Object pose |
| `color` | Arrow array | Color image |
| `depth` | Arrow array | Depth image |

### Test Nodes (E3 Desktop)

| Input | Type | Description |
| --- | --- | --- |
| `tick` | `dora/timer/millis/*` | Timer tick to refresh display |
| `left_arm_motor` / `right_arm_motor` | Arrow array | Arm motor state |
| `left_arm_end` / `right_arm_end` | Arrow array | Arm end-effector state |
| `left_grip_motor` / `right_grip_motor` | Arrow array | Grip motor state |
| `obj_pose` | Arrow array | Object pose |
| `head_color` / `left_color` / `right_color` | Arrow array | Color image |
| `head_depth` / `left_depth` / `right_depth` | Arrow array | Depth image |

## Outputs

### `arm_motor` / `left_arm_motor` / `right_arm_motor`

`{"pos": float64[6], "vel": float64[6], "eff": float64[6]}`

### `arm_end` / `left_arm_end` / `right_arm_end`

`{"pos": float64[3], "quat": float64[4]}`

### `grip_motor` / `left_grip_motor` / `right_grip_motor`

`{"pos": float64[1], "vel": float64[1], "eff": float64[1]}`

### `obj_pose`

`{"pos": float64[3], "quat": float64[4]}`

### `color` / `head_color` / `left_color` / `right_color`

Arrow array containing the color image (224x224 BGR).

```python
color_data: UInt8Array  # pa.array(img.ravel()) or pa.array(jpeg_bytes)
metadata = {
    "width": int,       # 224
    "height": int,      # 224
    "encoding": str,    # "bgr8" or "mjpg"
    "primitive": "image",
}
```

### `depth` / `head_depth` / `left_depth` / `right_depth`

Arrow array containing the depth image (224x224, uint16, millimeters).

```python
depth_data: UInt16Array  # pa.array(depth.ravel()) or pa.array(png_bytes)
metadata = {
    "width": int,       # 224
    "height": int,      # 224
    "encoding": str,    # "uint16" or "png"
    "primitive": "image",
}
```

## Arrow Dict Encoding

All state data is transmitted as `dict[str, arr]` via flat `float64` Arrow arrays. The `metadata` dict describes the layout:

```python
from hex_util_runtime import dict_decode, dict_encode

state = dict_decode(event["value"], event["metadata"])
print(state["pos"], state["vel"])

import numpy as np
cmd = {
    "q_tar": np.zeros(6),
    "kp": np.array([400.0, 400.0, 500.0, 200.0, 100.0, 100.0]),
    "kd": np.array([5.0, 5.0, 5.0, 5.0, 2.0, 2.0]),
    "err_limit": np.array([0.03]),
}
storage, metadata = dict_encode(["q_tar", "kp", "kd", "err_limit"], cmd, event["metadata"])
node.send_output("arm_pos_cmd", storage, metadata)
```

## Environment Variables

### Common (simulation nodes)

| Variable | Type | Default | Description |
| --- | --- | --- | --- |
| `NODE_NAME` | `str` | `""` | Dora node name |
| `STATE_RATE` | `float` | `1000` | State publishing rate (Hz) |
| `CAM_RATE` | `float` | `30` | Camera image publishing rate (Hz) |
| `HEADLESS` | `bool` | `False` | Run without MuJoCo viewer |
| `STATE_BUFFER_SIZE` | `int` | `200` | Size of the state ring buffer |
| `CAM_BUFFER_SIZE` | `int` | `8` | Size of the camera image ring buffer |
| `SEN_TS` | `bool` | `False` | Use simulation timestamps |
| `COLOR_ENCODING` | `str` | `bgr8` | Color encoding: `bgr8` or `mjpg` |
| `DEPTH_ENCODING` | `str` | `uint16` | Depth encoding: `uint16` or `png` |

### Archer Y6

| Variable | Type | Default | Description |
| --- | --- | --- | --- |
| `CAMERA_TYPE` | `str` | `usb` | Camera type: `empty`, `usb`, `berxel`, or `realsense` |

### E3 Desktop

| Variable | Type | Default | Description |
| --- | --- | --- | --- |
| `HEAD_CAM_TYPE` | `str` | `empty` | Head camera type: `empty`, `usb`, `berxel`, or `realsense` |
| `LEFT_CAM_TYPE` | `str` | `empty` | Left camera type: `empty`, `usb`, `berxel`, or `realsense` |
| `RIGHT_CAM_TYPE` | `str` | `empty` | Right camera type: `empty`, `usb`, `berxel`, or `realsense` |

### Test Nodes

| Variable | Type | Default | Description |
| --- | --- | --- | --- |
| `NODE_NAME` | `str` | `""` | Dora node name |

## License

This project is licensed under Apache-2.0.
