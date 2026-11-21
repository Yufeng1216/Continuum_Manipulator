# Training data

`Training_Data.csv` has 9,463 rows and 23 columns. Each row is one recorded robot configuration: the actuator commands and the end-effector pose measured by the AR tag.

| Column | Meaning |
|---|---|
| `Servo1_Pos` … `Servo4_Pos` | Feetech STS3215 position (encoder counts) for the four C2 tendon winders |
| `Step_Motor1` … `Step_Motor4` | Accumulated stepper pulses of the four C1 linear slides, relative to the zero point |
| `Pressure_1`, `Pressure_2` | Target absolute pressure (Pa) of chamber 1 and chamber 2. `Pressure_1` is constant (110000 Pa) in this dataset |
| `AR_Pose_X`, `AR_Pose_Y`, `AR_Pose_Z` | AR-tag position in the camera frame (m) |
| `AR_Orientation_X/Y/Z/W` | AR-tag orientation as a quaternion |
| `L` | Arc length of the bending segment used for sampling (cm, 21–31) |
| `alpha` | Bending-plane direction α (rad, −π/2 to π/2) |
| `theta` | Bending angle θ (rad, 0 to 40°) |
| `X`, `Y`, `Z` | Copy of the AR-tag position from a later processing step. Empty in 1,502 rows |

The training scripts convert the quaternion to roll/pitch/yaw and use `[AR_Pose_X, AR_Pose_Y, AR_Pose_Z, roll, pitch, yaw]` as network input.
