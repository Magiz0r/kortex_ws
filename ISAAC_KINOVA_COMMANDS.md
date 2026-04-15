# Isaac Kinova Commands

Use this file when you want the shortest path to the right launch command.

## Common Setup

In every terminal:

```bash
cd /home/zty/workspace/another_kortex_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Open Isaac:

```bash
cd /home/zty/isaac-sim
./isaac-sim.selector.sh
```

Inside Isaac:

- open the Kinova stage
- press `Play`
- for this Kinova Isaac stage, drive the robot through `/isaac_joint_commands`
- do not rely on `SingleArticulation.apply_action()` or `set_joint_positions()` as the main control path

## Which Command To Use

### 1. Isaac Sim controls fake Kinova follower

Terminal 1:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_140_moveit_config robot.launch.py \
  robot_ip:=0.0.0.0 \
  use_fake_hardware:=true
```

Terminal 2:

```bash
ros2 launch kortex_bringup isaac_to_real.launch.py
```

Use this when:

- you are at home
- you want RViz + safe testing

### 2. MoveIt moves the real Kinova arm, Isaac mirrors it

Terminal 1:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_140_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10
```

Terminal 2:

```bash
ros2 launch kortex_bringup isaac_mirror.launch.py
```

Use this when:

- you want to move the real arm with MoveIt
- you want Isaac to show the same motion

Important:

- use MoveIt in RViz from the normal `robot.launch.py` stack
- do not run `isaac_to_real.launch.py` at the same time for this mode

### 3. Isaac Sim controls the real Kinova arm

Terminal 1:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_140_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10
```

Terminal 2:

```bash
ros2 launch kortex_bringup isaac_to_real.launch.py
```

Use this when:

- Isaac should drive the real arm

Important:

- start with very small Isaac motions
- if Isaac and the real arm are far apart, sync first with mode 4
- this does not require the real arm to start at `[0 0 0 0 0 0 0]`
- the real arm may start from an arbitrary pose as long as you sync Isaac first and then send a small first step

### 4. Sync Isaac to the current real arm pose first

Use this before mode 3 if Isaac and the real arm do not start aligned.

Terminal:

```bash
ros2 launch kortex_bringup isaac_mirror.launch.py
```

Use this when:

- the real arm is already somewhere else
- you want Isaac to match it before switching to Isaac control

After Isaac matches the real arm:

1. stop `isaac_mirror.launch.py`
2. start `isaac_to_real.launch.py`

Why this matters:

- the bridge checks the step size from the current real joint state to the next Isaac target
- if the first Isaac target is too far away, the bridge rejects it
- this is why arbitrary start poses work only after a short mirror-based handoff

### 5. Direct Kinova hand controller moves the real arm, Isaac mirrors it

Terminal:

```bash
ros2 launch kortex_bringup kinova_read_only_isaac_mirror.launch.py \
  robot_ip:=192.168.1.10
```

Use this when:

- your physical Kinova hand controller is plugged into the robot
- you want Isaac to follow the real arm
- you do not want ROS to take over robot motion

Important:

- do not use `robot.launch.py` for this mode
- do not run `isaac_to_real.launch.py` for this mode

## Quick Checks

Check Isaac state:

```bash
ros2 topic echo /isaac_joint_states --once
```

Check normal real/fake robot state:

```bash
ros2 topic echo /joint_states --once
```

Check read-only hand-controller mirror state:

```bash
ros2 topic echo /kinova_read_only/joint_states --once
```

Check what Isaac is receiving:

```bash
ros2 topic echo /isaac_joint_commands --once
```

If a Script Editor test does not move Isaac:

- confirm `/isaac_joint_commands` changes, not just `/isaac_joint_states`
- this stage is topic-controlled, so changing the articulation directly may not produce visible motion

## One-Line Decision Guide

- Isaac -> fake follower: `robot.launch.py use_fake_hardware:=true` + `isaac_to_real.launch.py`
- MoveIt -> real arm -> Isaac: `robot.launch.py robot_ip:=192.168.1.10` + `isaac_mirror.launch.py`
- Isaac -> real arm: `robot.launch.py robot_ip:=192.168.1.10` + `isaac_to_real.launch.py`
- real arm -> Isaac temporary sync before Isaac control: `isaac_mirror.launch.py`
- direct Kinova hand controller -> Isaac: `kinova_read_only_isaac_mirror.launch.py`
