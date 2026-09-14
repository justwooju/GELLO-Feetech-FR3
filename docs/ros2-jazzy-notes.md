# Running GELLO's ROS 2 packages on Jazzy

GELLO's `ros2/` directory targets Humble. This is what it took to get it running on
Ubuntu 24.04 / Jazzy with `franka_ros2` v3.4.1, `libfranka` 0.20.5 and
`ros2_control` 4.47.0.

Tested against GELLO commit `fa0407bb` (2026-04-07).

## Build

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

`-DBUILD_TESTING=OFF` is required. `test/test_joint_impedance_controller.cpp` calls
`controller_->init("single_arm_controller_test")`, but `ControllerInterfaceBase::init`
now takes either five arguments or a `ControllerInterfaceParams`. The runtime code
compiles fine — `joint_impedance_controller.cpp` and `motion_generator.cpp` are 399
lines between them and use only `on_init`, `on_configure`, `update`,
`command_interface_configuration`, `state_interface_configuration` and
`interface_configuration_type::INDIVIDUAL`, all of which are stable.

Also worth knowing: `franka_semantic_components` appears in `CMakeLists.txt` but not in
`package.xml`, and no source file actually uses it. `rosdep` will not resolve it. It can
be dropped.

## Fix 1 — `k_alpha` is never declared

`on_init()`:

```cpp
auto_declare<std::string>("arm_id", "");
auto_declare<std::vector<double>>("k_gains", {});
auto_declare<std::vector<double>>("d_gains", {});
```

`on_configure()`, line 147:

```cpp
auto k_alpha = get_node()->get_parameter("k_alpha").as_double();
```

Older `ros2_control` implicitly declared parameters that appeared in a params file.
4.47 does not, so configure throws:

```
[ERROR] [joint_impedance_controller]: Caught exception in callback for transition 10
[ERROR] [joint_impedance_controller]: Original error: k_alpha
```

`ros2 param set` does not help — an undeclared parameter cannot be set. The declaration
has to be added:

```cpp
auto_declare<double>("k_alpha", 0.99);
```

See `patches/01-k_alpha-declare.patch`.

## Fix 2 — duplicate `/**:` in `controllers.yaml`

The shipped file is:

```yaml
/**:
  controller_manager:
    ros__parameters:
      joint_impedance_controller:
        type: franka_fr3_arm_controllers/JointImpedanceController
      ...

/**:                       # <- second block with the same key
  joint_impedance_controller:
    ros__parameters:
      k_alpha: 0.99
      ...
```

YAML keeps the last duplicate key. The `controller_manager` block — containing every
controller `type` — disappears. The launch then fails with:

```
[ERROR] [controller_manager]: The 'type' param was not defined for 'joint_impedance_controller'.
```

Merging the two blocks under one `/**:` fixes it. See
`patches/02-controllers-yaml-merge.md`.

This is the reason the standard launch file fails first; the `k_alpha` error only shows
up once you get past it.

## Fix 3 — `arm_id` is bound at configure time

Interface names (`fr3_joint1/effort`) are built during `on_configure`. Setting `arm_id`
afterwards has no effect on an already-configured controller:

```
Unable to activate controller 'joint_impedance_controller' since the command interface
'_joint1/effort' is not available.
```

The leading underscore is the empty `arm_id`. The controller has to be cycled:

```bash
ros2 control set_controller_state joint_impedance_controller unconfigured
ros2 param set /joint_impedance_controller arm_id fr3
ros2 control set_controller_state joint_impedance_controller inactive
ros2 control switch_controllers --activate joint_impedance_controller
```

Putting `arm_id: fr3` in `controllers.yaml` avoids this entirely.

## Bring-up order

Hardware first, then controller, then gripper.

```bash
# terminal A — hardware
ros2 launch franka_bringup franka.launch.py \
  robot_type:=fr3 robot_ip:=<ip> load_gripper:=true use_fake_hardware:=false

# terminal B — leader publisher
ros2 launch franka_gello_state_publisher main.launch.py config_file:=<your>.yaml

# terminal C — controller.  the robot moves here.
ros2 run controller_manager spawner joint_impedance_controller \
  --param-file <path>/controllers.yaml

# terminal D — gripper
ros2 launch franka_gripper_manager franka_gripper_client.launch.py \
  config_file:=example_fr3_config_franka_hand.yaml
```

The gripper client waits on the `franka_gripper/homing` action server, which
`franka_bringup` provides. Starting it before the hardware gives:

```
RuntimeError: Homing action server not available after 10.0 seconds!
```

Source the overlay workspace in every terminal that touches the controller. Without it
`pluginlib` cannot find `franka_fr3_arm_controllers/JointImpedanceController`.

## Two failure modes worth naming

**Orphaned `robot_state_publisher`.** Ctrl-C on a launch file does not always take its
children with it. Several `robot_state_publisher` processes publishing
`robot_description` leaves `controller_manager` stuck:

```
[WARN] [controller_manager]: Waiting for data on 'robot_description' topic to finish initialization
```

repeating forever, with `/controller_manager/list_controllers` never answering. Check:

```bash
ros2 topic info /robot_description | grep -i "publisher count"    # want 1
ps aux | grep -e ros2_control_node -e robot_state_publisher | grep -v grep
```

**FCI holds one connection.** `libfranka` allows a single control connection. Any other
`ros2_control_node` on the same robot has to be down first. The gripper connects on port
1338 and the arm on 1337 — if 1338 is established but 1337 is not, the gripper is up and
the arm is not:

```bash
ss -tnp | grep -e ":1337" -e ":1338"
```

## DDS

If the robot PC shares a LAN with other ROS 2 machines, set `ROS_DOMAIN_ID` to something
unused. Default domain 0 will discover every node on the subnet, and duplicate node
names make service calls non-deterministic.

Multicast discovery may be blocked. Static peers work around it:

```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="<other machine ip>"
```

`ROS_LOCALHOST_ONLY` is deprecated and overrides `ROS_AUTOMATIC_DISCOVERY_RANGE` when
set, which is confusing if it is left over in a shell.

## Real-time

The test machine runs `6.14.0-37-generic` (`PREEMPT_DYNAMIC`), not `PREEMPT_RT`.
`controller_manager` reports overruns even with only the broadcasters loaded:

```
[WARN] Overrun might occur, Total time : 1084 us (Expected < 1000 us)
       --> Read time : 1023 us, Update time : 53 us, Write time : 6 us
[WARN] Overrun detected! ... missed cycles : 2
```

One `libfranka: UDP receive: Timeout` was observed, which deactivates the hardware
component and with it every controller. `libfranka` documents a `PREEMPT_RT` kernel plus
`rtprio 99` and `memlock unlimited` for the control user. This build has the limits set
but not the kernel.
