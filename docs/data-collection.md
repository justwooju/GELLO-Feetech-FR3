# Demonstration data and policy

## Scope

To be clear about who did what:

- Leader arm build, calibration, ROS 2 integration, FR3 deployment — this project.
- Teleoperation recording of the demonstration episodes — this project.
- Policy fine-tuning on that dataset — carried out by a colleague on separate hardware.
- Running the resulting checkpoint on the FR3 — this project.

## Task

Pick up a black bowl and place it on a plate. Adapted from LIBERO-Spatial task 0:

> pick up the black bowl between the plate and the ramekin and place it on the plate

Two cameras: wrist-mounted and agent view.

## Recording

100 episodes, teleoperated with the leader arm described in this repository.

<!--
TODO — fill in from the recording machine:
  - dataset path and format version
  - episode length distribution
  - camera models, resolution, frame rate
  - recording rate
  - which episodes were discarded, if any
-->

## Fine-tuning

GR00T N1.7. Training was run by a colleague on an RTX A6000.

<!--
TODO — confirm with whoever ran the training:
  - base checkpoint
  - number of steps, batch size, learning rate
  - embodiment tag / modality config
  - wall-clock time
  - whether any validation split was held out
-->

## Result

The fine-tuned checkpoint was run on the FR3. The arm approaches the bowl under policy
control.

**Grasp success rate was not measured.** No trial count, no success criterion, no
comparison against a baseline. The observation is qualitative: the approach phase looks
reasonable, the grasp outcome is unconfirmed.

Stating it this way rather than as a success is deliberate. 100 episodes is a small
dataset, and "it moved toward the object" is not evidence that the policy learned the
task.

## What a real evaluation would need

Planned, not done:

- A success criterion — bowl lifted above a height threshold and released within the
  plate boundary, checked from recorded video or object pose.
- Enough trials for the number to mean something. Prior work in this lab settled on 24
  episodes minimum before an A/B judgement was considered reliable.
- A baseline. The zero-shot LIBERO checkpoint on the same setup, or a scripted
  trajectory.
- Failure taxonomy: approach miss, grasp slip, wrong placement, timeout.
- Object position randomisation, so the result is not a single memorised trajectory.

## Teleoperation quality

Not measured either, and it bounds everything downstream. Worth recording next time:

- Per-joint RMS error between `/gello/joint_states` and `/franka/joint_states`
- Leader-to-follower latency
- Overrun frequency with and without the impedance controller loaded

All three come out of a single `ros2 bag record` during a normal session:

```bash
ros2 bag record -o teleop_$(date +%Y%m%d_%H%M) \
  /gello/joint_states /franka/joint_states /joint_states
```

## Related

[Libero-GR00T-in-IsaacSim](https://github.com/SungjinDavidLee/Libero-GR00T-in-IsaacSim)
— same author. Zero-shot transfer of the same GR00T policy family into Isaac Sim,
measured over 500 episodes, with the action-scale and observation-convention work that
preceded this repository.

That project's finding — that the gap between simulators is in the action-space
convention rather than the policy — is why the leader arm here maps in joint space and
why calibration is verified in RViz before touching the robot.
