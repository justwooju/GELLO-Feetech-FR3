# Calibration

Four things have to be established, in this order. Each depends on the one before it.

1. Servo IDs
2. Joint zeros (`homing_offset`, written to EEPROM)
3. Joint signs
4. Assembly offsets and gripper range

## Before assembly — IDs

Do this with the servos loose. After assembly they are buried inside the printed links
and the daisy chain is awkward to break.

```bash
python3 tools/scan_ids.py
```

If the servos came from another robot they will have arbitrary IDs. `remap_chain.py`
renumbers the whole chain at once — possible because each servo already has a distinct
ID, so each can be addressed individually:

```bash
python3 tools/remap_chain.py           # dry run, prints the plan
DRY=0 python3 tools/remap_chain.py     # apply
```

It prints a reverse table. Keep it: running it backwards restores the servos to their
previous robot.

If several servos share an ID (factory state), use `changeid.py` with one servo
connected at a time.

`jog.py` moves one servo at a time so you can label which physical unit got which ID.
**Only run it with the joints free** — after assembly the links can collide.

Mark each servo with its ID *after* burning it, not before.

## After assembly — zeros

`set_zero.py` writes `homing_offset` so the current physical pose reads 2047, the centre
of the 12-bit range. This gives roughly ±180° of travel before the encoder wraps.

**This is an EEPROM write. Do it once.**

Hold the arm in the reference pose first:

```
J1 = 0    J2 = 0    J3 = 0    J4 = -90°    J5 = 0    J6 = +90°    J7 = 0
```

Shoulder straight up, elbow square forward, wrist square down — an "L" from the side.
J4 cannot be 0; its limit is `[-3.0770, -0.1169]`, which is why the reference pose uses
−90° there rather than all zeros.

```bash
python3 tools/set_zero.py            # dry run
APPLY=1 python3 tools/set_zero.py    # write
```

Offsets are clamped to ±2047. A joint that would need more is reported; a residual of a
few hundred counts is harmless once multi-turn accumulation is enabled in the driver,
since the encoder no longer wraps in the published signal.

## Joint signs

For each joint: move it in the FR3 positive direction and record whether the encoder
count rises or falls.

```bash
J=1 python3 tools/verify_sign.py
```

The tool accumulates wrapped deltas and prints a running total, so it stays correct
across the 4095/0 boundary. Move the joint, then Ctrl-C.

The FR3 positive direction comes from `franka_description` `kinematics.yaml`. Every
joint is `axis="0 0 1"`; the direction is set by each joint frame's `roll`:

| Joint | roll | Positive direction at the home pose |
|---|---|---|
| J1 | 0 | base counter-clockwise seen from above |
| J2 | −π/2 | shoulder leaning forward |
| J3 | +π/2 | upper arm twist, right-hand rule with thumb toward the elbow |
| J4 | +π/2 | elbow straightening |
| J5 | −π/2 | forearm twist, thumb toward the wrist |
| J6 | +π/2 | gripper tip lifting |
| J7 | +π/2 | wrist twist, thumb toward the gripper |

The roll and yaw joints (J1, J3, J5, J7) are the ones where a verbal description is
ambiguous. "Counter-clockwise seen from above" means nothing once the arm is folded.
Use the right-hand rule with the thumb pointing away from the base.

Measured on this build, then inverted on the roll/yaw joints for mirror operation:

```
measured:  [-1, -1, -1, -1, -1, -1, -1]      all servos mounted the same way around
in use:    [ 1, -1,  1, -1,  1, -1,  1]      J1/J3/J5/J7 inverted
```

Both were confirmed against the RViz model before going to hardware. The mirrored set is
what makes the leader feel natural when the operator faces the robot; the rationale is in
the README.

## Assembly offsets and gripper range

```bash
python3 tools/measure_all.py
```

Gripper open and closed first, then the reference pose on a five-second countdown so you
can hold the arm steady.

The offset is

```
assembly_offset = (raw - (q_ref + pi) * sign) mod 2pi
```

The `+ pi` comes from `GelloHardware.normalize_joint_positions`, which wraps into
`[mid - pi, mid + pi)`. Leaving it out puts J4 outside its limit, where `np.clip` pins
it and the joint stops responding. GELLO's own Franka defaults
(`[0, 0, 3.142, 3.142, 3.142, 4.712, 0]`) are π and 3π/2 for the same reason.

`measure_all.py` feeds its result back through `normalize_joint_positions` and refuses
to save unless every joint reconstructs to `q_ref` and lands inside its limit. Values
that fail the check are not written.

Measured on this build:

```yaml
joint_signs:       [1, -1, 1, -1, 1, -1, 1]
assembly_offsets:  [0.0307, 5.6941, 6.0792, 4.1924, 0.1672, 0.2163, 0.6749]
gripper_range_rad: [3.3579, 2.5617]   # [closed, open]
```

Gripper stroke is about 46°. Full mechanical travel is roughly twice that; narrowing the
range means less trigger motion covers the whole 0–1 command, which is easier to operate.
`process_gripper_position` clips, so anything outside the range reads as fully open or
fully closed. `tools/grip_range2.py` measures a comfortable sub-range rather than the
full stroke, and `tools/watch_grip.py` shows the mapped percentage live.

`assembly_offsets[6]` (J7) is not the measured neutral. It is shifted by +45° to
compensate for the FR3 hand mounting — see the README. `tools/tune_j7.py` changes that
one joint without touching the others.

Order matters: `gripper_range_rad` is `[closed, open]`, not `[open, close]`.
`franka_gripper_client` computes `width = max_width * percent`, so percent 1.0 is fully
open, and `process_gripper_position` normalises against `range[1]` as the open end.

## Repeatability

Holding the reference pose by hand gives roughly 10–17° of spread on the pitch joints
between runs, because the arm sags when you let go. The roll joints are stable to a few
degrees.

This is a symptom of the missing gravity compensation, not of the method. Clamping the
arm or adding a J2 spring would tighten it.

It is not fatal: `assembly_offsets` record whatever pose you measured, and that pose
becomes the reference. The consequence of a sloppy measurement is that the robot's
starting pose is tilted by the same amount, not that the mapping breaks.

## Verifying before touching the robot

Publish the leader joint states and drive an RViz FR3 model with them. Wrong signs show
up as joints turning the wrong way; wrong offsets show up as a pose that does not match
the leader. Both are obvious on screen and harmless there.

Check the published values against the joint limits before activating the controller —
the robot moves to the leader's pose the moment the controller activates, and starting
against a limit is not a good place to be.
