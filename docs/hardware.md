# Hardware

## Bill of materials

| Item | Qty | Note |
|---|---|---|
| Feetech STS3215, C044 (1:191) | 8 | 7 joints + gripper trigger |
| U-Arm Config3 printed frame | 1 set | base, link1–link7, sitter ×2, sitter_mid, trigger_short |
| Waveshare Serial Bus Servo Driver Board | 1 | USB ↔ TTL half-duplex |
| 5 V 4 A supply | 1 | |
| 608ZZ bearing | 1 | J1, so the arm does not hang off the servo output shaft |

Print settings used: FDM, wall ≥ 3 mm, infill 40 %.

## Gear ratio

The original U-Arm build guide assigns different ratios per joint: 1:345 (C001) on J2,
1:191 (C044) on J4, 1:147 (C046) elsewhere. This build uses **C044 on all eight joints**,
which is what was available.

That has two effects, in opposite directions:

- **J2 gets weaker.** 1:191 backdrives more easily than 1:345, so the shoulder sags more
  than the original design intended. This is the source of the gravity-droop limitation.
- **Everything else gets stiffer.** The leader arm uses its servos as encoders, not
  actuators, so lower gear ratios mean less backdrive friction and a lighter feel.
  1:191 is heavier to move than 1:147.

Estimated J2 moment is around 1.3 N·m (0.65 kg at 0.2 m). Franka's own GELLO uses a
McMaster-Carr 9271K53 torsion spring (~0.25 N·m/rad) for this. Config3 has no spring
boss, so that requires editing the STEP file or an external band.

## STS3215

- 45.2 × 24.7 mm, ~60 g
- 12-bit magnetic encoder, 4096 steps / 360°, 0.088° per step
- Operating modes: 0 = position, 1 = closed-loop velocity, 2 = open-loop velocity,
  3 = step. **No current or torque mode.**
- Bus: TTL half-duplex daisy chain, 1 Mbps, IDs 1–253

The missing current mode is the single fact that shapes this whole project. GELLO uses
current-based position control for its virtual gravity spring; that is not available
here.

## Servo IDs

IDs run 1–8 from base to gripper:

| ID | Joint | Type |
|---|---|---|
| 1 | J1 | yaw |
| 2 | J2 | pitch — carries the whole arm |
| 3 | J3 | roll |
| 4 | J4 | pitch (elbow) |
| 5 | J5 | roll |
| 6 | J6 | pitch (wrist) |
| 7 | J7 | roll |
| 8 | — | gripper trigger |

`tools/scan_ids.py` lists whatever is on the bus. `tools/remap_chain.py` renumbers a
whole chain in one pass and prints the reverse mapping, which matters if the servos came
from another robot and you want to be able to put them back.

`tools/changeid.py` writes one servo at a time using `BROADCAST_ID` — the method the
U-Arm scripts use. It refuses to run unless exactly one servo answers, because a
broadcast write with several servos connected sets them all to the same ID.

## Kinematics

Config3 is not a scaled replica of the FR3. One frame covers FR3, Panda, Rizon, RM75B
and xArm7, which means the joint topology matches (yaw-pitch-roll-pitch-roll-pitch-roll)
but link lengths and offsets do not.

Consequences:

- Joint angles map correctly, so joint-space teleoperation works.
- The leader does not visually resemble the follower, so it is less intuitive than a
  true GELLO replica.
- `joint_signs` cannot be copied from GELLO's Franka config. It has to be measured.

FR3 joint limits, from `franka_description` 2.8.1 `robots/fr3/joint_limits.yaml`:

```
J1  [-2.9007, +2.9007]      J5  [-2.8763, +2.8763]
J2  [-1.8361, +1.8361]      J6  [+0.4398, +4.6216]
J3  [-2.9007, +2.9007]      J7  [-3.0508, +3.0508]
J4  [-3.0770, -0.1169]
```

J4 is always negative and J6 always positive. That asymmetry is what fixes the physical
positive direction for those two joints without guessing.

## Serial port

The Waveshare board enumerates as a CH340 (`1a86:55d3`). Depending on kernel version it
binds to `ch341` (`/dev/ttyUSB0`) or `cdc_acm` (`/dev/ttyACM0`). A udev rule pins it:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", SYMLINK+="uarm"
```

This also avoids needing a re-login for `dialout` group membership.

## SDK

Use the `scservo_sdk` bundled with U-Arm, not the PyPI `feetech-servo-sdk` package.
The PyPI package ships only a generic `PacketHandler` — it has no `sms_sts` class and
none of the `SMS_STS_*` control table constants, so the STS3215 register addresses are
unavailable.
