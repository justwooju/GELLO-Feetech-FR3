# Patches

Three changes to GELLO's `franka_fr3_arm_controllers` for ROS 2 Jazzy with
`ros2_control` 4.47.0. Against GELLO commit `fa0407bb`.

| # | File | Change |
|---|---|---|
| 01 | `src/joint_impedance_controller.cpp` | declare `k_alpha` in `on_init` |
| 02 | `config/controllers.yaml` | merge duplicate `/**:` blocks, add `arm_id` |
| 03 | build | `-DBUILD_TESTING=OFF` — no patch, see below |

Background for each is in [`../docs/ros2-jazzy-notes.md`](../docs/ros2-jazzy-notes.md).

## 01 — `k_alpha`

`on_configure` reads `k_alpha` but `on_init` never declares it. `ros2_control` 4.x no
longer auto-declares parameters from a params file, so configure fails with
`Original error: k_alpha`.

This is an upstream bug, not a Jazzy-specific workaround. Any `ros2_control` recent
enough to have dropped implicit declaration will hit it.

```bash
cd <gello_software>/ros2/src/franka_fr3_arm_controllers
patch -p1 < 01-k_alpha-declare.patch
```

## 02 — `controllers.yaml`

Two top-level `/**:` mappings. YAML keeps the last one, so the `controller_manager`
block holding the controller `type` entries is dropped and the launch file fails before
it reaches `k_alpha`.

Not shipped as a `.patch` because the fix is a whole-file rewrite;
`02-controllers-yaml-merge.md` has the replacement.

## 03 — tests

`test_joint_impedance_controller.cpp` calls `controller_->init("name")`.
`ControllerInterfaceBase::init` now takes five arguments or a
`ControllerInterfaceParams`. Fixing the test properly means rewriting the fixture; this
project builds with `-DBUILD_TESTING=OFF` instead.

The controller itself is unaffected — it only uses stable `controller_interface` API.

## Upstream

01 and 02 are candidates for a pull request to
[wuphilipp/gello_software](https://github.com/wuphilipp/gello_software).
