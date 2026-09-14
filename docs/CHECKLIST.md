# Before publishing

Placeholders and unverified items, so nothing goes out wrong.

## Must fill in

- [ ] `README.md` and `README.ko.md` — demo gif at the top
- [ ] Keep both READMEs in sync when editing
- [ ] `docs/data-collection.md` — dataset path, format, episode lengths, camera specs
- [ ] `docs/data-collection.md` — fine-tuning config: base checkpoint, steps, batch size,
      embodiment tag. Ask whoever ran the training.
- [x] `docs/data-collection.md` — Isaac Sim repo linked
- [ ] `LICENSE` — replace "GELLO-Feetech-FR3 contributors" with a name

## Must verify

- [ ] Print settings in `docs/hardware.md` are what was actually used
- [ ] J2 moment estimate (1.3 N·m) — stated as an estimate, fine, but check the mass
- [ ] Gripper stroke ~95° against Franka GELLO's ~70° — both from measurement?
- [x] `config/uarm_fr3.yaml.example` — serial stripped by assemble.sh

## Media to record

One session covers all of it:

- [ ] Leader arm and robot in the same frame, teleoperated
- [ ] RViz model tracking the leader, side by side with the real arm
- [ ] Gripper open/close close-up
- [ ] Policy rollout — approach phase
- [ ] A failure: limit clipping, or the arm sagging when released
- [ ] Still photo of the leader arm alone, and of the FR3
- [ ] `ros2 bag record -o teleop_$(date +%Y%m%d_%H%M) /gello/joint_states /franka/joint_states /joint_states`

The bag is what turns "built it" into "measured it". Per-joint RMS error and latency
both come out of one recording.

## Accuracy

- [ ] Every "we did X" in the README is something that actually happened
- [ ] Attribution of the policy training is unambiguous
- [ ] No success rate is claimed anywhere
- [ ] `NOTICE` commit hashes match the trees actually used
