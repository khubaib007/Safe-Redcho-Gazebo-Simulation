# Verification and expected results

## Local software

```bash
bash scripts/verify.sh
bash scripts/test.sh quick
```

Expected: `PASS_LOCAL_SOFTWARE`, eight package manifests, valid scene/controller
geometry, and all quick checks PASS. This does not launch ROS or certify physics.

## Full numerical run

```bash
bash scripts/test.sh full
```

The report appears under `results/tests_<UTC>/test_report.json`. The numerical
subdirectory contains a full 45-second run and plots. Every evaluated check must
be true. The release-preparation numerical run observed minimum pair spacing
approximately 0.912904 m. Exact floating-point values can vary with numerical
dependencies; the actual hard-constraint checks remain authoritative.

## Built interfaces

```bash
bash scripts/verify.sh --ros
```

Expected: generated `StepSwarm`, control-frame/result and safety-bound imports,
and native libraries resolving inside this checkout. A successful build/import
does not prove that DDS discovery or native stepping works.

## Native mission

Use the README sequence. Expected after a full mission:

- `status` is `PASS`.
- `completed_cycles` equals `expected_cycles` (4,501 for 45 seconds).
- Six distinct agent process IDs and 4,501 results per agent.
- `accepted_motor_batches` equals 4,501.
- All reported checks are true.

Inspect `scope`, metadata and backend fields before comparing evidence. A short
smoke run omits the fault/obstacle/recovery portions and is not full validation.
