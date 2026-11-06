# Changelog

## single-position-v0.1.0 — 6 November 2026

- Add separate `safe_redcho_single_sim`: one position REDCHO bank per agent,
  derivative-channel velocity/acceleration, existing HOCBF-QP and native motors.
- Retain the six-agent ground-start, formation/height, battery fault and two
  obstacles with unchanged controller equations, gains and world geometry.
- Include complete portable commands, formulation, telemetry, figures, scoped
  Gazebo cleanup, tests and numerical/native validation evidence.
- Preserve the existing velocity/formation packages and prior release tags.

# Changelog

## formation-v0.1.0

Initial standalone release of the formation implementation. Adds portable setup, verification, numerical running, tests, CI and documentation. Preserves original reference/observer/QP equations; see docs/provenance.md for preparation changes.
