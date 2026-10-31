# Source provenance and preparation changes

Edition: `formation-v0.1.0`. These are standalone, documented snapshots prepared on
31 October 2026. They are not reconstructed original Git commit histories.

The formation controller and all eight packages came from the current simulation
workspace. The position/height bank and local formation tracking remain active.

`validation/source_manifest.json` records hashes before preparation edits and,
for the velocity edition, the archive hash. The published-source manifest records
the prepared files. Original workspaces were not edited.

Preparation changes:

- Add portable setup/build/verify/test/run scripts, documentation, CI and evidence.
- Replace placeholder package-maintainer email addresses with the GitHub noreply identity.
- Preserve old package delivery notes under docs/history and add current package pointers.
- Replace fixed home-directory defaults in supported coordinator/replay commands
  with checkout-relative results/reference defaults.
- Use the current scene/controller geometry guard in both editions.
- Retire stale reference-specific `check_local_controller.py`,
  `check_closed_loop.py`, `check_ground_mission.py` and the removed-function
  `check_formation_control.py` entry points when present. The numerical full
  runner and supported protocol checks replace them.
- Strengthen the recovered velocity controller to reject nonfinite acceleration,
  nonfinite slack and reported hard-row violations before motor allocation.
  The nominal controller, observer, battery and QP equations are unchanged.

The earlier single-process baseline remains available as a comparator. Old
package delivery notes may reference their historical installation paths; the
root README and docs/ guides define the supported release procedure.

License notices and Bitcraze mesh attribution are preserved. No GitHub credentials,
local IDE databases, build/install trees, virtual environments or full old
experiment datasets are included in the source commits.
