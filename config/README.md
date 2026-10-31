# Scenario configuration

The authoritative settings are in
`src/safe_redcho_distributed/safe_redcho_distributed/reactive_scenario.py`.
World poses and obstacle geometry must match those settings.

After editing a scenario, run from the checkout root:

```bash
source scripts/environment.sh
/usr/bin/python3 src/configure_scenario.py
bash scripts/verify.sh
bash scripts/test.sh full
bash scripts/build.sh
```

Restart both the world and all six controller processes after changing the
scenario. Formation recovery checks apply only to the documented formation
scenario. Changing geometry may require a new, explicitly documented evaluation
window; never bypass hard QP feasibility to obtain a PASS.
