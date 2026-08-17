# Contributing

## Branches

Use one short-lived branch per task:

- `feature/<short-name>` for new capability;
- `fix/<short-name>` for a correction;
- `docs/<short-name>` for documentation only.

Do not commit directly to the shared main branch once the team begins parallel development.

## Before coding

1. Confirm the assigned node and its inputs/outputs in `docs/interface-contract.md`.
2. Confirm another teammate is not editing the same node.
3. Write the acceptance test or manual acceptance steps in the issue/branch description.

## Code expectations

- C++17 and ROS 2 Humble APIs;
- parameters instead of hard-coded thresholds or topic names;
- relative topic names inside robot namespaces, absolute `/embertrace/...` names for project-wide coordination;
- short callbacks and explicit state transitions;
- throttled logs in high-rate callbacks;
- comments that explain *why*, not every line of syntax;
- no node may read the configured fire-source coordinates except `environment_simulator_node`.

## Pull-request checklist

- [ ] `./scripts/validate_repo.py` passes.
- [ ] Changed ROS package builds with `colcon build --packages-select <package>`.
- [ ] New parameters are documented.
- [ ] Topic or service changes are reflected in the interface contract.
- [ ] A launch or unit test covers the change where practical.
- [ ] No generated `build`, `install`, `log` or bag files are committed.

