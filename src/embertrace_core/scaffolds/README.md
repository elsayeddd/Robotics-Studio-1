# Completed scaffold phase

The Nav2 adapter and patrol manager are built from `../src/`. The adapter publishes
action status; the patrol manager consumes it, retries failures, checks measured
arrival and announces return completion. There are no excluded node scaffolds.
See `docs/integration-review.md` and `docs/testing.md` from the repository root.
