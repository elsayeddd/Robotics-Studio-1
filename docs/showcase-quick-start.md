# Showcase quick start — 5 November 2026

**Use the standalone drone for the autonomous demonstration.** The final larger-forest drone rehearsal passed. Full rover autonomous acceptance remains outstanding.

## Open the tested demonstration

From the repository root, after closing any existing EmberTrace launch:

```bash
bash scripts/run_showcase.sh drone
```

This opens the larger 70 × 70 m forest and **EmberTrace Ranger Console**, with the mission waiting in IDLE. Click **START** when ready to present. Keep the default scene and wind settings for the rehearsed run.

Expected sequence: takeoff → patrol → investigate/track → verify → confirmed alert/source marker → automatic return and landing → IDLE. The tested run took 107.54 simulation seconds, roughly 2½ minutes including startup on this machine. Runtime varies with host rendering load.

Show the map, sensor values and mission state while explaining how local evidence produces investigation goals. The same incident number is retained through investigation retries and confirmation. **Detection sample** means the source is unresolved; **Source estimate** means the location passed the acceptance gate.

After landing, RESET prepares another mission. ACK acknowledges an incident; it does not replace the vehicle's return sequence. Close the launch with Ctrl-C before starting a different platform.

## Results that can be stated

| Evidence | Recorded result |
| --- | --- |
| Final dense-forest drone fire rehearsal at `3b4f435` | Completed autonomously, one confirmed alert, source error **0.141 m**, returned and landed within **0.189 m** of launch in 3D, zero recovery cycles |
| Complete native confirmation at `3b4f435` | **69 tests**, zero errors/failures/skips across five packages; an earlier timing failure passed two isolated reruns and the complete confirmation |
| Earlier dense drone negative patrol | Two complete loops, all 12 sweep visits, no confirmed false alerts, automatic return/landing |
| Earlier standard-forest fire campaign at `0c932ba` | Ten missions, five per vehicle, all resolved and returned; **0.338 m** overall median source error |

The earlier campaign is pinned to an earlier revision. The final drone rehearsal is one mission, not a fresh ten-run statistical campaign. The drone is the supplied velocity-controlled simulation proxy, not validated physical aircraft dynamics.

## Rover status

The rover is integrated with perception, SLAM, Nav2, search, UI and return control, but recent complete missions failed on obstacles or unavailable thermal evidence. The last candidate adds a local clearance buffer, stronger preference for clear global paths and stable investigation IDs. A temporary smaller RGB/depth profile still hit thermal expiry and was reverted. Original sensor resolutions, rates and freshness checks are retained.

The rover command is available for subsystem inspection:

```bash
bash scripts/run_showcase.sh rover embertrace_forest.sdf
```

It is not a claimed successful autonomous showcase route. The dense world can be selected as the second argument, `embertrace_forest_dense.sdf`.

## Adam's presentation material

- [Full search/source fact sheet](search-source-presentation-factsheet.md)
- [Printable fact sheet](search-source-presentation-factsheet.pdf)
- [Final drone evidence](validation/2026-11-05/showcase-final-drone.json)
- [Native confirmation](validation/2026-11-05/showcase-native-final.json)

Opening explanation: “My subsystem uses local smoke, temperature, wind and measured positions to choose informative search goals and fit a possible source. It reports uncertainty; the mission manager independently decides whether the fire evidence and source location are sufficient. Nav2 or the drone controller executes the goals.”
