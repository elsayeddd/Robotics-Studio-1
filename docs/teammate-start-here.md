# Start here for teammates

## If you are new to ROS 2

Learn these four ideas before changing code:

1. A **node** is one program with a focused responsibility.
2. A **topic** is an ongoing stream of messages.
3. A **service** is a request followed by one response.
4. An **action** is a long-running goal with feedback and cancellation, which is why Nav2 uses one.

Run these commands while the demo is open:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /embertrace/environment
ros2 topic hz /husky1/scan
ros2 service type /embertrace/operator_command
```

## How to read a node

For each `*_node.cpp`, read in this order:

1. Parameters declared in the constructor;
2. Publishers and subscribers;
3. Callback functions;
4. Timer callback, if present;
5. `TODO(student)` comments;
6. `main()` at the bottom.

Every node begins with a block comment describing what is complete, what is deliberately simple, and what the assigned developer should build next.

## Difficulty guide

| Task | Difficulty | Good first owner |
| --- | ---: | --- |
| Tune plume parameters and scenarios | Low | Less-confident member |
| Improve documentation and test logs | Low | Less-confident member |
| Add dashboard status fields | Low–medium | C++ beginner |
| Perception filter tests | Medium | C++ beginner/intermediate |
| Waypoint patrol | Medium | ROS learner |
| Nav2 action client | Medium–high | Confident ROS member |
| Source-localisation algorithm | High | Confident autonomy member |
| Camera ML model | High | Only if baseline is already stable |

## Team rule

The mathematical plume is part of the simulation, not an autonomy shortcut. Only the environment simulator may read `source_x` and `source_y`. Any algorithm that reads them directly invalidates the source-search demonstration.

