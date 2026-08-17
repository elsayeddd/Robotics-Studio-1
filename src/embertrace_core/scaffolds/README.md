# Outstanding C++ scaffolds

These files are deliberately excluded from the build so the baseline cannot claim autonomous Nav2 execution before it is tested. Each file compiles against the current project interfaces once added to `CMakeLists.txt`; the Nav2 adapter then needs the additional action/TF dependencies described in its header.

Recommended order:

1. Complete and enable `nav2_goal_adapter_node.cpp`.
2. Verify cancellation and emergency-stop behaviour.
3. Complete and enable `patrol_manager_node.cpp`.
4. Add action-result feedback before attempting the full patrol–investigate–return sequence.

