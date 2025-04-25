Mediator
-----------

In our architecture, the low-level planner component plays a key role by centralizing the management of hardware components responsible for movement, navigation, and the gripper. Due to this central coordination role and the fact that it dynamically changes the program’s behavior at runtime, we can implement it using the mediator pattern.
