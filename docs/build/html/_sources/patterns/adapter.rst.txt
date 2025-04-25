Adapter
---------

As the name suggests, the adapter pattern is used to connect interfaces that are otherwise incompatible, typically because they handle different types of data.

In our scenario, we use this pattern to connect various sensors to their respective components. For example:
-	connecting the SLAM module to LIDAR and SONAR sensors
-	linking wheel or gripper encoders to their corresponding controllers
-	connecting the microphone to the interpreter within the third fundamental component
-	integrating the camera in the perception component with the object recognition system
-	connecting the gripper’s force sensor to its controlling logic
