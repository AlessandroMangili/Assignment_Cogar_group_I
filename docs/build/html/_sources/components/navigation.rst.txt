Navigation
--------------

.. image:: ../_static/navigation.png
   :alt: Navigation component
   :align: center
   :width: 100%
   :height: 576px
   :target: #

| The **navigation** component is used to create and manage the environment map through the connected sensors, saving it via the artifact linked to the **SLAM** component.
| Additionally, it is responsible for generating the path that the robot must follow based on actions received from the :doc:`Planner<planner>` component. To dynamically update the map and generate the trajectory planning, this component requires data from other modules, such as :doc:`Perception<perception>` for obstacle detection and tracking, :doc:`Motion<motion>` for wheels encoders, and :doc:`Gripper<gripper>` for gripper encoder.
| With the integration of **LIDAR** and **SONAR** sensors, the component can build and continuously update a 360-degree map of the environment.
| As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

To connect the **SLAM** component to the two internal sensors, LIDAR and SONAR, it is necessary to implement them using the **adapter** pattern, in order to make the exchanged data compatible and allow SLAM to manage them properly.
