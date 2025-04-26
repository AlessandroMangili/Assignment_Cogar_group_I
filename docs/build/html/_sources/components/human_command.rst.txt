Human command monitoring and conflict resolution
--------------------------------------------------

.. image:: ../_static/human_command.png
   :alt: Human command component
   :align: center
   :width: 100%
   :height: 800px
   :target: #

Da aggiungere qui .....

As we can observe, there are two ports that are not connected to any internal component, as these serve as the input and output for the :doc:`error handler <error_handler>` component, which will manage any recovery procedures.

.. rubric:: Implementation through patterns

First, the **Wi-Fi connection** manager component should be implemented as a **singleton** pattern, as there must be only one object responsible for handling internet requests, such as searching for recipes or proposals. This design choice is justified by the fact that there is only one physical Wi-Fi device available, and creating multiple instances could lead to consistency issues.

Next, to connect the **microphone** to the **interpreter**, we need an interface that handles the microphone's output data and adapts it so that the interpreter component can correctly process it in the input format it expects. To achieve this, we need to apply the **adapter** pattern.

As we mentioned in the previous component, the **strategy** pattern is a behavioral design pattern that enables the selection of an algorithm at runtime. In our case, the entire flow starting from the **evaluate** component can be represented using the Strategy pattern, since the behavior changes dynamically based on the user's request. For example:

-  If the user asks for a new recipe, the system needs to search online.
-  If the user requests new suggestions based on the ingredients they have at home, the system must search accordingly.
-  If the user issues a voice command, the robot needs to execute it.

Each of these actions represents a different strategy that can be selected and executed at runtime.