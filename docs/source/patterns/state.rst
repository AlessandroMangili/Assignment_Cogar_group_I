State
---------

The State pattern is a behavioral design pattern that allows an object to change its behavior based on its internal state. It appears as if the object has changed its class.
To effectively implement the robot state component, we can apply the state pattern. This is because the robot can be in various states, such as `NoRecipe`, `Cooking`, `Paused`, or `Error`, each encapsulating its own transition logic. This approach helps avoid long chains of `if/else` statements. Moreover, since the robot’s behavior must change dynamically at runtime based on its current state, this pattern is particularly well suited for this purpose.