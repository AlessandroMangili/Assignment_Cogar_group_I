Strategy
-----------

The strategy pattern is a behavioral design pattern that enables the selection of an algorithm at runtime. Instead of implementing a single algorithm directly, the system receives instructions on which strategy to apply from a defined family of algorithms.
In our case, the entire flow of the third block, starting from the evaluate component, can be represented using the strategy. This is because the behavior changes dynamically based on the user's request. For example:
-	if the user is asking for a new recipe, and the system needs to search online.
-	if the user wants new suggestions based on the ingredients they have at home, and the system needs to search online.
-	if the user gives a voice command and the robot needs to execute it.
Each of these actions represents a different strategy that can be selected and executed at runtime.
Additionally, the update best action component can also be implemented using this pattern. Based on the input it receives, it determinates the appropriate cooking strategy.
Even the error handler can be seen as an application of this pattern. It decides which recovery procedure to apply whether to reboot a single component or restart the entire system depending on the type and severity of the error.