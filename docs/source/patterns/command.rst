Command
----------

For this pattern, an object contains all the necessary information to execute an action or trigger an event. A good example of this is the search components, which can be effectively represented using this pattern, as they encapsulate everything needed to carry out a specific task.
In our case, the Interpreter component analyzes the verbal interaction and distinguishes whether it is a command, triggering different actions depending on what is requested or a voice input asking for new recipes or suggestions based on the available ingredients. For example, there are two types of search functionalities: both perform online searches, but one retrieves recipe suggestions based on a list of available ingredients, while the other searches for a specific recipe by name, as provided by the user through the microphone.
