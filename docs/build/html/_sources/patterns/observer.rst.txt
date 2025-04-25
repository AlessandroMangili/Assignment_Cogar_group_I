Observer
----------

Since the observer pattern defines a subject that maintains a list of observers and notifies them of state changes and is typically used when components need to react to events asynchronously, we can identify which components in our system are suited to this pattern.
The error handler component can also be implemented using the observer pattern, since it needs to keep all components informed in case of an error. Internally, it will employ the strategy pattern to select the appropriate recovery policy, and based on that strategy it must notify all observers subscribed to the error handler what actions to take following the error, whether to reboot the component or continue the execution.