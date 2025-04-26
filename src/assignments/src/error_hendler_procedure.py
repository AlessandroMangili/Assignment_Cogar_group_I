#!/usr/bin/env python3
import rospy
import time
import random
from std_msgs.msg import Bool
from assignments.srv import ErrorHandledProcedure, ErrorHandledProcedureResponse

class ErrorHandler:
    def __init__(self):
        self.error_occurred = False
        self.error_pub = rospy.Publisher('/system_error', Bool, queue_size=10)
        
    def check_error(self, condition, error_msg):
        if condition:
            rospy.logerr(f"Error detected: {error_msg}")
            self.error_occurred = True
            self.error_pub.publish(True)
            return True
        return False
    
    def reset_error(self):
        self.error_occurred = False
        self.error_pub.publish(False)

class Watchdog:
    def __init__(self, timeout=5.0):
        self.timeout = timeout
        self.last_activity = time.time()
        
    def activity(self):
        self.last_activity = time.time()
        
    def check_timeout(self):
        if (time.time() - self.last_activity) > self.timeout:
            rospy.logwarn("Watchdog timeout triggered!")
            return True
        return False

class ErrorHandledProcedureComponent:
    def __init__(self):
        rospy.init_node('error_handled_procedure_component')
        
        # Initialize subsystems
        self.error_handler = ErrorHandler()
        self.watchdog = Watchdog(timeout=10.0)
        
        # Create service
        self.service = rospy.Service('/execute_error_handled_procedure', 
                                   ErrorHandledProcedure, 
                                   self.handle_procedure_request)
        
        rospy.loginfo("Error Handled Procedure Component initialized")
        
    def handle_procedure_request(self, req):
        # Reset systems for new request
        self.error_handler.reset_error()
        self.watchdog.activity()
        
        # Error check - validate input parameters
        if self.error_handler.check_error(not req.enable, "Procedure requested with enable=False"):
            return ErrorHandledProcedureResponse(False, "Error: Procedure not enabled")
            
        if self.error_handler.check_error(req.duration <= 0, "Invalid duration specified"):
            return ErrorHandledProcedureResponse(False, "Error: Duration must be positive")
        
        # Execute main procedure
        try:
            result = self.execute_procedure(req.duration)
            if result:
                return ErrorHandledProcedureResponse(True, "Procedure completed successfully")
            else:
                return ErrorHandledProcedureResponse(False, "Procedure completed with warnings")
        except Exception as e:
            self.error_handler.check_error(True, f"Procedure execution failed: {str(e)}")
            return ErrorHandledProcedureResponse(False, f"Error: {str(e)}")
    
    def execute_procedure(self, duration):
        """Main procedure logic with error handling"""
        rospy.loginfo(f"Starting error-handled procedure for {duration} seconds")
        
        start_time = time.time()
        while (time.time() - start_time) < duration and not rospy.is_shutdown():
            # Simulate work
            time.sleep(0.1)
            self.watchdog.activity()  # Update watchdog
            
            # Simulate random errors (20% chance)
            if int(time.time()) % 5 == 0 and not self.error_handler.error_occurred:
                if self.error_handler.check_error(random.random() < 0.2, "Random error during procedure"):
                    return False
                    
            # Check for watchdog timeout
            if self.watchdog.check_timeout():
                self.error_handler.check_error(True, "Watchdog timeout during procedure")
                return False
                
        rospy.loginfo("Procedure completed")
        return True

if __name__ == '__main__':
    try:
        component = ErrorHandledProcedureComponent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass