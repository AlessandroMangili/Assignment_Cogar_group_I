#!/usr/bin/env python3
import rospy
import time
import random
from std_msgs.msg import Bool, Float32, String
from assignments.msg import ProcedureCommand, ProcedureStatus

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
        self.procedure_active = False
        self.procedure_duration = 0.0
        
        # Create publisher for status updates
        self.status_pub = rospy.Publisher('/procedure_status', ProcedureStatus, queue_size=10)
        
        # Create subscriber for commands
        self.command_sub = rospy.Subscriber('/procedure_command', ProcedureCommand, self.command_callback)
        
        # Timer for procedure execution
        self.procedure_timer = None
        
        rospy.loginfo("Error Handled Procedure Component initialized (Pub/Sub version)")
        
    def command_callback(self, msg):
        """Handle incoming procedure commands"""
        if msg.enable and not self.procedure_active:
            # Start new procedure
            self.error_handler.reset_error()
            self.watchdog.activity()
            
            # Error check - validate input parameters
            if self.error_handler.check_error(msg.duration <= 0, "Invalid duration specified"):
                status = ProcedureStatus()
                status.success = False
                status.message = "Error: Duration must be positive"
                self.status_pub.publish(status)
                return
            
            self.procedure_active = True
            self.procedure_duration = msg.duration
            
            # Start procedure execution
            self.execute_procedure()
            
        elif not msg.enable and self.procedure_active:
            # Cancel current procedure
            self.procedure_active = False
            if self.procedure_timer:
                self.procedure_timer.shutdown()
            
            status = ProcedureStatus()
            status.success = False
            status.message = "Procedure cancelled by command"
            self.status_pub.publish(status)
    
    def execute_procedure(self):
        """Main procedure execution with error handling"""
        rospy.loginfo(f"Starting error-handled procedure for {self.procedure_duration} seconds")
        
        start_time = time.time()
        self.procedure_timer = rospy.Timer(rospy.Duration(0.1), 
                                         lambda event: self.procedure_loop(start_time))
    
    def procedure_loop(self, start_time):
        """Periodic procedure execution check"""
        if not self.procedure_active:
            return
            
        self.watchdog.activity()
        
        # Check for completion
        if (time.time() - start_time) >= self.procedure_duration:
            self.procedure_active = False
            self.procedure_timer.shutdown()
            
            status = ProcedureStatus()
            status.success = True
            status.message = "Procedure completed successfully"
            self.status_pub.publish(status)
            rospy.loginfo("Procedure completed")
            return
        
        # Simulate random errors (20% chance)
        if int(time.time()) % 5 == 0 and not self.error_handler.error_occurred:
            if self.error_handler.check_error(random.random() < 0.2, "Random error during procedure"):
                self.procedure_active = False
                self.procedure_timer.shutdown()
                
                status = ProcedureStatus()
                status.success = False
                status.message = "Error: Random failure during procedure"
                self.status_pub.publish(status)
                return
                
        # Check for watchdog timeout
        if self.watchdog.check_timeout():
            self.error_handler.check_error(True, "Watchdog timeout during procedure")
            self.procedure_active = False
            self.procedure_timer.shutdown()
            
            status = ProcedureStatus()
            status.success = False
            status.message = "Error: Watchdog timeout"
            self.status_pub.publish(status)
            return

if __name__ == '__main__':
    try:
        component = ErrorHandledProcedureComponent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass