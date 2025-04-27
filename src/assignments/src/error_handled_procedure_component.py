#!/usr/bin/env python3
import rospy
import time
import random
from std_msgs.msg import Bool, Float32

class ErrorHandledProcedureComponent:
    def __init__(self):
        rospy.init_node('error_handled_procedure_component')
        
        # Configuration
        self.timeout = 10.0
        self.rate = rospy.Rate(10)  # 10Hz
        
        # State
        self.active = False
        self.error = False
        self.last_activity = time.time()
        
        # ROS interfaces
        self.cmd_sub = rospy.Subscriber('/procedure_cmd', Bool, self.cmd_callback)
        self.duration_sub = rospy.Subscriber('/procedure_duration', Float32, self.duration_callback)
        self.status_pub = rospy.Publisher('/procedure_status', Bool, queue_size=10)
        
        rospy.loginfo("Error Handled Procedure Component Ready")

    def cmd_callback(self, msg):
        
        if msg.data and not self.active:
            self.start_procedure()
        elif not msg.data and self.active:
            self.stop_procedure("Stopped by command")

    def duration_callback(self, msg):
        
        self.duration = msg.data

    def start_procedure(self):
        
        self.active = True
        self.error = False
        self.start_time = time.time()
        self.last_activity = time.time()
        rospy.loginfo(f"Starting procedure for {self.duration} sec")

    def stop_procedure(self, reason):
        
        self.active = False
        self.status_pub.publish(self.error)
        rospy.loginfo(f"Procedure stopped: {reason}")

    def check_errors(self):
        """Check for various error conditions"""
        
        if time.time() - self.last_activity > self.timeout:
            rospy.logwarn("Watchdog timeout!")
            return True
            
        
        if int(time.time()) % 5 == 0 and random.random() < 0.2:
            rospy.logwarn("Random error occurred!")
            return True
            
        return False

    def run(self):
        """Main execution loop"""
        while not rospy.is_shutdown():
            if self.active:
                self.last_activity = time.time()
                
                
                if time.time() - self.start_time >= self.duration:
                    self.stop_procedure("Completed successfully")
                    continue
                    
                
                if self.check_errors():
                    self.error = True
                    self.stop_procedure("Error detected")
                    continue
                    
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ErrorHandledProcedureComponent()
        node.run()
    except rospy.ROSInterruptException:
        pass