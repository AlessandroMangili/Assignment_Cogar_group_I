#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Int32, String
from assignments.msg import ErrorMessage

class ErrorHandlerComponent:
    def __init__(self):
        rospy.init_node('error_handler')
        
        self.timeout = 50.0
        self.last_activity = time.time()
        self.rate = rospy.Rate(20)
        
        self.error_code_sub = rospy.Subscriber('/error_code', Int32, self.error_code_callback)
        self.error_msg_pub = rospy.Publisher('/error_message', ErrorMessage, queue_size=10)
        
        rospy.loginfo("Error Handler Component Ready")
    
    def error_code_callback(self, msg):
        self.last_activity = time.time()
        
        error_code = msg.data
        if self.check_errors(error_code):
            error_msg = self.get_error_message(error_code)
            self.error_msg_pub.publish(error_msg)
            rospy.loginfo(f"Error published: {error_msg}")
    
    def check_errors(self, error_code):
        if time.time() - self.last_activity > self.timeout:
            rospy.logwarn("Watchdog timeout!")
            self.error_msg_pub.publish("Errore: Watchdog timeout")
            return False
        
        if not isinstance(error_code, int):
            rospy.logwarn("Error code not valid!")
            return False
        return True
    
    def get_error_message(self, error_code):
        error_messages = {
            1: ("ERROR HANDLER: Communication error", 1),
            2: ("ERROR HANDLER: Recipe failed!", 2),
            3: ("ERROR HANDLER: Failed to set the wheels speed", 3),
            4: ("ERROR HANDLER: Speaker error", 4),
            5: ("ERROR HANDLER: Navigation error", 5),
            6: ("ERROR HANDLER: Gripper error", 6)
        }
        
        message_tuple = error_messages.get(error_code, (f"Unknown error: {error_code}", 0))
        
        error_msg = ErrorMessage()
        error_msg.message = message_tuple[0]
        error_msg.id_component = message_tuple[1]
        
        return error_msg
    
    def run(self):
        while not rospy.is_shutdown():
            if time.time() - self.last_activity > self.timeout:
                self.error_msg_pub.publish("Error: Watchdog timeout")
                rospy.logwarn("Watchdog timeout!")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ErrorHandlerComponent()
        node.run()
    except rospy.ROSInterruptException:
        pass