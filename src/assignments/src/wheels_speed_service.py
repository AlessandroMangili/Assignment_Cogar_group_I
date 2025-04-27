#!/usr/bin/env python3
import rospy
from assignments.srv import WheelsSpeed, WheelsSpeedResponse
import random
from std_msgs.msg import String, Int32
from assignments.msg import ErrorMessage

max_wheel_speed = 10.0
min_wheel_speed = -10.0

def set_wheels_speed(req):
    rospy.loginfo("Received speeds for the wheels.")
    num = random.randint(1, 100)
    
    if(num < 20):
        rospy.loginfo("Speeds not settable")
        rospy.sleep(5)
        return WheelsSpeedResponse(False)

    if (req.left_speed >= -min_wheel_speed or req.left_speed  <= max_wheel_speed)  and \
       (req.right_speed >= -min_wheel_speed or req.right_speed  <= max_wheel_speed):
        rospy.loginfo("Speeds have been setted to the wheels.")
        rospy.sleep(1)
        rospy.loginfo("The robot moved!")
        return WheelsSpeedResponse(True)
    else:
        rospy.loginfo("Speeds are out of range.")
        return WheelsSpeedResponse(False)

def wheels_speed_service():
    rospy.init_node('wheels_speed_service', anonymous=True)
    service = rospy.Service('/set_wheels_speed', WheelsSpeed, set_wheels_speed)
    rospy.Subscriber('/error_message', ErrorMessage, error_callback)
    error_pub = rospy.Publisher('/error_code', Int32,  queue_size=10)
    rospy.loginfo("Wheels speed service is ready!")
    rospy.spin()

def error_callback(msg):
        error = msg
        if error.id_component == 6:
            rospy.logerr(f"Received an error from the error handler: {error}")

if __name__ == '__main__':
    try:
        wheels_speed_service()
    except rospy.ROSInterruptException:
        pass