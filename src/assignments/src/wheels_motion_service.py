#!/usr/bin/env python3
import rospy
from assignments.srv import WheelsSpeed, WheelsSpeedResponse
import random

def set_wheels_speed(req):
    """
    Service callback that set the speeds provided to the wheels.
    """
    rospy.loginfo("Received speeds for the wheels.")
    num = random.randint(1, 100)
    
    if(num < 20):
        rospy.loginfo("Speeds not settable")
        rospy.sleep(5)
        return WheelsSpeedResponse(False)

    # Check if the dimensions of positions, velocities, and efforts match the expected joint count
    if (req.left_speed >= -10 or req.left_speed  <= 10)  and \
       (req.right_speed >= -10 or req.right_speed  <= 10):
        rospy.loginfo("Speeds have been setted to the wheels.")
        rospy.sleep(5)
        rospy.loginfo("The robot moved!")
        return WheelsSpeedResponse(True)
    else:
        rospy.loginfo("Speeds are out of range.")
        return WheelsSpeedResponse(False)

def wheels_speed_service():
    """
    Initializes the ROS service server that set the wheels speed
    """
    rospy.init_node('wheels_speed_service', anonymous=True)
    # Create the service that listens for requests
    service = rospy.Service('/wheels_speed_service', WheelsSpeed, set_wheels_speed)
    rospy.loginfo("Wheels speed service is ready!")
    rospy.spin()

if __name__ == '__main__':
    try:
        wheels_speed_service()
    except rospy.ROSInterruptException:
        pass