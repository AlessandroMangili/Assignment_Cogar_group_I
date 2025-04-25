#!/usr/bin/env python3
import rospy
from assignments.srv import Navigation, NavigationResponse
import random

map_upper_bound = 100.0
map_lower_bound = -100.0

def mote_to_point(req):
    """
    Service callback that move the robot to the target goal provided by the request.
    """
    rospy.loginfo("Received target goal state for comparison.")
    num = random.randint(1, 100)
    
    if(num < 20):
        rospy.loginfo("Target unreachable")
        rospy.sleep(5)
        return NavigationResponse(False)

    # Check if the dimensions of positions, velocities, and efforts match the expected joint count
    if (req.x >= -map_lower_bound or req.x <= map_upper_bound)  and \
       (req.y >= -map_lower_bound or req.y <= map_upper_bound) and \
       (req.z >= -map_lower_bound or req.z <= map_upper_bound):
        rospy.loginfo("Target goal valid.")
        rospy.sleep(5)
        rospy.loginfo("Target goal setted, the robot should call the wheels_speed service to move!")
        return  NavigationResponse(True)
    else:
        rospy.loginfo("Target goal not valid.")
        return NavigationResponse(False)

def navigation_service():
    """
    Initializes the ROS service server that move the robot to the target goal.
    """
    rospy.init_node('navigation_service', anonymous=True)
    # Create the service that listens for requests
    service = rospy.Service('/move_to_point', Navigation, mote_to_point)
    rospy.loginfo("Navigation service is ready!")
    rospy.spin()

if __name__ == '__main__':
    try:
        navigation_service()
    except rospy.ROSInterruptException:
        pass