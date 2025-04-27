#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point
from assignments.srv import CheckJointState, WheelsSpeed

class SLAM:
    def __init__(self):
        rospy.loginfo("Initializing Simple SLAM component")
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.position_pub = rospy.Publisher('/current_position', Point, queue_size=10)
        rospy.Timer(rospy.Duration(10), self.update_position)
    
    def update_position(self, event=None):
        delta_x = random.uniform(-0.5, 0.5)
        delta_y = random.uniform(-0.5, 0.5)
        
        self.position['x'] += delta_x
        self.position['y'] += delta_y
        
        position_msg = Point()
        position_msg.x = self.position['x']
        position_msg.y = self.position['y']
        position_msg.z = 0.0
        self.position_pub.publish(position_msg)
        
        rospy.loginfo(f"SLAM updated position: x={self.position['x']:.2f}, y={self.position['y']:.2f}")

class TrajectoryPlanning:
    def __init__(self):
        rospy.loginfo("Initializing Simple Trajectory Planning component")
        rospy.Subscriber('/current_position', Point, self.position_callback)
        self.high_level_action = rospy.Subscriber('/high_level_action', String, self.action_callback)
        
        self.wheels_client = rospy.ServiceProxy('/set_wheels_speed', WheelsSpeed)
        self.gripper_client = rospy.ServiceProxy('/check_joint_state', CheckJointState)
        self.status_pub = rospy.Publisher('/trajectory_status', String, queue_size=10)
        
        self.current_position = Point()
        self.target_position = Point()
    
    def position_callback(self, data):
        self.current_position = data
        
    def action_callback(self, data):
        action = data.data
        if action == "Wheels":
            self.target_position.x = random.uniform(0, 10)
            self.target_position.y = random.uniform(0, 10)
            self.move_to_target()
        else:
            self.move_gripper()
    
    def move_to_target(self):
        dx = self.target_position.x - self.current_position.x
        dy = self.target_position.y - self.current_position.y
        
        left_speed = 5.0 if dx > 0 else -5.0
        right_speed = 5.0 if dy > 0 else -5.0
        
        rospy.loginfo(f"Setting wheel speeds: left={left_speed}, right={right_speed}")
        
        # Call wheel service
        try:
            response = self.wheels_client(left_speed, right_speed)
            success = response.success
            if success:
                self.status_pub.publish("MOVING")
                rospy.loginfo("Successfully set wheel speeds")
            else:
                self.status_pub.publish("FAILED")
                rospy.logerr("Failed to set wheel speeds")
            return success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            self.status_pub.publish("ERROR")
            return False
    
    def move_gripper(self, open_gripper=True):
        positions = [0.0] * 7
        if not open_gripper:
            positions = [1.0] * 7 
        
        velocities = [0.5] * 7
        efforts = [0.5] * 7
        
        rospy.loginfo(f"Setting gripper to {'open' if open_gripper else 'closed'}")
        
        try:
            response = self.gripper_client(positions, velocities, efforts)
            success = response.success
            if success:
                self.status_pub.publish("GRIPPER_OK")
                rospy.loginfo("Successfully moved gripper")
            else:
                self.status_pub.publish("GRIPPER_FAILED")
                rospy.logerr("Failed to move gripper")
            return success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            self.status_pub.publish("GRIPPER_ERROR")
            return False


def main():
    rospy.init_node('simple_navigation_system', anonymous=True)
    rospy.loginfo("Starting Simple Navigation System")
    
    # Initialize components
    slam = SLAM()
    trajectory = TrajectoryPlanning() 
    
    rospy.loginfo("Simple Navigation System running")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
