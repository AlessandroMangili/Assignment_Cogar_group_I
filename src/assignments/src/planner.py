#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from assignments.srv import Navigation, NavigationRequest
from assignments.msg import Action, RobotState

class HighLevelAction:
    def __init__(self):
        rospy.loginfo("Initializing High Level Action")
    
        self.action_pub = rospy.Publisher('/high_level_action', String, queue_size=10)
        
        rospy.Subscriber('/notify_action', Action, self.notify_action_callback)
        rospy.Subscriber('/battery_level', Float64, self.battery_callback)
        rospy.Subscriber('/robot_state', RobotState, self.robot_state_callback)
        
        self.current_best_action = Action()
        self.battery_level = 100.0
        self.current_state = ""
    
    def notify_action_callback(self, data):
        self.current_best_action = data.label

        if self.current_best_action == "":
            return
        
        if self.battery_level < 50.0:
            action = "RETURN_TO_CHARGING_STATION"
            rospy.loginfo("Battery low, sending robot to charging station")
        else:
            action = "Wheels" if random.random() < 0.5 else "Gripper"
            rospy.loginfo(f"Using best action: {action}")
        
        self.action_pub.publish(action)
        rospy.loginfo(f"Published high-level action: {action}")
    
    def battery_callback(self, data):
        self.battery_level = data.data
        rospy.loginfo(f"High Level Action received battery level: {self.battery_level:.1f}%")
    
    def robot_state_callback(self, data):
        self.current_state = data

class Robot_state:
    def __init__(self):
        rospy.loginfo("Initializing Robot State")
    
        self.state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)
        
        rospy.Subscriber('/high_level_action', String, self.action_callback)
        
        self.current_state = RobotState(state="No Recipe")
        self.last_action = ""
        
        rospy.Timer(rospy.Duration(1.0), self.publish_state)
    
    def action_callback(self, data):
        self.last_action = data.data
        if self.last_action.startswith("Grabbing"):
            self.current_state = "Grabbing"
        elif "Cutting" in self.last_action:
            self.current_state = "Cutting"

        rospy.loginfo(f"Robot state updated to {self.current_state} based on action: {self.last_action}")
    
    def publish_state(self, event=None):
        self.state_pub.publish(self.current_state)

class BatteryLevel:
    def __init__(self):
        rospy.loginfo("Initializing Battery Level component")
        
        self.battery_pub = rospy.Publisher('/battery_level', Float64, queue_size=10)
        
        rospy.Subscriber('/robot_state', RobotState, self.state_callback)
        
        self.current_level = 100.0
        self.discharge_rate = 0.1  # % per second
        
        rospy.Timer(rospy.Duration(10), self.update_battery)
    
    def state_callback(self, data):
        state = data
        if state == "Grubbing":
            self.discharge_rate = 0.3
        elif state == "Cutting":
            self.discharge_rate = 0.15
        elif state == "Charging":
            self.discharge_rate = -0.5
        else:
            self.discharge_rate = 0.1
    
    def update_battery(self, event=None):
        self.current_level -= self.discharge_rate
    
        self.current_level = max(0.0, min(100.0, self.current_level))
        
        self.battery_pub.publish(self.current_level)
        
        if self.current_level < 10.0:
            rospy.logwarn(f"Battery critically low: {self.current_level:.1f}%")
        elif self.current_level < 20.0:
            rospy.loginfo(f"Battery low: {self.current_level:.1f}%")
        elif self.current_level > 95.0:
            rospy.loginfo(f"Battery fully charged: {self.current_level:.1f}%")


def main():
    rospy.init_node('planner_high_level', anonymous=True)
    rospy.loginfo("Starting Planner High Level subsystem")
    
    action = HighLevelAction()
    robot_state = Robot_state()
    battery_level = BatteryLevel()
    
    rospy.loginfo("Planner High Level system running")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
