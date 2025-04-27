#!/usr/bin/env python3
import rospy
import unittest
import time
import rostest
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Point
from assignments.msg import Action, Recipe, RecipeHistory, OnExecutionActions, RobotState
from assignments.srv import Speaker, CheckJointState, WheelsSpeed

class RobotSystemIntegrationTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node('integration_test_node', anonymous=True)
        self.message_received = {}
        self.test_data = {}
        
        dummy_action = Action()
        dummy_action.label = "Grabbing"
        dummy_action.order = 1
        dummy_action.mandatory = True
        dummy_action.prerequisites = []
        dummy_action.tools = [1]
        dummy_action.ingredients = [1]
        self.test_data['action'] = dummy_action

        rospy.Subscriber('/recipe', Recipe, self.recipe_callback)
        rospy.Subscriber('/recipe_history', RecipeHistory, self.recipe_history_callback)
        rospy.Subscriber('/high_level_action', String, self.high_level_action_callback)
        rospy.Subscriber('/on_execution_actions', OnExecutionActions, self.on_execution_actions_callback)
        rospy.Subscriber('/battery_level', Float64, self.battery_level_callback)
        rospy.Subscriber('/robot_state', RobotState, self.robot_state_callback)
        rospy.Subscriber('/trajectory_status', String, self.trajectory_status_callback)
        rospy.Subscriber('/current_position', Point, self.position_callback)
        
        self.update_recipe_pub = rospy.Publisher('/update_recipe', String, queue_size=10)
        self.notify_action_pub = rospy.Publisher('/notify_action', Action, queue_size=10)
        self.update_on_execution_actions_pub = rospy.Publisher('/update_on_execution_actions', Action, queue_size=10)
        self.microphone_input_pub = rospy.Publisher('/microphone_input', String, queue_size=10)
        
        time.sleep(1)
        
        self.message_received = {
            'recipe': False,
            'recipe_history': False,
            'high_level_action': False,
            'on_execution_actions': False,
            'battery_level': False,
            'robot_state': False,
            'trajectory_status': False,
            'current_position': False
        }
        
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service('/speaker')
        rospy.wait_for_service('/check_joint_state')
        rospy.wait_for_service('/set_wheels_speed')
        rospy.loginfo("All services are available.")

    def recipe_callback(self, msg):
        self.message_received['recipe'] = True
        self.last_recipe = msg
        
    def recipe_history_callback(self, msg):
        self.message_received['recipe_history'] = True
        self.last_recipe_history = msg
        
    def high_level_action_callback(self, msg):
        self.message_received['high_level_action'] = True
        self.last_high_level_action = msg.data
        
    def on_execution_actions_callback(self, msg):
        self.message_received['on_execution_actions'] = True
        self.last_on_execution_actions = msg
        
    def battery_level_callback(self, msg):
        self.message_received['battery_level'] = True
        self.last_battery_level = msg.data
        
    def robot_state_callback(self, msg):
        self.message_received['robot_state'] = True
        self.last_robot_state = msg
        
    def trajectory_status_callback(self, msg):
        self.message_received['trajectory_status'] = True
        self.last_trajectory_status = msg.data
        
    def position_callback(self, msg):
        self.message_received['current_position'] = True
        self.last_position = msg

    def wait_for_messages(self, topics, timeout=10):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if all(self.message_received[topic] for topic in topics):
                return True
            time.sleep(0.1)
        return False

    def reset_message_flags(self):
        for key in self.message_received:
            self.message_received[key] = False

    def test_1_recipe_update_flow(self):
        self.reset_message_flags()
        
        recipe_name = "pasta carbonara"
        self.microphone_input_pub.publish(f"recipe {recipe_name}")
        
        topics_to_check = ['recipe', 'recipe_history']
        self.assertTrue(self.wait_for_messages(topics_to_check), 
                        "Did not receive recipe and recipe history messages")
        
        self.assertTrue(hasattr(self, 'last_recipe'), "Recipe message was not received")
        self.assertTrue(hasattr(self, 'last_recipe_history'), "Recipe history message was not received")
        
    def test_2_action_execution_flow(self):
        self.reset_message_flags()
        
        self.notify_action_pub.publish(self.test_data['action'])
        
        topics_to_check = ['high_level_action']
        self.assertTrue(self.wait_for_messages(topics_to_check), 
                        "Did not receive high level action message")
        
        self.assertTrue(hasattr(self, 'last_high_level_action'), 
                        "High level action message was not received")
                        
        self.reset_message_flags()
        topics_to_check = ['trajectory_status']
        self.assertTrue(self.wait_for_messages(topics_to_check, timeout=12), 
                        "Did not receive trajectory status message")
                        
    def test_3_battery_monitoring(self):
        self.reset_message_flags()
        
        topics_to_check = ['battery_level']
        self.assertTrue(self.wait_for_messages(topics_to_check, timeout=12), 
                        "Did not receive battery level message")
        
        self.assertTrue(hasattr(self, 'last_battery_level'), 
                        "Battery level message was not received")
        self.assertIsInstance(self.last_battery_level, float, 
                             "Battery level is not a float value")
        self.assertGreaterEqual(self.last_battery_level, 0.0, 
                               "Battery level is less than 0%")
        self.assertLessEqual(self.last_battery_level, 100.0, 
                            "Battery level is greater than 100%")
                            
    def test_4_slam_functionality(self):
        self.reset_message_flags()
        
        topics_to_check = ['current_position']
        self.assertTrue(self.wait_for_messages(topics_to_check, timeout=12), 
                        "Did not receive position update")
        
        self.assertTrue(hasattr(self, 'last_position'), 
                        "Position message was not received")
        self.assertIsInstance(self.last_position, Point, 
                             "Position is not a Point message")
                             
    def test_5_action_planning_to_execution(self):
        self.reset_message_flags()
        
        self.update_on_execution_actions_pub.publish(self.test_data['action'])
        
        topics_to_check = ['on_execution_actions']
        self.assertTrue(self.wait_for_messages(topics_to_check), 
                        "Did not receive on_execution_actions update")
        
        self.assertTrue(hasattr(self, 'last_on_execution_actions'), 
                        "On execution actions message was not received")
        
    def test_6_speaker_service(self):
        speaker_service = rospy.ServiceProxy('/speaker', Speaker)
        test_message = "This is a test message"
        
        try:
            response = speaker_service(test_message)
            self.assertIsNotNone(response, "Speaker service call failed")
        except rospy.ServiceException as e:
            self.fail(f"Speaker service call failed: {e}")
            
    def test_7_wheels_speed_service(self):
        wheels_service = rospy.ServiceProxy('/set_wheels_speed', WheelsSpeed)
        test_left_speed = 5.0
        test_right_speed = 5.0
        
        try:
            response = wheels_service(test_left_speed, test_right_speed)
            self.assertIsNotNone(response, "Wheels speed service call failed")
        except rospy.ServiceException as e:
            self.fail(f"Wheels speed service call failed: {e}")
            
    def test_8_joint_state_service(self):
        joint_service = rospy.ServiceProxy('/check_joint_state', CheckJointState)
        test_positions = [0.0] * 7
        test_velocities = [0.5] * 7
        test_efforts = [0.5] * 7
        
        try:
            response = joint_service(test_positions, test_velocities, test_efforts)
            self.assertIsNotNone(response, "Joint state service call failed")
        except rospy.ServiceException as e:
            self.fail(f"Joint state service call failed: {e}")

if __name__ == '__main__':
    rostest.rosrun('robot_system', 'integration_test', RobotSystemIntegrationTest)
