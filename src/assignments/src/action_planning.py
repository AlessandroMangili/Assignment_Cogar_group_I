#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import String, Bool
from assignments.msg import Action, Recipe, RecipeHistory, NewRecipeHistory, OnExecutionActions, RobotState
from assignments.srv import Speaker

class ActionPlanning:
    def __init__(self):
        rospy.init_node('action_planning', anonymous=True)
        
        self.update_on_execution_actions_pub = rospy.Publisher('/update_on_execution_actions', Action, queue_size=10)
        self.notify_action_pub = rospy.Publisher('/notify_action', Action, queue_size=10)
        
        rospy.Subscriber('/recipe', Recipe, self.recipe_callback)
        rospy.Subscriber('/recipe_history', RecipeHistory, self.recipe_history_callback)
        self.recipe = Recipe()
        self.recipe_history = RecipeHistory()
        
        """used in the real architecture
        rospy.Subscriber('/on_execution_actions', OnExecutionActions, self.on_execution_actions_callback)
        rospy.Subscriber('/new_recipe_history', NewRecipeHistory, self.new_recipe_history_callback)
        rospy.Subscriber('/object_tracking', String, self.object_tracking_callback)
        rospy.Subscriber('/robot_state', RobotState, self.robot_state_callback)
        self.best_action = Action()
        self.robot_state = RobotState()
        self.on_execution_actions = OnExecutionActions()
        self.new_recipe_history = NewRecipeHistory()"""
        
        rospy.wait_for_service('/speaker')
        self.speaker_client = rospy.ServiceProxy('/speaker', Speaker)
        
        self.planning_timer = rospy.Timer(rospy.Duration(1), self.planning_cycle)
        
        rospy.loginfo("Action Planning initialized")
    
    def speak(self, message):
        try:
            response = self.speaker_client(message)
            if not response.success:
                rospy.logwarn("Speaker service failed to reproduce the message")
                rospy.sleep(1)
                retry_response = self.speaker_client(message)
                if not retry_response.success:
                    rospy.logerr("Speaker service failed again")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def recipe_callback(self, msg):
        self.recipe = msg
    
    def recipe_history_callback(self, msg):
        self.recipe_history = msg
    
    """used in the real architecture
    def on_execution_actions_callback(self, msg):
        self.on_execution_actions = msg
    
    def new_recipe_history_callback(self, msg):
        self.new_recipe_history = msg
    
    def object_tracking_callback(self, msg):
        self.object_tracking_data = msg.data
    
    def robot_state_callback(self, msg):
        self.robot_state = msg"""
    
    def planning_cycle(self, event):
        if not self.recipe:
            return
        
        is_recipe_failed = self.unexpected_condition_check()
        
        if is_recipe_failed:
            rospy.logerr("Recipe failed!")
            self.speak("Recipe Failed, please provide a new recipe")
            return
        else:
            is_recipe_finished = self.update_best_action()
        
            if is_recipe_finished:
                rospy.loginfo("Recipe finished!")
                self.speak("Recipe Finished")
            else:
                self.update_on_execution_actions_pub.publish(self.best_action)
                self.notify_action_pub.publish(self.best_action)
    
    def unexpected_condition_check(self):
        rospy.loginfo("Checking for unexpected conditions")
        
        return random.random() < 0.1
    
    def update_best_action(self):
        rospy.loginfo("Updating best action")
        self.best_action = None
        
        if not self.recipe_history:
            return False
        
        if self.recipe_history and isinstance(self.recipe_history.executed, list) and all(self.recipe_history.executed):
            rospy.loginfo("All actions executed")
            return True
        else:
            self.best_action = self.recipe_history.actions[0]
        
        return random.random() < 0.1

if __name__ == '__main__':
    try:
    	node = ActionPlanning()
    	rospy.spin()
    except rospy.ROSInterruptException:
        pass
