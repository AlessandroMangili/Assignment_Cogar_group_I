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
        rospy.Subscriber('/on_execution_actions', OnExecutionActions, self.on_execution_actions_callback)
        rospy.Subscriber('/new_recipe_history', NewRecipeHistory, self.new_recipe_history_callback)
        rospy.Subscriber('/object_tracking', String, self.object_tracking_callback)
        rospy.Subscriber('/robot_state', RobotState, self.robot_state_callback)
        
        rospy.wait_for_service('/speaker')
        self.speaker_client = rospy.ServiceProxy('/speaker', Speaker)
        
        self.planning_timer = rospy.Timer(rospy.Duration(0.1), self.planning_cycle)
        
        self.best_action = Action()
        
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
    
    def on_execution_actions_callback(self, msg):
        self.on_execution_actions = msg
    
    def new_recipe_history_callback(self, msg):
        self.new_recipe_history = msg
    
    def object_tracking_callback(self, msg):
        self.object_tracking_data = msg.data
    
    def robot_state_callback(self, msg):
        self.robot_state = msg
    
    def planning_cycle(self, event):
        if None in [self.recipe]:
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
        
        if None in [self.object_tracking_data]:
            return False
        
        if len(self.on_execution_actions.actions) > 0:
            for idx, current_action in enumerate(self.on_execution_actions.actions):
                if self.on_execution_actions.in_execution[idx]:
                    for prereq_order in current_action.prerequisites:
                        prereq_idx = next((i for i, action in enumerate(self.recipe_history.actions) if action.order == prereq_order), -1)
                        
                        if prereq_idx >= 0 and not self.recipe_history.executed[prereq_idx]:
                            rospy.logwarn(f"Action {current_action.label} (order {current_action.order}) is executing but prerequisite {prereq_order} not met!")
                            if current_action.mandatory:
                                return True
        
        missing_ingredients = []
        if "missing" in self.object_tracking_data.lower():
            parts = self.object_tracking_data.split(":")
            if len(parts) > 1:
                missing_ingredients = [item.strip() for item in parts[1].split(",")]
            
            rospy.logwarn(f"Missing ingredients detected: {missing_ingredients}")
            
            for idx, action in enumerate(self.recipe_history.actions):
                if not self.recipe_history.executed[idx]:
                    required_ingredients = [self.get_ingredient_name(ing_id) for ing_id in action.ingredients]
                    if any(ing in missing_ingredients for ing in required_ingredients):
                        if action.mandatory:
                            rospy.logerr(f"Mandatory action {action.label} requires missing ingredients: {missing_ingredients}")
                            return True
        
        for idx, action in enumerate(self.recipe_history.actions):
            if self.recipe_history.executed[idx]:
                for prereq_order in action.prerequisites:
                    prereq_idx = next((i for i, a in enumerate(self.recipe_history.actions) if a.order == prereq_order), -1)
                    
                    if prereq_idx >= 0 and not self.recipe_history.executed[prereq_idx]:
                        rospy.logwarn(f"Human executed action {action.label} without completing prerequisite {prereq_order}")
                        if action.mandatory:
                            return True
        
        return False
    
    def get_ingredient_name(self, ingredient_id):
        return f"ingredient_{ingredient_id}"
    
    def update_best_action(self):
        rospy.loginfo("Updating best action")
        self.best_action = None
        
        if None in [self.recipe_history]:
            return False
        
        if self.recipe_history and all(self.recipe_history.executed):
            rospy.loginfo("All actions executed!")
            return True
        
        if self.on_execution_actions and any(self.on_execution_actions.in_execution):
            for i, in_exec in enumerate(self.on_execution_actions.in_execution):
                if in_exec:
                    if self.on_execution_actions.time_remaining[i] > 0:
                        self.on_execution_actions.time_remaining[i] -= 1
                    else:
                        self.on_execution_actions.in_execution[i] = False
                        rospy.loginfo(f"Action completed: {self.on_execution_actions.actions[i].label}")
        
        if self.new_recipe_history and self.new_recipe_history.new_actions:
            for new_action in self.new_recipe_history.actions:
                in_history = any(action.order == new_action.order for action in self.recipe_history.actions)
                in_execution = any(action.order == new_action.order for action in self.on_execution_actions.actions)
                
                if not in_history and not in_execution:
                    rospy.loginfo(f"New action found: {new_action.label} (order: {new_action.order})")
                    self.best_action = new_action
                    return False
        
        for idx, action in enumerate(self.recipe_history.actions):
            if not self.recipe_history.executed[idx]:
                prereqs_met = True
                for prereq_order in action.prerequisites:
                    prereq_idx = next((i for i, a in enumerate(self.recipe_history.actions) 
                                     if a.order == prereq_order), -1)
                    
                    if prereq_idx >= 0 and not self.recipe_history.executed[prereq_idx]:
                        prereqs_met = False
                        break
                
                if prereqs_met:
                    self.best_action = action
                    rospy.loginfo(f"Selected next action: {action.label} (order: {action.order})")
                    return False
                else:
                    rospy.logwarn(f"Action {action.label} not executable: prereqs_met={prereqs_met}")
        
        rospy.logwarn("No executable actions found, but recipe not finished")
        return False

if __name__ == '__main__':
    try:
        node = ActionPlanning()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
