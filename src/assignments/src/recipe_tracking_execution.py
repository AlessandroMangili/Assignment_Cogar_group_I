#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import String, Bool, Int32
from assignments.msg import Action, Recipe, RecipeHistory, NewRecipeHistory, OnExecutionActions, ErrorMessage

class RecipeTrackingExecution:
    def __init__(self):
        rospy.init_node('recipe_tracking_execution', anonymous=True)
        
        self.recipe_pub = rospy.Publisher('/recipe', Recipe, queue_size=10)
        self.recipe_history_pub = rospy.Publisher('/recipe_history', RecipeHistory, queue_size=10)
        self.on_execution_actions_pub = rospy.Publisher('/on_execution_actions', OnExecutionActions, queue_size=10)
        self.new_recipe_history_pub = rospy.Publisher('/new_recipe_history', NewRecipeHistory, queue_size=10)
        self.error_pub = rospy.Publisher('/error_code', Int32,  queue_size=10)
        
        self.initialize_recipe_history_pub = rospy.Publisher('/initialize_recipe_history', Bool, queue_size=10)
        self.update_recipe_history_pub = rospy.Publisher('/update_recipe_history', Bool, queue_size=10)
       
        rospy.Subscriber('/update_recipe', String, self.update_recipe)
        rospy.Subscriber('/initialize_recipe_history', Bool, self.initialize_recipe_history)
        rospy.Subscriber('/update_recipe_history', Bool, self.update_recipe_history)
        rospy.Subscriber('/update_on_execution_actions', Action, self.update_on_execution_actions)
        rospy.Subscriber('/command_recipe_history', String, self.command_recipe_history)
        rospy.Subscriber('/error_message', ErrorMessage, self.error_callback)
        
        self.recipe = Recipe()
        self.recipe_history = RecipeHistory()
        self.on_execution_actions = OnExecutionActions()
        self.new_recipe_history = NewRecipeHistory()
        
        self.dummy_action = Action()
        self.dummy_action.label = "Grabbing"
        self.dummy_action.order = 1
        self.dummy_action.mandatory = True
        self.dummy_action.prerequisites = []
        self.dummy_action.tools = [1]
        self.dummy_action.ingredients = [1]
        
        rospy.loginfo("Recipe Tracking and Execution initialized")

    def error_callback(self, msg):
        error = msg
        if error.id_component == 1:
            rospy.logerr(f"Received an error from the error handler: {error}")
    
    def update_recipe(self, req):
        rospy.loginfo("Updating recipe from internet")
        rospy.sleep(2)
        
        self.recipe = Recipe()
        self.recipe.actions = [self.dummy_action]
        
        self.recipe_pub.publish(self.recipe)
        self.initialize_recipe_history_pub.publish(Bool(data=True))
        
        rospy.loginfo("Recipe updated successfully")
        return True
    
    def initialize_recipe_history(self, req):
        rospy.loginfo("Initializing recipe history")
        
        self.recipe_history.actions = self.recipe.actions
        self.recipe_history.executed = [False] * len(self.recipe_history.actions)
        self.recipe_history.execution_order = [0] * len(self.recipe_history.actions)
        
        self.recipe_history_pub.publish(self.recipe_history)
        
        rospy.loginfo("Recipe history initialized")
        return True
    
    def update_recipe_history(self, req):
        rospy.loginfo("Updating recipe history based on on_execution_actions")

        is_executed = random.choice([True, False])
        
        if is_executed:
            if hasattr(self, 'recipe_history') and self.recipe_history.executed is not None:
                self.recipe_history.executed = [True]
                self.recipe_history.execution_order = [1]
                
                self.recipe_history_pub.publish(self.recipe_history)
                rospy.loginfo("Dummy action marked as executed")
            else:
                rospy.logwarn("No recipe history to update")
        else:
            rospy.loginfo("Dummy action not executed (random choice)")

        return True
    
    def update_on_execution_actions(self, req):
        rospy.loginfo(f"Received request to update on execution actions with label: {req.label}")
        
        self.on_execution_actions.actions = self.recipe_history.actions
        self.on_execution_actions.in_execution = [True]
        self.on_execution_actions.time_remaining = [random.randint(1, 10)]
        self.on_execution_actions.interruptable = [random.choice([True, False])]
        
        self.on_execution_actions_pub.publish(self.on_execution_actions)
        self.update_recipe_history_pub.publish(Bool(data=True))
        
        rospy.loginfo(f"Now executing action: {req.label}")
        return True
    
    def command_recipe_history(self, req):
        rospy.loginfo(f"Command recipe history: {req.data}")
        
        self.new_recipe_history = NewRecipeHistory()
        self.new_recipe_history.actions = self.recipe_history.actions
        self.new_recipe_history.executed = self.recipe_history.executed
        self.new_recipe_history.execution_order = self.recipe_history.execution_order
        self.new_recipe_history.new_actions = [False] * len(self.recipe_history.actions)
        
        if "add" in req.data.lower():
            new_action = Action()
            new_action.label = "Cutting"
            new_action.order = len(self.recipe_history.actions) + 1
            new_action.mandatory = random.choice([True, False])
            new_action.prerequisites = random.sample(range(1, new_action.order), k=random.randint(0, min(2, new_action.order-1)))
            new_action.tools = random.sample(range(1, 11), k=random.randint(1, 2))
            new_action.ingredients = random.sample(range(1, 21), k=random.randint(1, 3))
            
            self.new_recipe_history.actions.append(new_action)
            self.new_recipe_history.executed.append(False)
            self.new_recipe_history.execution_order.append(0)
            self.new_recipe_history.new_actions.append(True)
        
        self.new_recipe_history_pub.publish(self.new_recipe_history)
        
        rospy.loginfo("New recipe history updated")
        return "Recipe history updated with human command"

if __name__ == '__main__':
    try:
        node = RecipeTrackingExecution()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
