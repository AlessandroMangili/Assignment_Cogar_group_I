#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import String, Bool
from assignments.msg import Action, Recipe, RecipeHistory, NewRecipeHistory, OnExecutionActions

class RecipeTrackingExecution:
    def __init__(self):
        rospy.init_node('recipe_tracking_execution', anonymous=True)
        
        self.recipe_pub = rospy.Publisher('/recipe', Recipe, queue_size=10)
        self.recipe_history_pub = rospy.Publisher('/recipe_history', RecipeHistory, queue_size=10)
        self.on_execution_actions_pub = rospy.Publisher('/on_execution_actions', OnExecutionActions, queue_size=10)
        self.new_recipe_history_pub = rospy.Publisher('/new_recipe_history', NewRecipeHistory, queue_size=10)
        
        self.initialize_recipe_history_pub = rospy.Publisher('/initialize_recipe_history', Bool, queue_size=10)
        self.update_recipe_history_pub = rospy.Publisher('/update_recipe_history', Bool, queue_size=10)
       
        rospy.Subscriber('/update_recipe', String, self.update_recipe)
        rospy.Subscriber('/initialize_recipe_history', Bool, self.initialize_recipe_history)
        rospy.Subscriber('/update_recipe_history', Bool, self.update_recipe_history)
        rospy.Subscriber('/update_on_execution_actions', Action, self.update_on_execution_actions)
        rospy.Subscriber('/command_recipe_history', String, self.command_recipe_history)
        
        self.recipe = Recipe()
        self.recipe_history = RecipeHistory()
        self.on_execution_actions = OnExecutionActions()
        self.new_recipe_history = NewRecipeHistory()
        
        rospy.loginfo("Recipe Tracking and Execution initialized")
    
    def update_recipe(self, req):
        rospy.loginfo("Updating recipe from internet")
        rospy.sleep(2)
        
        self.recipe.actions = []

    	num_actions = random.randint(4, 8)
    	available_orders = list(range(1, num_actions + 1))
    	random.shuffle(available_orders)

    	for i in range(num_actions):
            action = Action()
            action.label = f"Action{random.randint(100, 999)}"
            action.order = available_orders[i]
            action.mandatory = random.choice([True, False])

            if i > 0:
            	possible_prerequisites = list(range(1, available_orders[i]))
            	prerequisites = random.sample(possible_prerequisites, k=random.randint(0, len(possible_prerequisites)))
            	action.prerequisites = prerequisites
            else:
            	action.prerequisites = []

            tools_pool = list(range(1, 20))
            ingredients_pool = list(range(1, 200))

            action.tools = random.sample(tools_pool, k=random.randint(1, 2))
            action.ingredients = random.sample(ingredients_pool, k=random.randint(1, 3))

            self.recipe.actions.append(action)
        
        self.recipe_pub.publish(self.recipe)
        self.initialize_recipe_history_pub.publish(Bool(data=True))
        
        rospy.loginfo("Recipe updated successfully")
        return True
    
    def initialize_recipe_history(self, req):
        rospy.loginfo("Updating recipe history")
        
        self.recipe_history.actions = self.recipe.actions
        self.recipe_history.executed = [False] * len(self.recipe.actions)
        executed_indices = [i for i, executed in enumerate(self.recipe_history.executed) if executed]
    	orders = list(range(1, len(executed_indices)+1))
    	random.shuffle(orders)

    	self.recipe_history.execution_order = [0] * len(self.recipe.actions)
    	for idx, exec_idx in enumerate(executed_indices):
            self.recipe_history.execution_order[exec_idx] = orders[idx]
        
        self.recipe_history_pub.publish(self.recipe_history)
        
        rospy.loginfo("Recipe history updated")
        return True
    
    def update_recipe_history(self, req):
	rospy.loginfo("Updating recipe history based on on_execution_actions")

	if not self.on_execution_actions.actions:
	    rospy.logwarn("No actions in execution to update.")
	    return False

	current_action_label = self.on_execution_actions.actions[0].label
	action_idx = -1

	for idx, action in enumerate(self.recipe_history.actions):
	    if action.label == current_action_label:
		action_idx = idx
		break

	if action_idx == -1:
    	    rospy.logwarn(f"Action '{current_action_label}' not found in recipe history. Adding it directly.")

    	    self.recipe_history.actions.append(self.on_execution_actions.actions[0])
    	    self.recipe_history.executed.append(False)
    	    self.recipe_history.execution_order.append(sum(1 for executed in self.recipe_history.executed if executed))

    	    self.recipe_history_pub.publish(self.recipe_history)

    	    rospy.loginfo(f"Action '{current_action_label}' added in recipe_history.")
    	    return True

	self.recipe_history.executed[action_idx] = True
	self.recipe_history.execution_order[action_idx] = sum(1 for executed in self.recipe_history.executed if executed)

	self.recipe_history_pub.publish(self.recipe_history)

	rospy.loginfo(f"Marked action '{current_action_label}' as executed.")
	return True
    
    def update_on_execution_actions(self, req):
	rospy.loginfo(f"Received request to update on execution actions with label: {req.data}")

	requested_label = req.data.strip()
	found_action = None

	for action in self.recipe_history.actions:
	    if action.label == requested_label:
		found_action = action
		break

	if not found_action:
	    for idx, is_new in enumerate(self.new_recipe_history.new_actions):
		if is_new and self.new_recipe_history.actions[idx].label == requested_label:
		    found_action = self.new_recipe_history.actions[idx]
		    break

	self.on_execution_actions.actions = [found_action]
	self.on_execution_actions.in_execution = [True]
	self.on_execution_actions.time_remaining = [random.randint(5, 20)]
	self.on_execution_actions.interruptable = [random.choice([True, False])]

	self.on_execution_actions_pub.publish(self.on_execution_actions)
	self.update_recipe_history_pub.publish(Bool(data=True))

	rospy.loginfo(f"Now executing action: {found_action.label}")
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
            new_action.label = f"NewAction{random.randint(100, 999)}"
            new_action.order = min([action.order for action in self.new_recipe_history.actions]) - 1
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
