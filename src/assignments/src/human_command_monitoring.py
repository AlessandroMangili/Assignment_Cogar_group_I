#!/usr/bin/env python3
import rospy
import random
from std_msgs.msg import String, Int32
from assignments.msg import RobotState, Recipe, RecipeHistory, OnExecutionActions, ErrorMessage
from assignments.srv import Speaker

class HumanCommandMonitoring:
    def __init__(self):
        rospy.init_node('human_command_monitoring', anonymous=True)
        
        self.update_recipe_pub = rospy.Publisher('/update_recipe', String, queue_size=10)
        self.command_pub = rospy.Publisher('/command_recipe_history', String, queue_size=10)
        
        rospy.Subscriber('/microphone_input', String, self.audio_callback)
        rospy.Subscriber('/object_tracking', String, self.object_tracking_callback)
        rospy.Subscriber('/error_message', ErrorMessage, self.error_callback)
        self.error_pub = rospy.Publisher('/error_code', Int32,  queue_size=10)
        
        self.robot_state = RobotState()
        self.robot_state = "No Recipe"
        
        """used in the real architecture
        rospy.Subscriber('/robot_state', RobotState, self.robot_state_callback)
        rospy.Subscriber('/recipe', Recipe, self.recipe_callback)
        rospy.Subscriber('/recipe_history', RecipeHistory, self.recipe_history_callback)
        rospy.Subscriber('/on_execution_actions', OnExecutionActions, self.on_execution_actions_callback)
        self.recipe = Recipe()
        self.recipe_history = RecipeHistory()
        self.on_execution_actions = OnExecutionActions()"""
        
        self.possible_recipes = ["pasta carbonara", "insalata greca", "risotto ai funghi", "pizza margherita", "lasagna", "tiramisu", "impossible"]
        self.possible_ingredients = ["pomodori", "pasta", "riso", "funghi", "mozzarella", "basilico", "uova", "farina", "zucchero", "carne"]
        self.timer = rospy.Timer(rospy.Duration(10), self.generate_random_input)
        
        rospy.wait_for_service('/speaker')
        self.speaker_client = rospy.ServiceProxy('/speaker', Speaker)
        
        rospy.loginfo("Human Command Monitoring initialized")
    
    """used in the real architecture
    def robot_state_callback(self, msg):
        self.robot_state = msg
        
    def recipe_callback(self, msg):
        self.recipe = msg
    
    def recipe_history_callback(self, msg):
        self.recipe_history = msg
    
    def on_execution_actions_callback(self, msg):
        self.on_execution_actions = msg"""
    
    def error_callback(self, msg):
        error = msg
        if error.id_component == 3:
            rospy.logerr(f"Received an error from the error handler: {error}")
        
    
    def speak(self, message):
        try:
            response = self.speaker_client(message)
            if not response.success:
                rospy.logwarn("Speaker service failed to reproduce the message")
                rospy.sleep(1)
                retry_response = self.speaker_client(message)
                if not retry_response.success:
                    self.error_pub.publish(4)
                    #rospy.logerr("Speaker service failed again")
            return response.success
        except rospy.ServiceException as e:
            self.error_pub.publish(4)
            #rospy.logerr(f"Service call failed: {e}")
            return False
    
    def generate_random_input(self, event):
        input_type = random.choice(["recipe", "ingredients", "command"])
        
        if self.robot_state == "No Recipe":
            if input_type == "recipe":
                recipe_name = random.choice(self.possible_recipes)
                simulated_input = f"recipe {recipe_name}"
                rospy.loginfo(f"Simulated voice input: {simulated_input}")
            
                msg = String()
                msg.data = simulated_input
                self.audio_callback(msg)
            
            elif input_type == "ingredients":
                num_ingredients = random.randint(1, 3)
                selected_ingredients = random.sample(self.possible_ingredients, num_ingredients)
                ingredients_str = " ".join(selected_ingredients)
                simulated_input = f"ingredients {ingredients_str}"
                rospy.loginfo(f"Simulated voice input: {simulated_input}")
            
                msg = String()
                msg.data = simulated_input
                self.audio_callback(msg)
        elif input_type == "command":
            commands = ["next step", "repeat", "previous step", "pause", "continue", "stop"]
            command = random.choice(commands)
            rospy.loginfo(f"Simulated command: {command}")
            
            msg = String()
            msg.data = command
            self.audio_callback(msg)
    
    def audio_callback(self, msg):
        rospy.loginfo(f"Received audio: {msg.data}")
        
        if len(msg.data) < 5:
            self.speak("Please repeat")
            return
            
        if self.robot_state == "No Recipe":
            if "recipe" in msg.data.lower():
                recipe_name = msg.data.lower().replace("recipe", "").strip()
                rospy.loginfo(f"Recipe requested: {recipe_name}")
                
                if self.simulate_wifi_connection():
                    result = self.search_recipe(recipe_name)
                    if result:
                        self.speak(f"OK, let's start the recipe {recipe_name}")
                        self.robot_state = "Recipe"
                        self.update_recipe_pub.publish(recipe_name)
                    else:
                        self.speak("Recipe not found")
                else:
                    self.speak("Please connect to Wi-Fi")
            elif "ingredients" in msg.data.lower():
                ingredients = msg.data.lower().replace("ingredients", "").strip().split()
                rospy.loginfo(f"Ingredients mentioned: {ingredients}")
                
                if self.simulate_wifi_connection():
                    recipes = self.propose_recipes(ingredients)
                    if recipes:
                        self.speak(f"I found these recipes: {', '.join(recipes)}")
                    else:
                        self.speak("No proposes found with these ingredients")
                else:
                    self.speak("Please connect to Wi-Fi")
        else:
            self.validate_command(msg.data)
    
    def object_tracking_callback(self, msg):
        rospy.loginfo(f"Object tracking: {msg.data}")
        
        if "ingredient" in msg.data.lower() and self.robot_state == "No Recipe":
            ingredients = msg.data.lower().split("ingredient:")[1].strip().split()
            rospy.loginfo(f"Ingredients seen: {ingredients}")
            
            if self.simulate_wifi_connection():
                recipes = self.propose_recipes(ingredients)
                if recipes:
                    self.speak(f"I see you have {', '.join(ingredients)}. I found these recipes: {', '.join(recipes)}")
                else:
                    self.speak("No proposes found with these ingredients")
            else:
                self.speak("Please connect to Wi-Fi")
    
    def search_recipe(self, recipe_name):
        rospy.loginfo(f"Searching for recipe: {recipe_name}")
        rospy.sleep(2)
        
        if recipe_name == "impossible":
            return False
        
        return True
    
    def validate_command(self, command):
        rospy.loginfo(f"Validating command: {command}")
        
        success = random.random() > 0.2
        
        if success:
            self.speak("OK")
            self.command_pub.publish(command)
        else:
            self.speak("Sorry, I can't do that")
            
        return success
    
    def propose_recipes(self, ingredients):
        rospy.loginfo(f"Finding recipes with: {ingredients}")
        rospy.sleep(2)
        
        if not ingredients:
            return []
        
        if "pomodori" in ingredients or "mozzarella" in ingredients:
            return ["Pizza Margherita", "Insalata Caprese", "Pasta al Pomodoro"]
        elif "pasta" in ingredients or "uova" in ingredients:
            return ["Carbonara", "Pasta all'uovo", "Frittata"]
        elif "riso" in ingredients or "funghi" in ingredients:
            return ["Risotto ai funghi", "Riso al salto", "Zuppa di funghi"]
        else:
            return ["Insalata mista", "Minestrone", "Zuppa"]
    
    def simulate_wifi_connection(self):
        return random.random() < 0.9

if __name__ == '__main__':
    try:
        node = HumanCommandMonitoring()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
