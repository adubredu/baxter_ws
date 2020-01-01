#! /usr/bin/env python
import rospy
import time
import speech_recognition as sr
from robot_gui import robot_gui

once = False

class gotomarker(robot_gui):

  		####################################
  		# SPEECH RECOGNITION METHODS	####
  		####################################

  		def setup_speech(self):
		    print "in speech setup"
		    self.rec.pause_threshold = .5
		    self.rec.dynamic_energy_threshold = False
		    with self.mic as source:
		        self.rec.adjust_for_ambient_noise(source, 3)


		def speech_callback(self, recognizer, audio):
		    # Defining Commands to be accepted
		    global t2s, dialog
		    #credsJson = ""
		    #with open('baxter-helper-bot-gspeechcreds.json', 'r') as gspeechcreds:
		        #credsJson = gspeechcreds.read()
		    
		    sens = 1
		    commands = ["take","close gripper", "open gripper", "stop", "stop", "stop", "open the microwave",
		                "faster", "slower", "open the fridge", "move to zero", "get a water bottle", "fridge", 
		                "place on", "table", "fridge is open", "holding something", "fridge is closed", "close the microwave","start the microwave",
		                "hand is empty", "put food in the microwave", "get food from the microwave", "start", "turn off", "continue", "cook for",
		                "put", "food", "is open", "get the food", "activate auto localization","activate mobile base",
		                "robot is localized","move arm lower","move arm higher", "put water bottle on the table", "ground grab mode", "table grab mode",
		                "pick object from floor", "forward", "backward", "left", "right", "pause", "drop object", "drop in bin"]
		    dialog = "Listening..."
		    print("listening")
		    #print('Distance to kitchen: ',self.dist_to_kitchen)
		    #print('Distance to table: ',self.dist_to_table)
		    #self.env['distanceToKitchen'] = self.dist_to_kitchen
		    #self.env['distanceToTable'] = self.dist_to_table
		    try:
		        commandIter = [command[0] for command in commands]
		        global rawCommand
		        rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', preferred_phrases=commands)
		        dialog = rawCommand
		        print("understood")
		        print(dialog)
		        self.interprete_command(rawCommand)

		        self.label_dict['dialog'] = dialog
		        #self.gui.update_labels(self.env)
		        #self.gui.update_command_display()

		    except sr.UnknownValueError:
		        dialog = "Listening..."
		        pass
		    except sr.RequestError as e:
		        print("Recognition error; {}".format(e))





		
			### End task related commands ###



		def reset_environment(self):
			self.env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 
				'bottleInFridge': True, 'microwaveOpen': False, 'holdingSomething': False, 
				'microwaveOn': False, 'foodInMicrowave': False, 'foodInFridge': True, 
				'foodOnTable': False, 'robotlocalized': True, 'mobileBaseActivated': False,
				'distanceToKitchen' : self.dist_to_kitchen, 'distanceToTable' : self.dist_to_table})



		def set_environment_state(self, command):
			if 'fridge is open' in command:
				 self.env['fridgeOpen'] = True
				 self.label_dict['command'] = command
		        
			elif "fridge is closed" in command:
				self.env['fridgeOpen'] = False
				self.label_dict['command'] = command
		        
			elif "holding something" in command:
				self.env['hasBottle'] = True
				self.env['holdingSomething'] = True
				self.label_dict['command'] = command
		        
			elif "hand is empty" in command:
				self.lGripper.open()
				self.env['hasBottle'] = False
				self.env['holdingSomething'] = True
				self.label_dict['command'] = command
		        
			elif "microwave is open" in command:
				self.env['microwaveOpen'] = True
				self.label_dict['command'] = command
		        
			elif "microwave is closed" in command:
				self.env['microwaveOpen'] = False
				self.label_dict['command'] = command
		        
			elif 'microwave is on' in command:
				self.env['microwaveOn'] = True
				self.label_dict['command'] = command
		        
			elif 'microwave is off' in command:
				self.env['microwaveOn'] = False
				self.label_dict['command'] = command 
		       
			elif 'food in microwave' in command:
				self.env['foodInMicrowave'] = True
				self.env['foodInFridge'] = False
				self.env['foodOnTable'] = False
		        self.label_dict['command'] = command

			elif 'microwave is empty' in command:
				self.env['foodInMicrowave'] = False
				self.label_dict['command'] = command
		        
			elif 'robot is localized' in command:
				self.env['robotlocalized'] = True
				self.label_dict['command'] = command

		        
			elif 'food in fridge' in command:
				self.env['foodInFridge'] = True
				self.label_dict['command'] = command
				
			elif 'food on table' in command:
				self.env['foodOnTable'] = True
				self.env['foodInFridge'] = False
				self.env['foodInMicrowave'] = False
				self.label_dict['command'] = command

			elif "don't move" in command:
				self.activate_auto_park = False
				self.label_dict['command'] = command
				




		######################################
		##   VOICE    COMMAND INTERPRETATION
		######################################
		def interprete_command(self,command):
			
			pass

		
		    

		def __init__(self):
			rospy.init_node('vision_move', disable_signals=True)			
			#gui 
			# self.gui = robot_gui()

			#Speech recognition variables
			self.rec = sr.Recognizer()
			self.mic = sr.Microphone()

			
			#Environment variables
			self.env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 
				'bottleInFridge': True, 'microwaveOpen': False, 'holdingSomething': False, 
				'microwaveOn': False, 'foodInMicrowave': False, 'foodInFridge': True, 
				'foodOnTable': False, 'robotlocalized': False, 'mobileBaseActivated': False,
				'distanceToKitchen' : 0.0, 'distanceToTable' : 0.0})

			
			#move_to_goal_joint_angle(self.lLimb, self.origin, self.pause_event)
			rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.master_callback)
			#rospy.Subscriber('/mobile_base_cam/ar_pose_marker', AlvarMarkers, self.auto_park_callback)

			

			self.setup_speech()
			self.stopListening = self.rec.listen_in_background(self.mic, self.speech_callback, phrase_time_limit=4)
			
			self.update_root()

			
			
			
			rospy.spin()
			
			


if __name__=="__main__":
	
	try:
		go = gotomarker()

	except rospy.ROSInterruptException: pass

