#! /usr/bin/env python
import rospy
from robot_gui import robot_gui
import time
import speech_recognition as sr
from robot_gui import robot_gui
from run_scripts.srv import *
from std_msgs.msg import String

class voice(robot_gui):
	def setup_speech(self):
		    print "in speech setup"
		    self.rec.pause_threshold = .5
		    self.rec.dynamic_energy_threshold = False
		    with self.mic as source:
		        self.rec.adjust_for_ambient_noise(source, 3)

	def send_command(self,command):
		#client
		print('waiting')
		rospy.wait_for_service('/receive_command')
		try:
			print('done waiting')
			parse_command = rospy.ServiceProxy('receive_command', Command)
			resp1 = parse_command(command)
			if resp1.status:
				self.label_dict['thoughts'] = 'I have successfully intiated the command ['+command+']'
			else:
				self.label_dict['thoughts'] = 'Could not initiate the command ['+command+']'

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def get_fridge_suggestion_counter(self):
		self.fridge_counter +=1
		if self.fridge_counter > 2:
			self.fridge_counter = 0
			self.init_suggestion()
		return self.fridge_counter


	def get_microwave_suggestion_counter(self):
		self.microwave_counter +=1
		if self.microwave_counter >6:
			self.microwave_counter = 0
			self.init_suggestion()
		return self.microwave_counter

	def init_suggestion(self):
		self.label_dict['first_suggestion'] = 'Open the fridge'
		self.label_dict['second_suggestion'] = ''
		self.which_branch = 2
		self.fridge_index = 1
		self.mic_index = 1


	def suggestion_printer(self,dialog):
		if  ('open' in dialog and 'fridge' in dialog):
			self.label_dict['first_suggestion'] = self.suggestion_tree[0][0]
			self.label_dict['second_suggestion'] = self.suggestion_tree[1][0]
			self.fridge_index = 0
			self.mic_index = 0
			self.fridge_counter = 0
			self.microwave_counter = 0
			self.which_branch = 3

		elif 'bottle' in dialog:
			self.which_branch = 0 
			# self.fridge_index += 1

		elif 'open' in dialog and 'microwave' in dialog:
			self.which_branch = 1
			# self.microwave_counter += 1
			# self.mic_index+=1


        
		if self.which_branch == 0:
			self.fridge_index = self.get_fridge_suggestion_counter()
			if self.fridge_index <=1:
				self.label_dict['first_suggestion'] = self.suggestion_tree[0][self.fridge_index]
				self.label_dict['second_suggestion'] = ''
			else:
				self.init_suggestion()



		elif self.which_branch == 1:
			
			if 'cook' in dialog:
				self.microwave_counter+=1
			self.mic_index = self.get_microwave_suggestion_counter()
			if self.mic_index <=5:
				self.label_dict['first_suggestion'] = self.suggestion_tree[1][self.mic_index]
				if 'close' in dialog and 'microwave' in dialog:
					self.label_dict['second_suggestion'] = 'Cook for ___ seconds'
					
				else:
					self.label_dict['second_suggestion'] = ''

			else:
				self.init_suggestion()
			


	def speech_callback(self, recognizer, audio):
		    # Defining Commands to be accepted
		    global t2s, dialog
		    sens = 1
		    commands = ["open gripper", "close gripper"]
		    dialog = "Listening..."
		    print("listening")
		    try:
		        commandIter = [command[0] for command in commands]
		        global rawCommand
		        rawCommand = recognizer.recognize_google_cloud(audio_data=audio, language='en-US', preferred_phrases=commands)
		        dialog = rawCommand.lower()
		        print("understood")
		        print(dialog)
		        self.label_dict['dialog'] = dialog
		        self.label_dict['command'] = ""

				#screen voice input for relevant command 

		        if dialog in self.COMMANDS or 'cook' in dialog or 'floor' in dialog or 'microwave' in dialog:
					self.label_dict['command'] = dialog
					self.send_command(dialog)
					self.label_dict['thoughts'] = ''
					self.suggestion_printer(dialog)
		       
				
		        else:
					self.label_dict['thoughts'] = dialog + ' is NOT A COMMAND!'

				
				


		    except sr.UnknownValueError:
		        dialog = "Listening..."
		        pass
		    except sr.RequestError as e:
		        print("Recognition error; {}".format(e))



	def __init__(self):
			rospy.init_node('vision_move', disable_signals=True)


			self.COMMANDS = "move to origin open the fridge get a water bottle extra you are in the kitchen \
			place on the table close the fridge puts food in the microwave pick up object from the floor extra \
			put food in a microwave put food in the microwave  take a water bottle close the microwave is\
			cook for seconds get food from the microwave get food from microwave extra\
			get food start the microwave close the fridge close gripper stop the microwave holding something\
			fridge is open microwave is open food in microwave microwave is on extra\
			hand is empty starts the microwave fridge is closed microwave is closed microwave is empty extra\
			let's go let go open the microwave pick up object from the floor drop object pour water in the cup extra\
			take object from the shelf gets food from the microwave open gripper close a microwave dummy is here"
			
			self.command_publisher = rospy.Publisher('/command', String, queue_size=1)
			#Speech recognition variables
			robot_gui.__init__(self)
			self.rec = sr.Recognizer()
			self.mic = sr.Microphone()

			self.fridge_counter = 1; self.microwave_counter = 1
			print len(self.COMMANDS)
			#Environment variables
			self.env = ({'fridgeOpen': False, 'hasBottle': False, 'bottleOnTable':False, 
				'bottleInFridge': True, 'microwaveOpen': False, 'holdingSomething': False, 
				'microwaveOn': False, 'foodInMicrowave': False, 'foodInFridge': True, 
				'foodOnTable': False, 'robotlocalized': False, 'mobileBaseActivated': False,
				'distanceToKitchen' : 0.0, 'distanceToTable' : 0.0})

			# rospy.Subscriber('/head_kinect/ar_pose_marker', AlvarMarkers, self.master_callback)
			#rospy.Subscriber('/mobile_base_cam/ar_pose_marker', AlvarMarkers, self.auto_park_callback)

			fridge_branch = ['Get a water bottle', 'Close the fridge']
			microwave_branch = ['Open the microwave', 'Put food in the microwave', 'Close the microwave',
			'Start the microwave','Stop the microwave','Get food from the microwave']
			self.suggestion_tree = [fridge_branch, microwave_branch]
			self.which_branch = 3

			self.setup_speech()
			self.stopListening = self.rec.listen_in_background(self.mic, self.speech_callback, phrase_time_limit=4)
			
			self.update_root()

			rospy.spin()




if __name__ == '__main__':
	try:
		
		v= voice()
		#while not rospy.is_shutdown():
		#g.update_root() 
	except KeyboardInterrupt:
		rospy.loginfo('Shutting down')