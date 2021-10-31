#!/usr/bin/python
import threading
from time import sleep
import evdev
from evdev import InputDevice, categorize, ecodes
from enum import Enum
import pygame
import RPi.GPIO as GPIO
from sys import exit


g_BUTTON_A = 0
g_BUTTON_B = 0
g_BUTTON_C = 0
g_BUTTON_D = 0
g_BUTTON_start = 0
g_BUTTON_rt = 0
g_BUTTON_lt = 0

#for pi B+???
LimitSwitch = 26        #Header pin 37 - Limit Switch
           #Outputs
g_StatusLED = 12        #Header pin 32 - Status LED    - Active High
g_Relay1 = 20           #Header pin 38 - Relay1 Torso  - Active Low
g_Relay2 = 16           #Header pin 36 - Relay2 Head   - Active Low
g_Relay3 = 7            #Header pin 26 - Relay3 Unused - Active Low
g_Relay4 = 21           #Header pin 40 - Relay4 Strobe - Active Low

g_soundChannelA = 0
g_soundChannelB = 0
g_soundChannelC = 0
g_sndA = 0
g_sndB = 0
g_sndC = 0

g_devices = []
#wait here until a logitech device is found
#TODO: make this better. Maybe use exceptions or give more information to the user
while len(g_devices) == 0:
	for path in evdev.list_devices():
		print('device: ', path)
		print(evdev.InputDevice(path)) #note: this print statement has to be here or it crashes
		device = evdev.InputDevice(path)
		if 'F710' in device.name:
			g_devices.append(device)
	sleep(1)  # sleep 1 second?

def setup():
	global g_soundChannelA
	global g_soundChannelB
	global g_soundChannelC
	global g_sndA
	global g_sndB
	global g_sndC


	#script using BCM GPIO
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)

	#inputs
	GPIO.setup(LimitSwitch, GPIO.IN)  #head limit switch

	#outputs
	GPIO.setup(g_StatusLED, GPIO.OUT)   #Status LED
	GPIO.setup(g_Relay1, GPIO.OUT)      #Torso
	GPIO.setup(g_Relay2, GPIO.OUT)      #Head
	GPIO.setup(g_Relay3, GPIO.OUT)      #Unused
	GPIO.setup(g_Relay4, GPIO.OUT)      #Strobe

	# Initialize Outputs
	GPIO.output(g_StatusLED, GPIO.HIGH)  # LED ON
	GPIO.output(g_Relay1, GPIO.HIGH)  # torso down
	GPIO.output(g_Relay2, GPIO.HIGH)  # head down
	GPIO.output(g_Relay3, GPIO.HIGH)  # Unused off
	GPIO.output(g_Relay4, GPIO.HIGH)  # strobe off

	# init. pygame for sound
	pygame.mixer.init()
	pygame.init()

	g_sndA = pygame.mixer.Sound("/home/pi/ted/Scream1.wav")
	g_sndA.set_volume(1)
	g_sndB = pygame.mixer.Sound("/home/pi/ted/Head1.wav")
	g_sndB.set_volume(1)
	g_sndC = pygame.mixer.Sound("/home/pi/ted/Laugh1.wav")
	g_sndC.set_volume(1)

	g_soundChannelA = pygame.mixer.Channel(1)
	g_soundChannelB = pygame.mixer.Channel(2)
	g_soundChannelC = pygame.mixer.Channel(3)

class myEvHandler (threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		#self.device = evdev.InputDevice('/dev/input/event0')
		self.device = g_devices[0]
		self._stop_event = threading.Event()
		
	def stop(self):
		self._stop_event.set()
		
	def run(self):
		global g_BUTTON_A
		global g_BUTTON_B
		global g_BUTTON_C
		global g_BUTTON_D
		global g_BUTTON_start
		global g_BUTTON_rt
		global g_BUTTON_rt
		print ("monitoring evdev")
		for event in self.device.read_loop():
			if event.type == evdev.ecodes.EV_KEY:
				#print("key down event with code of: ", event.code)
				if event.code == 304: #A
					g_BUTTON_A = event.value
				elif event.code == 305: #B
					g_BUTTON_B = event.value
				elif event.code == 307: #X
					g_BUTTON_C = event.value
				elif event.code == 308: #Y
					g_BUTTON_D = event.value
				elif event.code == 315: #start
					g_BUTTON_start = event.value
				elif event.code == 310:  # LT
					print("LT")
					g_BUTTON_lt = event.value
				elif event.code == 311:  # RT
					print("RT")
					g_BUTTON_rt = event.value

class myOutputter (threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self._stop_event = threading.Event()
		
	def stop(self):
		self._stop_event.set()

	def run(self):
		global g_BUTTON_A
		global g_BUTTON_B
		global g_BUTTON_C
		global g_BUTTON_D
		global g_BUTTON_start
		global g_BUTTON_rt
		global g_BUTTON_rt
		global g_soundChannelA
		global g_soundChannelB
		global g_soundChannelC
		global g_sndA
		global g_sndB
		global g_sndC
		
		while True:
			try:
				#full sequence
				if (g_BUTTON_A == True):
					print("Waiting for Head to be fully down.")
					while GPIO.input(LimitSwitch) == GPIO.LOW:
						sleep(0.01)  # wait for head to be fully down
					print("Strobe On.")
					GPIO.output(g_Relay4, GPIO.LOW)
					print("Torso up and play Scream1.wav.")
					g_soundChannelA.play(g_sndA)
					GPIO.output(g_Relay1, GPIO.LOW)
					sleep(2)
					print("Play Head1.wav.")
					g_soundChannelB.play(g_sndB)
					sleep(2.7)
					print("Lift Head.")
					GPIO.output(g_Relay2, GPIO.LOW)
					sleep(4)
					print("Strobe Off.")
					GPIO.output(g_Relay4, GPIO.HIGH)
					print("Lowering Head and waiting for it to be fully down.")
					GPIO.output(g_Relay2, GPIO.HIGH)
					while GPIO.input(LimitSwitch) == GPIO.LOW:
						sleep(0.01)  # wait for head to be fully down
					print("Lowering Torso and playing Laugh1.wav.")
					g_soundChannelC.play(g_sndC)
					GPIO.output(g_Relay1, GPIO.HIGH)
					sleep(5)
					print("Ready for next cycle.")
				#torso only
				if (g_BUTTON_B == True):
					print("Button B Pressed.")
					while GPIO.input(LimitSwitch) == GPIO.LOW:
						sleep(0.1)  # wait for head to be fully down
					print("Torso up and play Scream1.wav.")
					g_soundChannelA.play(g_sndA)
					GPIO.output(g_Relay1, GPIO.LOW)  # Torso Up
					while g_BUTTON_B == True:
						sleep(0.01)  # wait for Button Debounce
					while g_BUTTON_B == False:
						sleep(0.01)  # wait for Button Press to toggle off.
					print("torso down")
					GPIO.output(g_Relay1, GPIO.HIGH)  # Torso Down
					while g_BUTTON_B == True:
						sleep(0.01)  # wait for Button Debounce
				#Head only
				if (g_BUTTON_C == True):
					print("Button C Pressed.")
					print("Head up and play Scream1.wav.")
					g_soundChannelA.play(g_sndA)
					GPIO.output(g_Relay2, GPIO.LOW)  # Head Up
					while g_BUTTON_C == True:
						sleep(0.01)  # wait for Button Debounce
					while g_BUTTON_C == False:
						sleep(0.01)  # wait for Button Press to toggle off.
					print("Head down")
					GPIO.output(g_Relay2, GPIO.HIGH)  # Head Down
					while g_BUTTON_C == True:
						sleep(0.01)  # wait for Button Debounce
				#Strobe
				if (g_BUTTON_D == True):
					print("Turn on strobe")
					GPIO.output(g_Relay4, GPIO.LOW)
					#spin until done
					while(g_BUTTON_D == True):
						sleep(0.01) # wait for button debounce
					print("Turn off strobe")
					GPIO.output(g_Relay4, GPIO.HIGH)
				#scream
				if (g_BUTTON_lt == True):
					print("scream sound")
					g_soundChannelA.play(g_sndA)
					while(g_BUTTON_lt == True):
						sleep(0.01) #wait for button debounce
					print("scream done, ready for next cycle")
				#laugh
				if (g_BUTTON_rt == True):
					print("laugh sound")
					g_soundChannelC.play(g_sndC)
					while(g_BUTTON_rt == True):
						sleep(0.01) #wait for button debounce
					print("laugh done, ready for next cycle")
				sleep(.01)
			except KeyboardInterrupt:
				halt()
				exit()
			sleep(0.1) #can do values like 0.1

class flashLed (threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self._stop_event = threading.Event()
		
	def stop(self):
		self._stop_event.set()

	def run(self):
		while True:
			GPIO.output(g_StatusLED, GPIO.HIGH) #LED ON
			sleep(1)
			GPIO.output(g_StatusLED, GPIO.LOW) #LED OFF
			sleep(1)

#safety stop, needs to do some things to safely terminate the program though
def halt():
	print("cleaning up threads")
	#TODO: make .stop function work
	thread1.stop()
	thread2.stop()
	thread3.stop()
	GPIO.cleanup()
	print("exiting")
	exit()


setup()
thread1 = myEvHandler(1)
thread2 = myOutputter(2)
thread3 = flashLed(3)

thread1.start()
thread2.start()
thread3.start()
thread1.join()
thread2.join()
thread3.join()
print("done")