#!/usr/bin/python
import threading
from time import sleep
import evdev
from evdev import InputDevice, categorize, ecodes
import numpy as np
from enum import Enum

#for i2c
from smbus import SMBus
g_bus = SMBus(1) #using /dev/i2c-1
address = 10

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


g_btnState = 0
#128 is neutral state for analog sticks
g_lx = 0x80
g_ly = 0x80
g_rx = 0x80
g_ry = 0x80

class myEvHandler (threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID
		#self.device = evdev.InputDevice('/dev/input/event9')
		self.device = g_devices[0]

	def run(self):
		global g_btnState
		global g_absState
		global g_ry
		global g_ly
		global g_rx
		global g_lx
		print ("monitoring evdev")
		for event in self.device.read_loop():
			mask = np.uint16(0x1)
			if event.type == evdev.ecodes.EV_KEY and event.value == 1:
				#print("key down event with code of: ", event.code)
				if event.code == 304: #A
					g_btnState |= mask << BTN_A
				elif event.code == 305: #B
					g_btnState |= mask << BTN_B
				elif event.code == 307: #X
					g_btnState |= mask << BTN_X
				elif event.code == 308: #Y
					g_btnState |= mask << BTN_Y
				elif event.code == 310: #LT
					g_btnState |= mask << BTN_L1
				elif event.code == 311: #RT
					g_btnState |= mask << BTN_R1
				elif event.code == 317: #LT
					g_btnState |= mask << BTN_LEFT_STICK
				elif event.code == 318: #RT
					g_btnState |= mask << BTN_RIGHT_STICK
				elif event.code == 315: #Start
					g_btnState |= mask << BTN_START
				elif event.code == 314: #Select
					g_btnState |= mask << BTN_SELECT

			elif event.type == evdev.ecodes.EV_KEY and event.value == 0:
				#print("key up event with code of: ", event.code)
				if event.code == 304: #A
					g_btnState &= ~(mask << BTN_A)
				elif event.code == 305:  # B
					g_btnState &= ~(mask << BTN_B)
				elif event.code == 307:  # X
					g_btnState &= ~(mask << BTN_X)
				elif event.code == 308:  # Y
					g_btnState &= ~(mask << BTN_Y)
				elif event.code == 310: #LT
					g_btnState &= ~(mask << BTN_L1)
				elif event.code == 311: #RT
					g_btnState &= ~(mask << BTN_R1)
				elif event.code == 317: #Left stick
					g_btnState &= ~(mask << BTN_LEFT_STICK)
				elif event.code == 318: #Right stick
					g_btnState &= ~(mask << BTN_RIGHT_STICK)
				elif event.code == 315: #Start
					g_btnState &= ~(mask << BTN_START)
				elif event.code == 314: #Select
					g_btnState &= ~(mask << BTN_SELECT)


			elif event.type == evdev.ecodes.EV_ABS:
				if event.code == 2: #left analog trigger
					if event.value > 128:
						g_btnState |= mask << BTN_L2
					else:
						g_btnState &= ~(mask << BTN_L2)
				elif event.code == 5: #right analog trigger
					if event.value > 128:
						g_btnState |= mask << BTN_R2
					else:
						g_btnState &= ~(mask << BTN_R2)
				elif event.code == 16: #L/R D-pad (+ is R, - is L)
						if event.value == -1:
							g_btnState |= mask << BTN_D_LEFT
							g_btnState &= ~(mask << BTN_D_RIGHT)
						elif event.value == 1:
							g_btnState |= mask << BTN_D_RIGHT
							g_btnState &= ~(mask << BTN_D_LEFT)
						else:
							g_btnState &= ~(mask << BTN_D_LEFT)
							g_btnState &= ~(mask << BTN_D_RIGHT)
				elif event.code == 17:  # U/D D-pad (+ is down, - is up)
						if event.value == -1:
							g_btnState |= mask << BTN_D_UP
							g_btnState &= ~(mask << BTN_D_DOWN)
						elif event.value == 1:
							g_btnState |= mask << BTN_D_DOWN
							g_btnState &= ~(mask << BTN_D_UP)
						else:
							g_btnState &= ~(mask << BTN_D_UP)
							g_btnState &= ~(mask << BTN_D_DOWN)

				elif event.code == 1: #left analog stick Y
					g_ly = ((event.value+32768) /256) #compress +-32768 to 0-255 so we fit into a single byte
					#g_absState &= ~(0xFF << 0) #zero out our section
					#g_absState |= value << 0 #write to our section
				elif event.code == 0: #left analog stick X
					g_lx = ((event.value+32768) /256) #compress +-32768 to 0-255 so we fit into a single byte
					#g_absState &= ~(0xFF << 8) #zero out our section
					#g_absState |= value << 8 #write to our section
				elif event.code == 4: #right analog stick Y
					g_ry = ((event.value+32768) /256) #compress +-32768 to 0-255 so we fit into a single byte
					#g_absState &= ~(0xFF << 16) #zero out our section
					#g_absState |= value << 16 #write to our section
				elif event.code == 3: #right analog stick X
					g_rx = ((event.value+32768) /256) #compress +-32768 to 0-255 so we fit into a single byte
					#g_absState &= ~(0xFF << 24) #zero out our section
					#g_absState |= value << 24 #write to our section



#define button friendly names
BTN_SELECT = 0  #: The Select button
BTN_LEFT_STICK = 1  #: Left stick click button
BTN_RIGHT_STICK = 2  #: Right stick click button
BTN_START = 3  #: Start button
BTN_D_UP = 4  #: D-pad up
BTN_D_RIGHT = 5  #: D-pad right
BTN_D_DOWN = 6  #: D-pad down
BTN_D_LEFT = 7  #: D-pad left
BTN_L2 = 8  #: L2 lower shoulder trigger
BTN_R2 = 9  #: R2 lower shoulder trigger
BTN_L1 = 10  #: L1 upper shoulder trigger
BTN_R1 = 11  #: R1 upper shoulder trigger
BTN_Y = 12  #: Triangle
BTN_B = 13  #: Circle
BTN_A = 14  #: Cross
BTN_X = 15  #: Square
BTN_PS = 16  #: PS button



class myOutputter (threading.Thread):
	def __init__(self, threadID):
		threading.Thread.__init__(self)
		self.threadID = threadID

	def run(self):
		#global g_state
		global g_btnState
		global g_absState
		global g_bus
		while True:
			sleep(0.01) #tested as low as 0.01
			#print(hex(g_btnState))
			#print(hex(g_absState))
			#int_values = [x for x in (g_btnState.tobytes()+g_absState.tobytes())]
			#print(int_values)
			#write statement from old code
			#bus.write_i2c_block_data(address, 0xAA, [lowByte, highByte, int((rightx + 1) * 127), int((righty + 1) * 127), int((leftx + 1) * 127), int((lefty + 1) * 127)])
			#bus.write_i2c_block_data(address, 0xAA, int_values)
			lowbyte = g_btnState & 0x00FF
			highbyte = (g_btnState >> 8) & 0x00FF
			print([lowbyte, highbyte, int(g_rx), int(g_ry), int(g_lx), int(g_ly)])
			try:
				g_bus.write_i2c_block_data(address, 0xAA, [int(lowbyte), int(highbyte), int(g_rx), int(g_ry), int(g_lx), int(g_ly)] )
			except IOError:
				print('i2c bus error, attempting recovery...')
				sleep(0.1)
				g_bus = SMBus(1) #using /dev/i2c-1


thread1 = myEvHandler(1)
thread2 = myOutputter(2)

thread1.start()
thread2.start()
thread1.join()
thread2.join()
print("done")

####
#	THESE 2 LINES WORK IN PYTHON3
#   It should work, I think the crash happens when it closes, not when it is running
###
#from myevdev import evdev
#a = evdev.InputDevice('/dev/input/event1')

#
# import sys
# #sys.path.append('/home/pi/evdev/myevdev/evdev/')
# from myevdev import *
#
# devices = [evdev.InputDevice(path) for path in evdev.list_devices()]



#for path in evdev.list_devices():
#	print path
#	evdev.InputDevice(path)
#for device in devices:
#	print(device.path, device.name, device.phys)
#	if "Logitech Gamepad F710" in device.name:
#		print("gamepad found")
#		gamepad = device



#print("sadf")
#print(gamepad)
