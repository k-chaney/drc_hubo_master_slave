# Copyright (c) 2013, Daniel Lofaro, Kenneth Chaney 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Based on: https://github.com/thedancomplex/pydynamixel
# Based on: https://github.com/hubo/hubo-ach
# */


import os
import dynamixel
import time
import random
import subprocess
import optparse
import yaml
import numpy as np

from multiprocessing import Process, Lock
import sys
import getch

# Hubo-ach stuff
import hubo_ach as ha
import ach
from ctypes import *

userExit = False
upperBody = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]
rightArm = [0,1,2,3,4,5,6,7]
leftArm  = [8,9,10,11,12,13,14,15]

def rad2dyn(rad):
    return np.int(np.floor( (rad + np.pi)/(2.0 * np.pi) * 1024 ))

def dyn2rad(en):
    return en / 1024.0 * 2.0 * np.pi - np.pi

def mapMiniToFull(n):
    if ( n == 0 ):
       return ha.RF1
    elif ( n == 1 ):
       return ha.RWR
    elif ( n == 2 ):
       return ha.RWP
    elif ( n == 3 ):
       return ha.RWY
    elif ( n == 4 ):
       return ha.REB
    elif ( n == 5 ):
       return ha.RSY
    elif ( n == 6 ):
       return ha.RSR
    elif ( n == 7 ):
       return ha.RSP
    elif ( n == 8 ):
       return ha.LSP
    elif ( n == 9 ):
       return ha.LSR
    elif ( n == 10):
       return ha.LSY
    elif ( n == 11):
       return ha.LEB
    elif ( n == 12):
       return ha.LWY
    elif ( n == 13):
       return ha.LWP
    elif ( n == 14):
       return ha.LWR
    elif ( n == 15):
       return ha.LF1
    elif ( n == 16):
       return ha.WST
    else:
       print n
       return 37

def getJointDirection(n):
    n = mapMiniToFull(n)
    if ( n == ha.LWY or n == ha.LWP or n==ha.LSY or n==ha.RSP or n == ha.RWY or n == ha.RSY ):
       return -1
    else:
       return 1

def keyPresses(actuators,lock):
  print "Keyboard checking started"
  while True:
     time.sleep(0.01)
     ch = getch.getch()
     if (ch == ' '):
         lock.acquire()
         toggleTorques(actuators,upperBody)
         lock.release()
     elif ( ch == 'f' ):
         lock.acquire()
         toggleTorques(actuators,leftArm)
         lock.release()
     elif ( ch == 'j' ):
         lock.acquire()
         toggleTorques(actuators,rightArm)
         lock.release()
     elif ( ch == 'b' ):
         lock.acquire()
         toggleTorques(actuators,[16])
         lock.release()
     elif (ch == 'q'):
         userExit=True
         sys.exit("User exited program")

def toggleTorques(actuators, actList):
    for actuator in actuators:
      if (actuator.id in actList):
        time.sleep(0.01)
        if actuator.torque_enable==True:
            actuator.torque_enable=False
        else:
            actuator.torque_enable=True
            actuator.torque_limit=800
            actuator.max_torque=800

def main(settings):
    # Open Hubo-Ach feed-forward and feed-back (reference and state) channels
    s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
    e = ach.Channel(ha.HUBO_CHAN_ENC_NAME)
    r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
    #s.flush()
    #r.flush()

    # feed-forward will now be refered to as "state"
    state = ha.HUBO_STATE()

    # encoder channel will be refered to as "encoder"
    encoder = ha.HUBO_ENC()

    # feed-back will now be refered to as "ref"
    ref = ha.HUBO_REF()

    # Get the current feed-forward (state) 
    [statuss, framesizes] = s.get(state, wait=False, last=True)
    [statuss, framesizes] = e.get(encoder, wait=False, last=True)

    portName = settings['port']
    baudRate = settings['baudRate']
    highestServoId = settings['highestServoId']

    # Establish a serial connection to the dynamixel network.
    # This usually requires a USB2Dynamixel
    serial = dynamixel.SerialStream(port=portName, baudrate=baudRate, timeout=1)
    net = dynamixel.DynamixelNetwork(serial)
    
    myActuators = []
    # Ping the range of servos that are attached
    print "Scanning for Dynamixels..."
    net.scan(0, highestServoId)
    
    for dyn in net.get_dynamixels():
        print dyn.id
        myActuators.append(net[dyn.id])
    
    if not myActuators:
      print 'No Dynamixels Found!'
      sys.exit(0)
    else:
      print "...Done"
 
    for actuator in myActuators:
	print actuator.id 
        actuator.moving_speed = 50
        actuator.synchronized = True
        actuator.torque_enable =  False
        actuator.torque_limit = 30 
        actuator.max_torque = 10
        time.sleep(0.05)

    print myActuators
    actuatorsLock = Lock()
    p = Process(target=keyPresses, args=(myActuators, actuatorsLock))
    p.start()
    print "Master Slave Server Running"

    while True:
        if userExit==False:
            sys.exit(0)
	[statuss, framesizes] = s.get(state, wait=False, last=True)
        actuatorsLock.acquire()
        for actuator in myActuators:
            actuator.read_all()
            time.sleep(0.005)
            ref.ref[mapMiniToFull(actuator.id)]=getJointDirection(actuator.id) * dyn2rad(actuator.current_position)
        r.put(ref)
        actuatorsLock.release()
        time.sleep(0.02)

def validateInput(userInput, rangeMin, rangeMax):
    '''
    Returns valid user input or None
    '''
    try:
        inTest = int(userInput)
        if inTest < rangeMin or inTest > rangeMax:
            print "ERROR: Value out of range [" + str(rangeMin) + '-' + str(rangeMax) + "]"
            return None
    except ValueError:
        print("ERROR: Please enter an integer")
        return None
    
    return inTest

if __name__ == '__main__':
    
    parser = optparse.OptionParser()
    parser.add_option("-c", "--clean",
                      action="store_true", dest="clean", default=False,
                      help="Ignore the settings.yaml file if it exists and \
                      prompt for new settings.")
    
    (options, args) = parser.parse_args()
    
    settings = {}
    if os.name == "posix":
        portPrompt = "Which port corresponds to your USB2Dynamixel? \n"
        # Get a list of ports that mention USB
        try:
            possiblePorts = subprocess.check_output('ls /dev/ | grep -i usb',
                                                    shell=True).split()
            possiblePorts = ['/dev/' + port for port in possiblePorts]
        except subprocess.CalledProcessError:
            sys.exit("USB2Dynamixel not found. Please connect one.")
            
        counter = 1
        portCount = len(possiblePorts)
        for port in possiblePorts:
            portPrompt += "\t" + str(counter) + " - " + port + "\n"
            counter += 1
        portPrompt += "Enter Choice: "
        portChoice = None
        while not portChoice:                
            portTest = raw_input(portPrompt)
            portTest = validateInput(portTest, 0, portCount)
            if portTest:
                portChoice = possiblePorts[portTest - 1]

    else:
        portPrompt = "Please enter the port name to which the USB2Dynamixel is connected: "
        portChoice = raw_input(portPrompt)
    
    settings['port'] = portChoice
        
    # Baud rate
    baudRate = None
    while not baudRate:
        brTest = raw_input("Enter baud rate [Default: 1000000 bps]:")
        if not brTest:
            baudRate = 1000000
        else:
            baudRate = validateInput(brTest, 9600, 1000000)
                
    settings['baudRate'] = baudRate
        
    # Servo ID
    highestServoId = None
    while not highestServoId:
       hsiTest = raw_input("Please enter the highest ID of the connected servos: ")
       highestServoId = validateInput(hsiTest, 1, 255)
        
    settings['highestServoId'] = highestServoId
        
    main(settings)
