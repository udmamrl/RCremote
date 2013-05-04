#!/usr/bin/env python
#########################################################################
# IGVC RC Remote driver for AMRL lab
#
# Copyright (c) 2013, Cheng-Lung Lee, Yu-Ting Wu, Hong-Yi Zhang
# University of Detroit Mercy. Advanced Mobile Robotic Lab.
# All rights reserved.
#########################################################################
# Changelog
# 2013.01.04 initial release.
# 2013.01.08 Add LCD function
#########################################################################
# Software License Agreement (BSD License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
######################################################################
# // RC dataformat for AGV
# 1.Header for data :Byte 1     : $
# 2.ESTOP STATUS    :Byte 2     : Normal=0 , ESTOP=F
# 3.PC/RC STATUS    :Byte 3     : RC=0, PC=F
# 4.SPEED           :Byte 4-7   : Unit mMPH  ,range -1000mMPH-2500mMPH (-1MPH~2.5MPH)
# 5.BIAS            :Byte 8-11  : Unit mMPH  ,range -2500mMPH-2500mMPH (-2.5MPH~2.5MPH)
# 6.Battery         :Byte 12-15 : Unit mV , R/C Battry/System voltage
# 7.Checksum        :Byte 16,17 : 8bit checksum
# 8.LF              :Byte 18    : 0x10
# C printf format "$%2X%4X%4X%4X%2X\n"
# Example: "$0F09C4F63C106879\n"
# $: Header
# 0F: No ESTOP, PC mode
# 09C4 : Speed:  2500mMPH 
# F63C : Bias:  -2500mPH
# 1068 : Battry: 4200mV
# 79   :CheckSum = ~(0x0F+0x09+0xC4+0xF6+0x3C+0x10+0x68)
# \n : (change to \n for readline() function , was \r)
###########################################################3
#  Robot -> RC Remote , 2 byte only
# $E : Manual E-STOP ON
# $N : Normal, no e-stop on robot
# RX format PowerBox -> RC , 
# $E : Manual E-STOP ON
# $N : Normal
# New in the code add LCD data we will send LCD data to remote for display
# $N00StringToDisplay\n
# $N is old formate,
# 00 is the Ascii hex string indicate the startup address the the string in LCD_buffer
# StringToDisplay is the string data end with \n
# all the string will store in 84 byte LCD buffer & update every second

# TODO subscribe to husky safety status and read E-STOP status
# TODO subscribe to Odometry read real speed/ turn rate from robot and display on LCD


import roslib; roslib.load_manifest('RCremote')
import rospy
import numpy as np    
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from clearpath_base.msg import SafetyStatus
from clearpath_base.msg import SystemStatus
#rosmsg show clearpath_base/SafetyStatus 
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#uint16 flags
#bool estop

#rosmsg show clearpath_base/SystemStatus 
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#uint32 uptime
#float64[] voltages
#float64[] currents
#float64[] temperatures


from nav_msgs.msg import Odometry
import serial, string, math, time, calendar




#Check the RC sentence checksum. Return True if passes and False if failed
def check_checksum(remote_sentence):
        if  ('$' in remote_sentence) and (len(remote_sentence)==18):
                checksum=0
                for i in range(1,17,2):   
                # print remote_sentence[i:i+2]          
                        checksum=checksum+int(remote_sentence[i:i+2],16) 
   
                checksum=np.uint8(checksum)   # cast to 8bit                        
                return checksum == 255        # the sum should be 0xFF
        else:
                return False
def String_replace(str_in1,str_in2,start):
    return str_in1[0: start]+str_in2+str_in1[(start+len(str_in2)):len(str_in1)]
def Autonomous_cmd_vel_cb(Twist_in):
    global PCRC
    global VelocityCommandPublisher
    global Autonomous_cmd_vel_DataTimeSec
    if (PCRC):
        VelocityCommandPublisher.publish(Twist_in)
        Autonomous_cmd_vel_DataTimeSec=rospy.get_time()
#        rospy.info("Got Autonomous_cmd_vel %f" % Twist_in.linear.x)
#        print Twist_in.linear.x
#        print Twist_in.angular.z
def HandleHuskyOdom(odom):
    global HuskySpeed
    global HuskyTrunRate
    HuskySpeed=odom.twist.twist.linear.x
    HuskyTrunRate=odom.twist.twist.angular.z

# TODO add battery handle
def HandleHuskySystemStatus(HuskySystemStatus):
    global HuskyBatteryV
    HuskyBatteryV=SystemStatus.voltages(1) # main

def HandleHuskysafety(Huskysafety):
    global HuskyMEstop
    HuskyMEstop=Huskysafety.estop

def RCremoteshutdownhook():
    global D_RCremote
    print "RCremote shutdown time!"
    D_RCremote.flush() # flush data out
    rospy.loginfo('Closing D_RCremote Serial port')
    result=D_RCremote.close() #Close D_Compass serial port
    print  result
    # rospy.on_shutdown(RCremoteshutdownhook)
    
if __name__ == '__main__':
    global PCRC
    global VelocityCommandPublisher
    global Autonomous_cmd_vel_DataTimeSec
    global D_RCremote
    global HuskySpeed
    global HuskyTrunRate
    global HuskyMEstop
    global Odom
    global HuskySafty
    global HuskyBatteryV
    
    RC_DataTimeSec=0
    rospy.init_node('RCremote')
    VelocityCommandPublisher = rospy.Publisher("/husky/cmd_vel", Twist)
    rospy.Subscriber("/RCremote/plan_cmd_vel", Twist, Autonomous_cmd_vel_cb)
    #Init D_RCremote port
    D_RCremote_port = rospy.get_param('~port','/dev/ttyUSB0')
    D_RCremote_rate = rospy.get_param('~baud',57600)
    D_RCremote_WheelBase = rospy.get_param('~WheelBase',0.556)
    D_RCremote_SpeedScale = rospy.get_param('~SpeedScale',1.0)
    D_RCremote_TurnRateScale = rospy.get_param('~TurnRateScale',1.0)
    WatchDogTimeOut = rospy.get_param('~WatchDogTimeOut',1.0)
    mMPH2ms  =0.001 * 0.44704 *D_RCremote_SpeedScale
    bias2rads=0.001 * 0.44704 *D_RCremote_TurnRateScale /D_RCremote_WheelBase
    RCmV=0
    Autonomous_cmd_vel_DataTimeSec=rospy.get_time()
    #global PCRC=0
    # TODO make it global
    HuskyBatteryV=0 #V
    HuskyMESTOP=False
    HuskySpeed=0
    HuskyTrunRate=0
    velocityCommand = Twist()
    # LCD String 
    LCD_size=14*6 #6 lines 14 char
    LCD_str= "".join([" " for x in range(LCD_size)]) # make 84byte string
    LCD_lineindex=0 # range 0~5 , bufferindex=LCD_lineindex*14

#                           #12345678901234
#    LCD_str=String_replace(LCD_str,'Husky A200 ',0)
#                           #12345678901234
#    LCD_str=String_replace(LCD_str,'R/C Speed:    ' ,1*14)
#    LCD_str=String_replace(LCD_str,'Speed:%5.2fm/s' % velocityCommand.linear.x  ,2*14)
#    LCD_str=String_replace(LCD_str,'Turn :%5.2fr/s' % velocityCommand.angular.z ,3*14)
#    LCD_str=String_replace(LCD_str,'Robot:%5.2fV  ' % HuskyBatteryV ,4*14)
#    LCD_str=String_replace(LCD_str,'  R/C:%5.2fV  ' % (RCmV/1000.)   ,5*14)
    #Odom=Odometry()
    #HuskySafty=SafetyStatus()
    #TODO battyer HuskySys_status=SafetyStatus() # for battery
    rospy.Subscriber('/odom', Odometry, HandleHuskyOdom)
    rospy.Subscriber('/husky/data/safety_status', SafetyStatus, HandleHuskysafety)
    rospy.Subscriber('/husky/data/power_status', SystemStatus, HandleHuskySystemStatus)
    rospy.on_shutdown(RCremoteshutdownhook)
    try:
        #Setup RCremote serial port
        D_RCremote = serial.Serial(port=D_RCremote_port, baudrate=D_RCremote_rate, timeout=0.5)

        while not rospy.is_shutdown(): 
           # D_RCremote.write('$0F09C4F63C106879\n') # for debug RC
           # D_RCremote.write('$0009C4F63C1F6879\n') # for debug PC
            data = D_RCremote.readline()
            #rospy.loginfo("Received a sentence: %s" % data)

            # if length is shorter then 18 , send stop [either bad remote or no signal]
            if (len(data)<18):
                #VelocityCommandPublisher.publish(Twist()) # this will publish all 0 data
                rospy.logerr("Received a sentence but length is not 18. Sentence was: %s" % data)
                if (rospy.get_time()-RC_DataTimeSec>WatchDogTimeOut):
                    VelocityCommandPublisher.publish(Twist())
                continue

            if not check_checksum(data):
                rospy.logerr("Received a sentence with an invalid checksum. Sentence was: %s" % data)
                #VelocityCommandPublisher.publish(Twist()) # this will publish all 0 data
                if (rospy.get_time()-RC_DataTimeSec>WatchDogTimeOut):
                    VelocityCommandPublisher.publish(Twist())
                continue

            RC_DataTimeSec = rospy.get_rostime()

            try:
                    if '$' in data:
                        estop=int(data[1],16)
                        PCRC =int(data[2],16)
                        speed_mMPH=np.int16(int(data[3:7],16))
                        bias_mMPH =np.int16(int(data[7:11],16))
                        RCmV      =np.int16(int(data[11:15],16))
                        #TODO add switch here if RC mode send speed/bias from remote data , 
                        #     PC mode relay data from VFH/navigation cmd_vel to robot
                        if (PCRC): # if PC mode
                        # add watchdog , if no data from Autonomous_cmd_vel comming in stop the robor
                            if (rospy.get_time()-Autonomous_cmd_vel_DataTimeSec>WatchDogTimeOut):
                                VelocityCommandPublisher.publish(Twist())
                        else: # RC mode
                            velocityCommand.linear.x = speed_mMPH * mMPH2ms
                            velocityCommand.angular.z = bias_mMPH *-1.* bias2rads
                            VelocityCommandPublisher.publish(velocityCommand)
                        # have to relay husky E-stop status to robot, 
                        #      so we can know m-e-stop status on remote
                        if (HuskyMESTOP): # Manual ESTOP must update from husky safety status message
                                D_RCremote.write('$E')  # right now we only send Normal, must send $E if robot is estoped
                        else:
                                D_RCremote.write('$N')  # right now we only send Normal, must send $E if robot is estoped
                        # now we update LCD data
                        LCD_lineindex+=1
                        if (LCD_lineindex>=6):
                                LCD_lineindex=0
                                                               #12345678901234
#                        LCD_str=String_replace(LCD_str,'Husky A200 ',0)
#                                                               #12345678901234
#                        LCD_str=String_replace(LCD_str,'R/C Speed:    ' ,1*14)
#                        LCD_str=String_replace(LCD_str,'Speed:%5.2fm/s' % HuskySpeed  ,2*14)
#                        LCD_str=String_replace(LCD_str,'Turn :%5.2fr/s' % HuskyTrunRate ,3*14)
#                        LCD_str=String_replace(LCD_str,'Robot:%5.2fV  ' % HuskyBatteryV ,4*14)
#                        LCD_str=String_replace(LCD_str,'  R/C:%5.2fV  ' % (RCmV/1000.)   ,5*14)
#
#                        
#                        D_RCremote.write(('%i' % (LCD_lineindex) )+LCD_str[LCD_lineindex*14:LCD_lineindex*14+14]+'\n') # send 14 byte each time


                    else:
                        #VelocityCommandPublisher.publish(Twist()) # this will publish all 0 data
                        rospy.logerr("Received a sentence but no $. Sentence was: %s" % data)

            except ValueError as e:
                #VelocityCommandPublisher.publish(Twist()) # this will publish all 0 data
                rospy.logwarn("Value error, likely due to missing fields in the header")
    except rospy.ROSInterruptException:

        D_RCremote.close() #Close D_Compass serial port
