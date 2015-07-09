#!/usr/bin/env python
import os
import sys
import struct
import time
import termios
import tty

import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

import roscopter.msg



mavlink_dir = os.path.realpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '..', 'mavlink'))
sys.path.insert(0, mavlink_dir)

pymavlink_dir = os.path.join(mavlink_dir, 'pymavlink')
sys.path.insert(0, pymavlink_dir)


from optparse import OptionParser
parser = OptionParser("roscopter.py [options]")

parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
parser.add_option("--device", dest="device", default="/dev/ttyUSB1", help="serial device")
parser.add_option("--rate", dest="rate", default=10, type='int', help="requested stream rate")
parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
parser.add_option("--enable-control",dest="enable_control", default=False, help="Enable listning to control messages")

(opts, args) = parser.parse_args()

import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

if opts.device is None:
    print("You must specify a serial device")
    sys.exit(1)

def wait_heartbeat(m):
    '''wait for a heartbeat so we know the target system IDs'''
    print("Waiting for APM heartbeat")
    m.wait_heartbeat()
    print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))


def send_rc(data):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        data.channel[0],
        data.channel[1],
        data.channel[2],
        data.channel[3],
        data.channel[4],
        data.channel[5],
        data.channel[6],
        data.channel[7])
    print "sending rc: %s" % data


def set_arm(req):
    master.arducopter_arm()
    return []

def set_disarm(req):
    master.arducopter_disarm()
    return []

def param_set(req):  
    master.param_set_send("RC1_DZ",31)
    return []


#if opts.enable_control:
#    rospy.Subscriber("send_rc", roscopter.msg.RC , send_rc)

#define service callbacks
arm_service = rospy.Service('arm', Empty, set_arm)
disarm_service = rospy.Service('disarm', Empty, param_set)


#state
gps_msg = NavSatFix()



def mainloop():
    rospy.init_node('roscopter')

#    tstart = time.time()    
#    while time.time() - tstart < 3:
#       master.param_fetch_one("UAV_DEL_X")
#       ack = master.recv_match(type='PARAM_VALUE', blocking=False)
#       if ack == None:
#          time.sleep(0.1)
#          continue
#       else:    
#          del_x=ack.param_value
#          break
    del_x=0
    del_y=0
    del_z=0
    del_yaw=40
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        fd = sys.stdin.fileno()  
        old_settings = termios.tcgetattr(fd)  
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO  
        try :  
            tty.setraw( fd )  
            ch = sys.stdin.read( 1 )  
        finally :  
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if ch == 'w':      # forward
            del_x=del_x+10
            print'x %d'%+del_x
	    master.param_set_send("UAV_DEL_X",del_x)
	elif ch == 's':  
            del_x=del_x-10
            print'x %d'%-del_x
	    master.param_set_send("UAV_DEL_X",del_x)   
	elif ch == 'a':  
            del_y=del_y-10
            print'y %d'%del_y
	    master.param_set_send("UAV_DEL_Y",del_y)  
	elif ch == 'd':  
            del_y=del_y+10
            print'y %d'%del_y
	    master.param_set_send("UAV_DEL_Y",del_y)  
	elif ch == 'i':  
            del_z=del_z+20
            print'z %d'%del_z
	    master.param_set_send("UAV_DEL_Z",del_z)  
	elif ch == 'k':  
            del_z=del_z-20
            print'z %d'%del_z
	    master.param_set_send("UAV_DEL_Z",del_z)  
	elif ch == 'j':  
            del_yaw=del_yaw-20
            print del_yaw
	    master.param_set_send("UAV_DEL_YAW",del_yaw) 
        elif ch == 'l':  
            del_yaw=del_yaw+20
            print del_yaw
	    master.param_set_send("UAV_DEL_YAW",del_yaw)
        elif ch=='c': 
            break


wait_heartbeat(master)

print("Sending all stream request for rate %u" % opts.rate)
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    opts.rate,
    1)


if __name__ == '__main__':
    try:
        mainloop()
    except rospy.ROSInterruptException: pass
