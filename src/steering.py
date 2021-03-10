#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import can
import sys
import time
import logging




try:
    baud_rate=rospy.get_param('baudrate')
    channel=rospy.get_param('channel_can')
    bus_type=rospy.get_param('bustype')
    


    if channel=='vcan0' or channel=='can0':
        pass
    else:
        print("YOU ENTERED "+str(channel))
        print("GIVEN INVALID CAN CHANNEL NAME ")
        sys.exit()
    
    # bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=1000000)
    bus = can.interface.Bus(bustype=bus_type, channel=channel, bitrate=baud_rate)

    print("---------------------------------------------------------------")

    print("         CONNECTION TO CAN DEVICE-- SUCCESS")
    print("---------------------------------------------------------------")
    
except Exception as e:
    print("---------------------------------------------------------------")
    print("                CONNECTION TO CAN DEVICE-- FAILED")
    print("     			PLEASE MAKE CAN UP              	 ")
    print("                                OR                                ")
    print("            PLEASE DO CHECK BAUDRATE AND CHANNEL NAME IN LAUNCH FILE")
    print(' ERROR is --',e)
    print("----------------------------------------------------------------")
    sys.exit()



def callback(data):
    x_linear=data.linear.z
    x_angular=data.angular.z
    rospy.loginfo("twist.linear: %f ; angular %f", data.linear.z, data.angular.z)
    a=abs(x_angular)
    if a==2:
        if x_angular==2:
           msg=[0x1,0X00,0X64,0x0, 0X0,0X0,0X0,0X0]
        elif x_angular==-2:
           msg=[0x1,0X00,0X9C,0x0, 0X0,0X0,0X0,0X0]
        else:
            msg=[0x00,0X00,0X0,0x0, 0X0,0X0,0X0,0X0]

    
        print('MSG '+str(msg))
        can_msg = can.Message(arbitration_id=0x188,
                              data=msg,
                              extended_id=False)
    
        bus.send(can_msg)
        return 0
    if a==0:
        b=0
        

    elif a>0:
        b = x_angular * 100
    
    elif a<0:
        b = x_angular * -100
    
    print (b)

    hex_v=int(b)
    hex_v=hex(hex_v & (2**8-1))
    d2 = hex_v[2:]
    d2 = int(d2 ,16)
    if x_angular>0:
        d3=0X1
    elif x_angular<0:
        d3=0X1
    else: 
        d3=0X0
    print(d2,d3)

    msg=[d3,0x0,d2,0X0,0X0,0X0,0X0,0X0]
    print(msg)

    can_msg = can.Message(arbitration_id=0x298,
                              data=msg,
                              extended_id=False)
    bus.send(can_msg)
    
    
def listener():
    rospy.init_node('mobile_subscriber', anonymous=True)
    rospy.Subscriber("turtle1/cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
