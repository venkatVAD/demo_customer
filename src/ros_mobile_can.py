#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import can
import sys
from numpy import interp
import time
import logging



# paramers for connecting with can , make sure you provided correctly
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
    x_linear=data.linear.x
    x_angular=data.angular.x
    rospy.loginfo("twist.linear: %f ; angular %f", data.linear.x, data.angular.x)
    a=abs(x_linear)
    if a==2:
        if x_linear==2:
           msg=[0x00,0X02,0X1,0x0, 0X0,0X0,0X0,0X0]
        elif x_linear==-2:
           msg=[0x00,0X02,0X2,0x0, 0X0,0X0,0X0,0X0]
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
        

    elif a>0 or a<0:
        b = interp(a, [0,1], [0,5])
    
    print (b)

    hex_v=int(b*256)
    hex_v=hex(hex_v)
    hex_v=hex_v[2:]
    k=hex_v
    if len(k)==4:
        d1=k[:2]
        d2=k[2:]
    elif len(k)==3:
        d1='0'+k[0]
        d2=k[1:]
    elif len(k)==2:
        d1='00'
        d2=k
    elif len(k)==1:
        d1='00'
        d2='0'+k
    else:
        d1='00'
        d2='00'
    d1 = int(d1, 16)
    d2 = int(d2, 16)

    if x_linear>0:
        d3=0X1
    elif x_linear<0:
        d3=0X2
    else: 
        d3=0X0
    print(d1,d2,d3)

    msg=[d2,d1,d3,0X0,0X0,0X0,0X0,0X0]
    print(msg)

    can_msg = can.Message(arbitration_id=0x188,
                              data=msg,
                              extended_id=False)
    bus.send(can_msg)
    
    
def listener():
    rospy.init_node('mobile_subscriber', anonymous=True)
    rospy.Subscriber("turtle1/cmd_vel", Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
