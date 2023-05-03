#!/usr/bin/env python3
import rospy
import can
from can.interfaces import robotell
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

L = 0.3
R = 0.05


def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value
class can_testing:
    def __init__(self):
        #self.bus = robotell.robotellBus(ttyBaudrate=115200, channel='/dev/ttyACM0', bitrate=1000000)
        self.bus = can.interface.Bus(channel='can0', interface='socketcan',bitrate=1000000)
        # ros
        self.sub_cmd  = rospy.Subscriber('cmd_vel', Twist, self.subCmdCB, queue_size=10)
        self.timer_can = rospy.Timer(rospy.Duration(1/50), self.timerCanCB)    # timer 100Hz
        self.TxData = [128,0,128,0,128,0,128,0]
        self.TyData = [128,0,128,0,128,0,128,0]
    def subCmdCB(self, msg):
        vx = msg.linear.x
        omega = msg.angular.z
        v1 = -1 *(vx - omega*L) /R
        v2 = -1 *(vx + omega*L) /R
        V1 = int(map(v1,-100 ,100,0 ,65535))
        V2 = int(map(v2,-100 ,100 ,0 ,65535))
####################################JLJL############
        self.TxData[0] = ((V1 & 0xFF00) >> 8)
        self.TxData[1] = (V1 & 0x00FF)
        self.TxData[2] = ((V2 & 0xFF00) >> 8)
        self.TxData[3] = (V2 & 0x00FF)
        self.TxData[4] = ((0 & 0xFF00) >> 8)
        self.TxData[5] = (0 & 0x00FF)
        self.TxData[6] = ((0 & 0xFF00) >> 8)
        self.TxData[7] = (0 & 0x00FF)
        
      
    def timerCanCB(self, event):
        message = can.Message(arbitration_id=0X111, is_extended_id=False,
                      data= self.TxData)
        self.bus.send(message,0.1)
        
if __name__ == '__main__':
    try:
        rospy.init_node('can_node', anonymous=True)
        can_testing()
        rospy.spin()
    except KeyboardInterrupt:
        pass
