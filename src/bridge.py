import rospy
import serial
import time
from std_msgs.msg import Bool as Boolmsg

# class Boolmsg():
#     data = True
class Bridge():
    def __init__(self):
        # arduino_port = 'ttyUSB4'  # or something
        arduino_port = 'COM6'       # or something
        grab_topic = '/grab'

        self.port = serial.Serial(arduino_port, 9600)
        # rospy.Subscriber(grab_topic, Boolmsg, self.grab_callback)

    def grab_callback(self, msg):
        print("command:" + str(msg.data))
        if msg.data:
            self.port.write(b'c')
        else:
            self.port.write(b'o')

if __name__ == '__main__':
    rospy.init_node('SerialBridge', anonymous=True)
    b = Bridge()

    # Should open and close once

    time.sleep(3)
    msg = Boolmsg()
    msg.data = False
    b.grab_callback(msg)
    time.sleep(3)

    msg.data = True
    b.grab_callback(msg)
    time.sleep(5.0)
    rospy.spin()