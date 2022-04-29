import rospy
import serial
from std_msgs.msgs import Bool as Boolmsg


class Bridge():
    def __init__(self):
        # arduino_port = 'ttyUSB4'  # or something
        arduino_port = 'COM1'       # or something
        grab_topic = '/grab'

        self.port = serial.Serial(arduino_port, 9600)
        rospy.Subscriber(grab_topic, Boolmsg, self.grab_callback)

    def grab_callback(self, msg):
        if msg.data:
            self.port.write(b'c')
        else:
            self.port.write(b'o')

if __name__ == '__main__':
    rospy.init_node('SerialBridge', anonymous=True)
    b = Bridge()

    # Should open and close once
    msg = Boolmsg
    msg.data = False
    b.grab_callback(msg)
    rospy.sleep(2.0)
    msg.data = True
    b.grab_callback(msg)

    rospy.spin()