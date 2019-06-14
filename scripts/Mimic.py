#!/usr/bin/env python

# Short porgram to have RVIZ display mimic movements of real arm by subscribing to gummi/joint_state_publisher and publishing to rviz joint_state_commands
import rospy
import sys
import numpy as np

# from std_msgs.msg import Bool
# from std_msgs.msg import UInt16
# from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Decoder:
    def __init__(self):
        rospy.Subscriber('/gummi/joint_commands', JointState, self.MovingCallback)
        self.jointStatePub = rospy.Publisher("/rviz_sim/controller_joint_states", JointState,  queue_size=10)
        self.JointStateMsgIn = JointState()
        self.JointStateMsgOut = JointState()

    def MovingCallback(self, msg):
        self.JointStateMsgIn = msg
        position = self.JointStateMsgIn.position
        effort = self.JointStateMsgIn.effort

        newARRAY = position + effort

        self.JointStateMsgOut.header.stamp = rospy.Time.now()
        self.JointStateMsgOut.name = ('shoulder_yaw', 'shoulder_roll', 'shoulder_pitch', 'upperarm_roll', 'elbow', 'forearm_roll', 'wrist_pitch', 'gripper', 'i1f', 'i2f', 'i3f', 'i4f', 'i5f')
        self.JointStateMsgOut.position = newARRAY
        #print(self.JointStateMsgOut)
        self.jointStatePub.publish(self.JointStateMsgOut)


def main(args):

    rospy.init_node('decoder', anonymous=True)
    r = rospy.Rate(20)

    decoder = Decoder()

    while not rospy.is_shutdown():
        r.sleep()


if __name__ == '__main__':
    main(sys.argv)
