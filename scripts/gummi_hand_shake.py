#!/usr/bin/env python

import rospy
import sys
import csv
import math

from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class HandShake:
    def __init__(self):
        self.person_in_front = False
        self.hand_shake_done = False
        self.touch_data_palm = 10000
        self.person_counter = 0
        self.touch_counter = 0

        rospy.Subscriber('~person', Bool, self.personCallback)
        rospy.Subscriber('~touch', UInt16, self.touchCallback)

        self.pwm1_pub = rospy.Publisher("~pwm1", UInt16,  queue_size=10)
        self.pwm2_pub = rospy.Publisher("~pwm2", UInt16,  queue_size=10)

        self.jointStatePub = rospy.Publisher("~joint_commands", JointState,  queue_size=10)
        self.handClosePub = rospy.Publisher("~hand_close", Float64,  queue_size=10)

        # the JointState object is created here to save time later
        self.JoinStateMsg = JointState()
        self.JoinStateMsg.name = ('shoulder_yaw', 'shoulder_roll', 'shoulder_pitch', 'upperarm_roll', 'elbow', 'forearm_roll', 'wrist_pitch')
        self.JoinStateMsg.position = [None]*len(self.JoinStateMsg.name)
        self.JoinStateMsg.velocity = [None]*len(self.JoinStateMsg.name)
        self.JoinStateMsg.effort = [None]*len(self.JoinStateMsg.name)

    def personCallback(self, msg):
        self.person_in_front = msg.data
        #print "Person: " + str(person_in_front)

    def touchCallback(self, msg):
        self.touch_data_palm = msg.data
        #print "Touch: " + str(touch_data_palm)

    def doUpdate(self):
        if self.person_in_front:
            if self.person_counter < 100:
                self.person_counter = self.person_counter + 3
        else:
            self.person_counter = self.person_counter - 6
        if self.person_counter < 0:
            self.hand_shake_done = False
            self.person_counter = 0
        print "Person counter: " + str(self.person_counter)

        if self.haveTouch():
            if self.touch_counter < 100:
                self.touch_counter = self.touch_counter + 15
        else:
            self.touch_counter = self.touch_counter - 30
        if self.touch_counter < 0:
            self.touch_counter = 0
        print "Touch counter: " + str(self.touch_counter)

    def havePersistentPerson(self):
        if self.person_counter >= 100:
            return True
        else:
            return False

    def haveTouch(self):
        if self.touch_data_palm < 1000: #TODO
            return True
        else:
            return False

    def havePersistentTouch(self):
        if self.touch_counter >= 100:
            return True
        else:
            return False

    def haveNewPerson(self):
        if self.hand_shake_done:
            return False
        else:
            return True

    def done(self):
        self.hand_shake_done = True

    def closeHand(self, value):
        print "Closing hand"
        msg = Float64()
        msg.data = value
        self.handClosePub.publish(msg)
        #self.closePwm1()
        #self.closePwm2()

    def closePwm1(self):
        msg = UInt16()
        msg.data = 10
        self.pwm1_pub.publish(msg)

    def closePwm2(self):
        msg = UInt16()
        msg.data = 10
        self.pwm2_pub.publish(msg)

    def publishJointState(self, joint_angles, cocontractions):
        self.JoinStateMsg.header.stamp = rospy.Time.now()
        for i in range(0, len(self.JoinStateMsg.name)):
            self.JoinStateMsg.position[i] =  joint_angles[i]
            self.JoinStateMsg.velocity[i] = 0.0
            self.JoinStateMsg.effort[i] = cocontractions[i]
        self.jointStatePub.publish(self.JoinStateMsg)

def main(args):

    pi = 3.1416
    rest = [0.0, -0.35, 0.25, 0.0030679615757712823, -0.7465373167710121, 0, -0.0051132692929521375]
    mid = [0.0, 0.05, 0.14317154020265985, -0.21475731030398976, -0.4755340442445488, 0, 0.0]
    final = [0.3170226961630325, 0.45, 0.25, -0.2684466378799872, -0.30, 0.3, 0.0]

    desired_pose = rest
    cocontraction = [0.75, 0.6, 0.6, 1.0, 0.0, 1.0, 0.2]

    width = 1.0 # 0.4
    frequency = 4.0

    rospy.init_node('gummi', anonymous=True)
    r = rospy.Rate(20)

    hand_shake = HandShake()

    do_shake_hand = False
    hand_is_closed = False
    time_counter = 1

    while not rospy.is_shutdown():

        hand_shake.doUpdate()

        if do_shake_hand:
            if time_counter < 20:
                print "Moving, first step"
                cocontraction = [0.75, 0.6, 0.85, 1.0, 0.6, 1.0, 0.5]
                desired_pose = mid
            else:
                if time_counter < 125:
                    print "Moving, second step"
                    cocontraction = [0.75, 0.8, 0.85, 1.0, 0.85, 1.0, 0.7]
                    desired_pose = final
                else:
                    print "Waiting..."

                    if not(hand_shake.havePersistentPerson()):
                        print "Person left before handshaking, return to resting pose."
                        cocontraction = [0.75, 0.6, 0.6, 1.0, 0.2, 1.0, 0.2]
                        desired_pose = rest
                        hand_shake.done()
                        do_shake_hand = False
                        time_counter = 0

                    else:

                        if time_counter < 500:
                            if not hand_is_closed:
                                if hand_shake.havePersistentTouch():
                                    print "Closing hand"
                                    hand_shake.closeHand(2)
                                    elbow_waiting = final[4] #elbow final
                                    hand_is_closed = True
                                    time_counter = 350
                            else:
                                print "Shaking hand"
                                elbow = elbow_waiting + width/2 * math.sin(frequency * time_counter/20.0)
                                desired_pose[4] = elbow
                        else:
                            if time_counter < 550:
                                print "Opening hand"
                                hand_shake.closeHand(0)
                                hand_is_closed = False
                            else:
                                if time_counter < 580:
                                    print "Moving back, second step"
                                    cocontraction = [0.75, 0.6, 0.85, 1.0, 0.6, 1.0, 0.5]
                                    desired_pose = mid
                                else:
                                    if time_counter < 600:
                                        print "Go to rest"
                                        cocontraction = [0.75, 0.6, 0.6, 1.0, 0.2, 1.0, 0.2]
                                        desired_pose = rest
                                    print "Done with hand shake"
                                    hand_shake.done()
                                    do_shake_hand = False
                                    time_counter = 0

            time_counter = time_counter + 1

        else:

           if hand_shake.haveNewPerson():
               if hand_shake.havePersistentPerson():
                   do_shake_hand = True
                   print "Do hand shake"

        hand_shake.publishJointState(desired_pose, cocontraction)
        r.sleep()

if __name__ == '__main__':
    main(sys.argv)
