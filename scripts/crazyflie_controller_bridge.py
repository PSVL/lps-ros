#!/usr/bin/env python
# Make the Crazyflie internal position controller accessible the same way as
# the Crazyflie ROS package demo controller

# TODO: Read PID parameters and set them in the Crazyflie parameters

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Empty
from crazyflie_driver.srv import UpdateParams
import time

# Takeoff and landing speed in ms^-1
TAKEOFF_SPEED = 1.0
LANDING_SPEED = 0.5
UPDATE_RATE = 30


class ControllerBridge:
    def __init__(self):
        self.target_setpoint = Twist()
        self.landed_setpoint = Twist()
        self.curr_pos = PoseStamped()
        self.flying = False
        self.landing = False
        self.transient_height = 0
        self.curr_z_vel = 0
        self.landing_ticks = 0

        self.assisted_takeoff = rospy.get_param("~assisted_takeoff", True)
        self.assisted_landing = rospy.get_param("~assisted_landing", True)
        self.reconnect_on_takeoff = rospy.get_param("~reconnect_on_takeoff",
                                                    False)

        self.goal_sub = rospy.Subscriber("goal", PoseStamped, self.new_goal)
        self.pose_sub  = rospy.Subscriber("pose", PoseStamped, self.pose_cb)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # Services
        self.ts = rospy.Service('takeoff', Empty, self.takeoff)
        self.ls = rospy.Service('land', Empty, self.land)
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)
        if self.reconnect_on_takeoff:
            self._reconnect = rospy.ServiceProxy('reconnect', Empty)

        # Send position setpoints 30 times per seconds
        self.timer = rospy.Timer(rospy.Duration(1.0/UPDATE_RATE),
                                 self.send_setpoint)

    def run(self):
        rospy.spin()

    def pose_cb(self, data):
        self.curr_z_vel = (data.pose.position.z - self.curr_pos.pose.position.z) * UPDATE_RATE
        self.curr_pos = data

    def send_setpoint(self, event):
        if self.flying:
            sp = Twist()
            sp.linear = self.target_setpoint.linear
            if self.landing:
                sp.linear.z = -1
                print(self.landing_ticks, self.curr_z_vel)
                if abs(self.curr_z_vel) < 0.4:
                    self.landing_ticks += 1
                    if self.landing_ticks >= 0.5*UPDATE_RATE:
                        self.landing_ticks = 0
                        rospy.set_param("flightmode/posSet", 0)
                        self._update_params(["flightmode/posSet"])
                        self.landing = False
                        self.flying = False
                else:
                    self.landing_ticks = 0
            self.cmd_vel_pub.publish(sp)
        else:
            self.cmd_vel_pub.publish(self.landed_setpoint)

    def new_goal(self, goal):
        self.target_setpoint.linear.x = goal.pose.position.x
        self.target_setpoint.linear.y = goal.pose.position.y
        self.target_setpoint.linear.z = goal.pose.position.z

    def takeoff(self, req):

        if self.reconnect_on_takeoff and not self.flying:
            self._reconnect()
            time.sleep(2)

        rospy.set_param("flightmode/posSet", 1)
        self._update_params(["flightmode/posSet"])
        if self.landing or (not self.flying) and self.assisted_takeoff:
            self.flying = True
            self.landing = False
        elif not self.assisted_takeoff:
            self.flying = True
            self.landing = False
        return ()

    def land(self, req):
        if self.flying and self.assisted_landing and not self.landing:
            self.landing = True
        elif not self.assisted_landing:
            self.flying = False
            self.landing = False

        return ()

if __name__ == "__main__":
    rospy.init_node("controller_bridge")
    cb = ControllerBridge()
    cb.run()
