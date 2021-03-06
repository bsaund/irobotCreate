#!/usr/bin/env python

from irobot_create.robots import create2
from irobot_create.control import cartesian
import numpy as np
from irobot_create.control.pid import RampController
import threading
import time
import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions


class Bradbot(create2.Create2):
    def __init__(self, serial=None, remote=False):
        super(Bradbot, self).__init__(serial, remote=remote)
        self.is_running = True

        self.irobot_data = None
        self.read_irobot_data()
        self.pos = cartesian.Cartesian(self.irobot_data.left_encoder_counts, self.irobot_data.right_encoder_counts)

        self.x_target = None
        self.y_target = None
        self.err_limit = None
        self.vel = None
        self.moving = False
        self.left_controller = RampController(max_accel=2000)
        self.right_controller = RampController(max_accel=2000)
        self.prev_sent_velocities = [0, 0]  # [left, right]
        self.send_vel_thread = threading.Thread(target=self.send_updated_velocity_thread)
        self.send_vel_thread.start()
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        self.control_thread = None
        self.time_in_stasis = 0.0
        self.time_of_last_sensor_read = time.time()

    def shutdown(self):
        self.is_running = False

    def start_control_loop_thread(self, rate=10):
        def async_control_loop():
            while self.is_running and not rospy.is_shutdown():
                self.control_loop()
                rospy.Rate(rate).sleep()

        self.control_thread = threading.Thread(target=async_control_loop)
        self.control_thread.start()

    def send_updated_velocity_thread(self):
        while self.is_running:
            # print("Running update velocity thread")
            dl, dr = self.left_controller.get_current_value(), self.right_controller.get_current_value()
            if not (dl == self.prev_sent_velocities[0] and dr == self.prev_sent_velocities[1]):
                self.prev_sent_velocities = [dl, dr]
                self.drive_direct(dr, dl)
                # print("Sending new velocity: {}, {}".format(dl, dr,))
            time.sleep(0.05)

    def set_velocity_target(self, left, right):
        # print("Setting new velocity: {}, {}".format(left, right))
        self.left_controller.set_target(left)
        self.right_controller.set_target(right)

    def read_irobot_data(self):
        self.irobot_data = self.sensor_group100

    def update_from_sensors(self):
        self.read_irobot_data()

        self.time_in_stasis += time.time() - self.time_of_last_sensor_read
        if self.irobot_data.stasis.toggling:
            self.time_in_stasis = 0.0

        self.time_of_last_sensor_read = time.time()

        self.pos.update(self.irobot_data.left_encoder_counts, self.irobot_data.right_encoder_counts)
        self.publish_pose()

    def publish_pose(self):
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, -self.pos.theta)
        q = tf_conversions.transformations.quaternion_matrix(q)
        translation = tf_conversions.transformations.translation_matrix([self.pos.x, self.pos.y, 0])
        transform = np.dot(translation, q)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "bradbot"

        pose = tf_conversions.posemath.toMsg(tf_conversions.fromMatrix(transform))
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.w = pose.orientation.w
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z

        self.transform_broadcaster.sendTransform(t)

    def is_bumping(self):
        bumps = self.irobot_data.bumps_and_wheel_drops
        if bumps.bump_left or bumps.bump_right:
            return True
        lt = self.irobot_data.light_bumper
        if lt.left or lt.front_left or lt.center_left or lt.center_right or lt.front_right or lt.right:
            return True
        return False

    def cliff_sensed(self):
        s = self.irobot_data
        return s.cliff_left_sensor or s.cliff_front_left_sensor or \
               s.cliff_front_right_sensor or s.cliff_front_right_sensor

    def control_loop(self):
        """
        Main control loop. Call this to keep this periodically to keep the robot sensing and moving
        """
        self.update_from_sensors()
        self.handle_move()

    def handle_move(self):
        if not self.moving:
            return

        if np.linalg.norm([self.pos.x - self.x_target, self.pos.y - self.y_target]) <= self.err_limit:
            print("Reached! Stopping")
            self.moving = False

        vr, vl = self.simple_control(self.x_target, self.y_target, self.vel)

        if self.is_bumping() and vr + vl > 0:
            print("Bumped! Stopping. Will not move forward")
            self.moving = False

        if not self.moving:
            self.drive_direct(0, 0)
            return

        self.drive_direct(vr, vl)

    def go_to(self, x_target, y_target, vel=400, err_limit=0.05):
        """
        Commands the robot to move to the target, returning true if reached within error limit.
        Blocks
        """
        self.moving = True
        self.x_target = x_target
        self.y_target = y_target
        self.vel = vel
        self.err_limit = err_limit

    def simple_control(self, x_target, y_target, vel):
        """
        returns (vr, vl) wheel velocities using a simple controller
        """
        th_limit = 0.1  # rad

        """reset the robot origin to 0"""
        x_target -= self.pos.x
        y_target -= self.pos.y
        th = self.pos.theta

        x_target_tmp = x_target * np.cos(th) - y_target * np.sin(th)
        y_target = x_target * np.sin(th) + y_target * np.cos(th)
        x_target = x_target_tmp

        th_target = np.arctan2(y_target, x_target)
        print("th: %.2f, x: %.2f, y: %.2f" % (th_target, x_target, y_target))

        vr = min(vel, vel * np.abs(th_target) * 3) * np.sign(th_target)
        vl = -vr

        if np.abs(th_target) > th_limit:
            return vr, vl

        vr *= 2
        vl *= 2
        vr += vel
        vl += vel

        scale = vel / max(np.abs(vr), np.abs(vl))
        vr *= scale
        vl *= scale

        return vr, vl

    def set_katie_song(self):
        q = 36  # quarter note
        e = q / 2  # eigth note

        notes = [[69, q - 4],
                 [1, 4],
                 [74, q],
                 [74, e],
                 [76, e],
                 [77, e],
                 [74, e],
                 [1, e],
                 [74, e],
                 [73, e],
                 [76, e],
                 [1, e],
                 [76, e],
                 [77, e],
                 [74, e]
                 ]
        self.set_song(0, notes)
        self.play_song(0)
