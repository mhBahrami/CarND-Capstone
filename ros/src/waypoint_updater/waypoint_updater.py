#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.6
MAX_NUM = float('inf')

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # The `/current_pose` topic provides the vehicle's current position
        rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb)
        # The `/base_waypoints` provides a complete list of waypoints the car will be following
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # The `/traffic_waypoint` topic provides locations to stop for red traffic lights
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb)
        # The `/obstacle_waypoint` topic provides locations of obstacles
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_waypoint_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.speed_limit = self.kmph2mps(rospy.get_param('waypoint_loader/velocity', 40))
        # rospy.logwarn(">> WaypointUpdater - __init__ | speed_limit: {0}".format(self.speed_limit))
        # Add other member variables you need below
        self.current_pose = None
        self.base_line = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.current_velocity = None
        self.stopline_wp_idx = None
        self.waypoint_speeds = []

        self.loop()


    def loop(self):
        '''
        This function allows us to control publishing frequency.
        '''
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # rospy.logwarn("{0},{1},{2}".format(self.current_pose is not None, self.base_line is not None, self.current_velocity is not None))
            if not None in (self.current_pose, self.base_line, self.current_velocity):
                self.publish_waypoints()

            rate.sleep()

    
    def kmph2mps(self, velocity_kmph):
        return velocity_kmph / 3.6


    def mps2kmph(self, velocity_mps):
        return velocity_mps * 3.6
    

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)


    def generate_lane(self):
        # Get closest waypoint 
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_line.waypoints[closest_idx:farthest_idx]

        # rospy.logwarn(">> WaypointUpdater | closest_idx {0}, stopline_idx {1}, farthest_idx {2}, cur_vel: {3}".format(closest_idx, self.stopline_wp_idx, farthest_idx, self.current_velocity))
        
        lane = Lane()
        lane.header.frame_id = self.base_line.header.frame_id
        lane.header.stamp = rospy.Time(0)

        if self.should_accelerate(self.stopline_wp_idx, closest_idx, farthest_idx):
            lane.waypoints = self.accelerate_waypoints(base_waypoints)
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane
        

    def get_closest_waypoint_idx(self):
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        # The `query` will return the position and also the index.
        # The index is in the same order as we put into the`KDTree`.
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:
            # It means that the closest waypoint is behind the car.
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx


    def should_accelerate(self, stop_line_idx, closest_idx, farthest_idx):
        if stop_line_idx is None:
            return False

        if stop_line_idx == -2:
            # TrafficLight.GREEN
            return True
        elif stop_line_idx == -4:
            # TrafficLight.UNKNOWN
            return True
        elif stop_line_idx < 0:
            # TrafficLight.YELLOW
            # Calculate the distance
            if -stop_line_idx <= closest_idx or -stop_line_idx >= farthest_idx:
                return True
            else:
                dist = self.distance(self.base_line.waypoints, closest_idx, -stop_line_idx)
                YELLOW_SAFE_DIST = 16.0 # [m]
                return dist >= YELLOW_SAFE_DIST
        elif stop_line_idx >= 0:
            # TrafficLight.RED
            if stop_line_idx <= closest_idx or stop_line_idx >= farthest_idx:
                return True
            else:
                dist = self.distance(self.base_line.waypoints, closest_idx, stop_line_idx)
                # rospy.logwarn(">> WaypointUpdater - should_accelerate | dist {0}".format(dist))
                return dist >= self.get_stop_distance()

        else:
            return False
        

    def get_stop_distance(self):
        """
        A function that gives the appropriate decelerate distance for TrafficLight.RED 
        """
        stop_dist = 0.0 # [m]
        speed = self.mps2kmph(self.current_velocity) # [km/h]
        if speed < 20.0:
            stop_dist = 15.0
        elif speed >=20.0 and speed < 30.0:
            stop_dist = 0.7*(speed-20.0)+15.0
        elif speed >=30.0 and speed < 40.0:
            stop_dist = 1.0*(speed-30.0)+22.0
        else: # speed >=40.0
            stop_dist = 1.2*(speed-40.0)+32.0

        return stop_dist
 

    def accelerate_waypoints(self, waypoints):
        accel_wps = []

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            velocity = self.speed_limit
            p.twist.twist.linear.x = velocity
            accel_wps.append(p)

        return accel_wps


    def decelerate_waypoints(self, waypoints, closest_idx):
        decel_wps = []
        # 3 waypoints back from line so front of car stops at line
        stop_idx = max((self.stopline_wp_idx - 3) - closest_idx, 0) if self.stopline_wp_idx is not None else 0
        while stop_idx >= len(waypoints):
            stop_idx -= len(waypoints)

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, i, stop_idx)
            velocity = math.sqrt(2 * MAX_DECEL * dist)
            if velocity < 1.0:
                velocity = 0.0

            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            decel_wps.append(p)

        return decel_wps


    def current_pose_cb(self, msg):
        '''
        This callback is called with 50 Hz rate
        and only stores the current car's position.
        '''
        self.current_pose = msg


    def base_waypoints_cb(self, waypoints):
        '''
        This callback is called once.
        '''
        # Store the waypoint in an object
        # rospy.logwarn(">> base_waypoints_cb | waypoints: {0}".format(waypoints is not None))
        self.base_line = waypoints

        for i in range(len(self.base_line.waypoints)):
            self.base_line.waypoints[i].pose.header.frame_id = self.base_line.header.frame_id
            # self.waypoint_speeds.append(self.speed_limit)

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)


    def traffic_waypoint_cb(self, msg):
        """
        Callback for `/traffic_waypoint` message.

        + `msg` value meanings:
            - TrafficLight.UNKNOWN: msg = -4
            - TrafficLight.GREEN  : msg = -2
            - TrafficLight.YELLOW : msg = -(stop line waypoint's index)
            - TrafficLight.RED    : msg = stop line waypoint's index
        """
        # rospy.logwarn(">> WaypointUpdater - traffic_waypoint_cb | State: {0}".format(msg.data))
        self.stopline_wp_idx = msg.data


    def obstacle_waypoint_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        pass


    def current_velocity_cb(self, msg):
        v = msg.twist.linear.x
        self.current_velocity = 0.0 if v < 0.1 else v


    def get_waypoint_velocity(self, waypoint):

        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):

        waypoints[waypoint].twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1_idx, wp2_idx):
        # rospy.logwarn(">> WaypointUpdater - distance | wp1_idx: {0}, wp2_idx: {1}".format(wp1_idx, wp2_idx))
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1_idx, wp2_idx+1):                
            dist += dl(waypoints[wp1_idx].pose.pose.position, waypoints[i].pose.pose.position)
            wp1_idx = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
