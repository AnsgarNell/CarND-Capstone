#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb
        #rospy.Subscriber('/obstacle_waypoint', ????, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # TODO: Add other member variables you need below
    
        self.pose = None
        self.waypoints = None
        rospy.spin()

    def positions_distance(a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def get_closest_waypoint(pose, waypoints):
        best_distance = float('inf')
        best_index = 0
        pose_position = pose.position

        for i, waypoint in enumerate(waypoints):

            waypoint_position = waypoint.pose.pose.position
            distance = positions_distance(pose_position, waypoint_position)

            if distance < best_distance:
                best_index, best_distance = i, distance

        return best_index

    def is_waypoint_behind(pose, waypoint):
        _, _, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                         pose.orientation.y,
                                                         pose.orientation.z,
                                                         pose.orientation.w])
        x = pose.position.x
        y = pose.position.y

        shift_x = waypoint.pose.pose.position.x - x
        shift_y = waypoint.pose.pose.position.y - y

        x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)

        if x > 0:
            return False
        return True

    def get_ahead_waypoint(self, pose, waypoints):

        index = get_closest_waypoint(pose, waypoints)
        is_behind = is_waypoint_behind(pose, waypoints[index])
        if is_behind:
            index += 1

    def pose_cb(self, msg):
        # TODO: Implement
        rospy.loginfo(rospy.get_caller_id() + "pose_cb received")
        self.pose = msg.pose
        index = get_ahead_waypoint(self.pose, self.waypoints)

        final_waypoints = []

        for i in range(LOOKAHEAD_WPS):
            if (index == len(self.waypoints)): index = 0
            final_waypoints[i] = self.waypoints[index]
            index += 1

        self.final_waypoints_pub.publish(final_waypoints)
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo(rospy.get_caller_id() + "base_waypoints received")
        self.waypoints = waypoints
        #pass
    

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
