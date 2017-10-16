import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

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

def positions_distance(a, b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

def get_closest_waypoint(pose, waypoints):
    best_distance = float('inf')
    best_index = 0

    for i, waypoint in enumerate(waypoints):

        waypoint_position = waypoint.pose.pose.position
        distance = positions_distance(pose.position, waypoint_position)

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

    x = shift_x * math.cos(0 - yaw) - shift_y * math.sin(0 - yaw)

    if x > 0:
        return False
    return True

def get_ahead_waypoint(pose, waypoints):

    index = get_closest_waypoint(pose, waypoints)
    is_behind = is_waypoint_behind(pose, waypoints[index])
    if is_behind:
        index += 1
    return index