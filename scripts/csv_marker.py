import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import sys

def rviz_markers(pose,idx):
    if idx == 0:
        points = Marker()
        points.type = Marker.POINTS
        points.header.frame_id = "map"
        points.ns = "raceline"
        points.action = Marker.ADD
        points.pose.orientation.w = 1
        points.scale.x = 0.5
        points.scale.y = 0.5
        points.color.r = 1
        points.color.g = 0
        points.color.a = 1

        for i in pose:
            p = Point()
            p.x = i[0]
            p.y = i[1]
            points.points.append(p)

    elif idx == 1:
        points = Marker()
        points.type = Marker.POINTS
        points.header.frame_id = "map"
        points.ns = "spline"
        points.action = Marker.ADD
        points.pose.orientation.w = 1
        points.scale.x = 1
        points.scale.y = 1
        points.color.b = 1
        points.color.a = 1

    for i in pose:
        p = Point()
        p.x = i[0]
        p.y = i[1]
        points.points.append(p)

    return points

def get_spline_path(csv_f):

    waypoints = np.genfromtxt(csv_f, dtype=float, delimiter=",")
    coords = waypoints[1:,0:2]

    return coords


if __name__ == "__main__":
    rospy.init_node('csv_publisher')

    file_ = sys.argv[1]

    raceline_pub = rospy.Publisher('visualization_markers',Marker,queue_size=1)
    points = get_spline_path(file_)
    rate = rospy.Rate(50)
    print(points)
    while not rospy.is_shutdown():
        try:

            raceline_pub.publish(rviz_markers(points,0))

            rate.sleep()
        except IndexError:
            continue
        except RuntimeError:
            continue
        except rospy.exceptions.ROSInterruptException:
            break
