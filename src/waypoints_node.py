#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3
from std_msgs.msg import Duration

def create_point(x, y, z, qx, qy, qz, qw, secs, nsecs):
    point = MultiDOFJointTrajectoryPoint()
    transform = Transform()
    transform.translation.x = x
    transform.translation.y = y
    transform.translation.z = z
    transform.rotation.x = qx
    transform.rotation.y = qy
    transform.rotation.z = qz
    transform.rotation.w = qw
    point.transforms.append(transform)

    # Assuming zero velocities and accelerations for simplicity
    point.velocities.append(Twist())
    point.accelerations.append(Twist())

    point.time_from_start = rospy.Duration(secs, nsecs)
    return point

def main():
    rospy.init_node('waypoints_publisher')

    pub = rospy.Publisher('/rmf_obelix/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        trajectory = MultiDOFJointTrajectory()

        trajectory.points.append(create_point(18.0, 0, 3, 0.0, 0.0, 0.0, 1, 6, 0))
        # trajectory.points.append(create_point(18.0, 0, 3, 0.0, 0.0, 0.0, 1,6,5000000000*10 ))
        # trajectory.points.append(create_point(18.0, 0, 0.5, 0.0, 0.0, 0.0, 1,6,5000000000 * 20 ))
        # trajectory.points.append(create_point(2, 0, 0.5, 0.0, 0.0, 0.0, 1, 6, 5000000000 * 30))
        # trajectory.points.append(create_point(2.0, 0, 3, 0.0, 0.0, 0.0, 1, 6, 5000000000 * 30))

        pub.publish(trajectory)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
