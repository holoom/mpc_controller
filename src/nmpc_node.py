#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import Vector3Stamped, TwistStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory
import casadi as ca
import numpy as np

# Global variables
desired_waypoint = None
thrust_pub = None
angular_rates_pub = None 

def waypoint_callback(data):
    global desired_waypoint
    if data.points:
        last_point = data.points[-1]
        translation = last_point.transforms[0].translation
        rotation = last_point.transforms[0].rotation
        #calculate norm of rotation
        rotation_array = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        norm = np.linalg.norm(rotation_array)

        linear_velocity = last_point.velocities[0].linear
        desired_waypoint = np.array([translation.x, translation.y, translation.z, rotation.w, rotation.x, rotation.y, rotation.z, linear_velocity.x, linear_velocity.y, linear_velocity.z])
        rospy.loginfo("desired waypoint: %s",desired_waypoint)
        rospy.loginfo("Norm of q_desired: %s", norm)
    else:
        rospy.loginfo("Received an empty trajectory.")


def nmpc_controller(ref, state):
    param = np.concatenate((state, ref))  # current state and ref
    opt_u = NMPC (param)
    return opt_u.toarray()

def state_callback(data):
    position = data.position
    orientation = data.orientation
    norm_q = np.linalg.norm(np.array([orientation.x, orientation.y, orientation.z, orientation.w]))
    # rospy.loginfo("norm_q_current: %s", norm_q)
    state = np.array([position.x, position.y, position.z, orientation.w, orientation.x, orientation.y, orientation.z, 0, 0, 0])
    if desired_waypoint is not None:
        opt_u = nmpc_controller(desired_waypoint, state)
        # Publish Thrust
        thrust_msg = Vector3Stamped()
        thrust_msg.header = Header(stamp=rospy.Time.now(), frame_id="rmf_obelix/base_link")
        thrust_msg.vector.z= opt_u[0,0]
        thrust_pub.publish(thrust_msg)

        # Publish Angular Rates
        twist_msg = TwistStamped()
        twist_msg.header = Header(stamp=rospy.Time.now(), frame_id="rmf_obelix/base_link")
        twist_msg.twist.angular.x = opt_u[1,0]
        twist_msg.twist.angular.y = opt_u[2,0]
        twist_msg.twist.angular.z = opt_u[3,0]
        angular_rates_pub.publish(twist_msg)
         

def main():
    global thrust_pub, angular_rates_pub
    rospy.init_node('NMPC_controller_node', anonymous=True)
    state_sub = rospy.Subscriber('/rmf_obelix/ground_truth/pose', Pose, state_callback)
    waypoint_sub = rospy.Subscriber('/rmf_obelix/command/trajectory', MultiDOFJointTrajectory, waypoint_callback)
    # waypoint_sub = rospy.Subscriber('/waypoints', MultiDOFJointTrajectory, waypoint_callback)
    # cmd_pub = rospy.Publisher('/thrust_and_angularrates', Twist, queue_size=10)
    
    thrust_pub = rospy.Publisher('/thrust', Vector3Stamped, queue_size=10)
    angular_rates_pub = rospy.Publisher('/angular_rates', TwistStamped, queue_size=10)
    rospy.spin()  

if __name__ == "__main__":
    NMPC = ca.Function.load('/home/aosaad/ws_ttk22_ros/src/mpc_controller/src/NMPC.casadi')
    main()
