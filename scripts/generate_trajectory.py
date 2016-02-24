#!/usr/bin/env python
import rospy
import sys

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def get_ros_param(param_name, default_value=None):
    target_var = rospy.get_param(param_name, default_value);
    rospy.loginfo("[TRAJ] Param %s : %s" % (param_name, target_var))
    return target_var

def main():

    pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    pub_land = rospy.Publisher('bebop/land', Empty, queue_size = 1)
    pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size = 30)

    empty_msg = Empty()
    cmdvel_msg = Twist()

    rospy.init_node('trajectory_generator', anonymous = False)

    abs_yaw_ctrl = get_ros_param("~abs_yaw_ctrl", False)
    abs_alt_ctrl = get_ros_param("~abs_alt_ctrl", False)
    do_takeoff = get_ros_param("~takeoff", False)
    do_lannd = get_ros_param("~land", False)
    plan_list = get_ros_param("~plan")

    if not plan_list or len(plan_list) <= 0:
        rospy.logfatal("[TRAJ] No plan has been specified")
        sys.exit(1)
    else:
        rospy.loginfo("[TRAJ] Plan size: %s" % (len(plan_list)))

    rospy.loginfo("[TRAJ] Starting the trajectory_generator node ...")

    last_transition_time = rospy.Time(0.0)
    current_timeout = rospy.Duration(0.0)
    plan_id = 0
    plan = {}
    rate = rospy.Rate(30)
    action = ''
    while not rospy.is_shutdown():
        try:
            if (rospy.Time.now() - last_transition_time > current_timeout):
                plan = plan_list[plan_id]
                current_timeout = rospy.Duration(plan['duration'])
                plan_id += 1
                last_transition_time = rospy.Time.now()

                action = plan['action']

                rospy.loginfo("Executing plan %s [%s]" % (plan_id, action))
                if (action == 'takeoff'):
                    pub_takeoff.publish(empty_msg)
                elif (action == 'land'):
                    pub_land.publish(empty_msg)
                elif (action == 'stop'):
                    cmdvel_msg.linear.x = 0.0
                    cmdvel_msg.linear.y = 0.0
                    cmdvel_msg.linear.z = 0.0
                    cmdvel_msg.angular.z = 0.0
                elif (action == 'vel'):
                    target = plan['target']
                    cmdvel_msg.linear.x = target['vx']
                    cmdvel_msg.linear.y = target['vy']

                    if (abs_alt_ctrl):
                        cmdvel_msg.linear.z = target['alt']
                    else:
                        cmdvel_msg.linear.z = target['vz']

                    if (abs_yaw_ctrl):
                        cmdvel_msg.angular.z = target['yaw']
                    else:
                        cmdvel_msg.angular.z = target['vw']
                elif (action == 'wait'):
                    pass
                else:
                    rospy.logerr("[TRAJ] Invalid action")
                    current_timeout = rospy.Duration(0.0)
        except IndexError:
            break
        except KeyError as e:
            rospy.logfatal("[TRAJ] Bad plan: %s" % (e, ))
            pub_land.publish(empty_msg)

        if action in ['vel', 'stop']:
            pub_twist.publish(cmdvel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass