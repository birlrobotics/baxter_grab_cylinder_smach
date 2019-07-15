#!/usr/bin/python
# coding:utf-8

import rospy
import roslib
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from coordinator.srv import *
from coordinator.msg import *
import tf
import smach
import smach_ros
import xform_utils


# define state Gribbing
class Backing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Back_Complete', 'Back_false'])

    def execute(self, ud):
        rospy.loginfo("backing up")
        rest = rospy.Rate(1.0)
        rospy.wait_for_service("open_loop_nav_service")
        rospy.wait_for_service("open_loop_yaw_service")
        nav_move_client = rospy.ServiceProxy("open_loop_nav_service",
                                             OpenLoopNavSvc)
        nav_yaw_client = rospy.ServiceProxy("open_loop_yaw_service",
                                            OpenLoopNavSvc)
        rospy.loginfo('Trying to navigate to target place...')

        openLoopNavSvcMsg = coordinator.srv.OpenLoopNavSvcRequest(move_distance=-1.0)
        resp = nav_move_client(openLoopNavSvcMsg)
        rest.sleep()
        openLoopNavSvcMsg = coordinator.srv.OpenLoopNavSvcRequest(move_distance=1.57)
        resp = nav_yaw_client(openLoopNavSvcMsg)
        rest.sleep()

        return 'Back_Complete'


# define state Moving
class Approaching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Approach_Complete', 'Approach_false'])

    def execute(self, ud):
        rospy.loginfo('Approaching to next table...')
        rest = rospy.Rate(0.5)
        rate = rospy.Rate(5.0)
        table2_yaw_des = -1.57
        table2_y_des = -0.3

        rest.sleep()

        tfListener = tf.TransformListener()

        tferr = True
        while tferr:
            tferr = False
            try:
                (trans, rot) = tfListener.lookupTransform('map', 'base_link', rospy.Time(0))
                yaw = xform_utils.convertPlanarQuat2Phi(rot=rot)
                openLoopNavSvcMsg = coordinator.srv.OpenLoopNavSvcRequest(move_distance=table2_yaw_des - yaw)
                nav_yaw_client = rospy.ServiceProxy("open_loop_yaw_service",
                                                    OpenLoopNavSvc)
                resp = nav_yaw_client(openLoopNavSvcMsg)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("retrying...")
                tferr = True
                rate.sleep()

        rest.sleep()

        tferr = True
        while tferr:
            tferr = False
            try:
                (trans, rot) = tfListener.lookupTransform('map', 'base_link', rospy.Time(0))
                openLoopNavSvcMsg = coordinator.srv.OpenLoopNavSvcRequest(move_distance=-table2_y_des + trans[1])
                nav_move_client = rospy.ServiceProxy("open_loop_nav_service",
                                                     OpenLoopNavSvc)
                resp = nav_move_client(openLoopNavSvcMsg)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("retrying...")
                tferr = True
                rate.sleep()

        rest.sleep()

        tferr = True
        while tferr:
            tferr = False
            try:
                (trans, rot) = tfListener.lookupTransform('map', 'base_link', rospy.Time(0))
                current_pose = xform_utils.get_pose_from_stamped_tf(trans, rot)
                rospy.loginfo("tf is good; current pose is:")
                xform_utils.print_Stamped_Pose(current_pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("retrying...")
                tferr = True
                rate.sleep()

        return 'Approach_Complete'


def main():
    rospy.init_node('task_client_node')

    tfListener = tf.TransformListener()
    current_pose = geometry_msgs.msg.PoseStamped()
    pose_table2_approach = geometry_msgs.msg.PoseStamped()

    pose_table2_approach.header.frame_id = "/map"
    pose_table2_approach.header.stamp = rospy.Time.now()
    pose_table2_approach.pose.orientation.x = 0.0
    pose_table2_approach.pose.orientation.y = 0.0
    pose_table2_approach.pose.orientation.z = -0.707
    pose_table2_approach.pose.orientation.w = 0.707
    pose_table2_approach.pose.position.x = -8.8
    pose_table2_approach.pose.position.y = 0.18
    pose_table2_approach.pose.position.z = 0.0
    move_base_goal2 = MoveBaseGoal()
    move_base_goal2.target_pose = pose_table2_approach

    via_pose = geometry_msgs.msg.PoseStamped()
    via_pose.header.frame_id = "/map"
    via_pose.header.stamp = rospy.Time.now()
    via_pose.pose.orientation.x = 0.62
    via_pose.pose.orientation.y = 6.4
    via_pose.pose.orientation.z = 0.707
    via_pose.pose.orientation.w = 0.707
    via_pose.pose.position.x = -8.8
    via_pose.pose.position.y = 0.18
    via_pose.pose.position.z = 0.0
    move_base_goal1 = MoveBaseGoal()
    move_base_goal1.target_pose = via_pose

    tferr = True
    tfBaseLinkWrtMap = geometry_msgs.msg.TransformStamped()

    goal1 = ManipTaskGoal()
    goal1.action_code = ManipTaskGoal.MOVE_TO_PRE_POSE

    goal2 = ManipTaskGoal()
    goal2.action_code = ManipTaskGoal.FIND_TABLE_SURFACE

    goal3 = ManipTaskGoal()
    goal3.action_code = ManipTaskGoal.GET_PICKUP_POSE
    goal3.object_code = 1001
    goal3.perception_source = ManipTaskGoal.PCL_VISION

    goal4 = ManipTaskGoal()
    goal4.action_code = ManipTaskGoal.GRAB_OBJECT
    goal4.object_code = 1001
    goal4.perception_source = ManipTaskGoal.PCL_VISION

    goal5 = ManipTaskGoal()
    goal5.action_code = ManipTaskGoal.MOVE_TO_PRE_POSE
    goal5.object_code = 1001

    goal6 = ManipTaskGoal()
    goal6.action_code = ManipTaskGoal.DROPOFF_OBJECT
    goal6.object_code = 1001
    goal6.perception_source = ManipTaskGoal.PCL_VISION

    # rospy.wait_for_service("manip_task_action_service")
    # rospy.wait_for_service("move_base")

    rate = rospy.Rate(2.0)
    rospy.loginfo("waiting for tf between map and base_link...")
    while tferr:
        tferr = False
        try:
            (trans, rot) = tfListener.lookupTransform('map', 'base_link', rospy.Time(0))
            current_pose = xform_utils.get_pose_from_stamped_tf(trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("retrying...")
            tferr = True
            rate.sleep()

    rospy.loginfo("tf is good; current pose is:")
    xform_utils.print_Stamped_Pose(current_pose)

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        def mani_result_cb(userdata, status, result):
            g_callback_status = result.manip_return_code
            g_goal_done = 'aborted'

            if g_callback_status == coordinator.msg.ManipTaskResult.MANIP_SUCCESS:
                rospy.loginfo("returned MANIP_SUCCESS")
                g_goal_done = 'succeeded'
            elif g_callback_status == coordinator.msg.ManipTaskResult.FAILED_PERCEPTION:
                rospy.logwarn("returned FAILED_PERCEPTION")
            elif g_callback_status == coordinator.msg.ManipTaskResult.FAILED_PICKUP:
                rospy.logwarn("returned FAILED_PICKUP")
            elif g_callback_status == coordinator.msg.ManipTaskResult.FAILED_DROPOFF:
                rospy.logwarn("returned FAILED_DROPOFF")

            return g_goal_done

        def find_obj_result_cb(userdata, status, result):
            goal4.pickup_frame = result.object_pose
            goal5.pickup_frame = result.object_pose
            goal6.dropoff_frame = result.object_pose
            # goal6.dropoff_frame.pose.position.z += 0.01
            print ("object pose:")
            xform_utils.print_Stamped_Pose(result.object_pose)

            return mani_result_cb(userdata, status, result)

        def navigator_done_cb(userdata, status, result):
            rospy.loginfo(" navigatorDoneCb: server responded with state [%s]", status)
            return 'succeeded'

        smach.StateMachine.add('move_pre',
                               smach_ros.SimpleActionState('manip_task_action_service',
                                                           ManipTaskAction,
                                                           goal=goal1,
                                                           result_cb=mani_result_cb,
                                                           output_keys=['move_pre_output']),
                               transitions={'succeeded': 'find_table'})

        smach.StateMachine.add('find_table',
                               smach_ros.SimpleActionState('manip_task_action_service',
                                                           ManipTaskAction,
                                                           goal=goal2,
                                                           result_cb=mani_result_cb,
                                                           output_keys=['find_table_output']),
                               transitions={'succeeded': 'find_obj'})

        smach.StateMachine.add('find_obj',
                               smach_ros.SimpleActionState('manip_task_action_service',
                                                           ManipTaskAction,
                                                           goal=goal3,
                                                           result_cb=find_obj_result_cb,
                                                           output_keys=['find_obj_output']),
                               transitions={'succeeded': 'grab_tar'})

        smach.StateMachine.add('grab_tar',
                               smach_ros.SimpleActionState('manip_task_action_service',
                                                           ManipTaskAction,
                                                           goal=goal4,
                                                           result_cb=mani_result_cb,
                                                           output_keys=['grab_tar_output']),
                               transitions={'succeeded': 'bring_back_tar'})

        smach.StateMachine.add('bring_back_tar',
                               smach_ros.SimpleActionState('manip_task_action_service',
                                                           ManipTaskAction,
                                                           goal=goal5,
                                                           result_cb=mani_result_cb,
                                                           output_keys=['bring_back_tar_output']),
                               transitions={'succeeded': 'back_up'})

        smach.StateMachine.add('back_up', Backing(),
                               transitions={'Back_Complete': 'navigate1',
                                            'Back_false': 'aborted'})

        smach.StateMachine.add('navigate1',
                               smach_ros.SimpleActionState('move_base',
                                                           MoveBaseAction,
                                                           goal=move_base_goal1,
                                                           result_cb=navigator_done_cb,
                                                           output_keys=['navigate1_output']),
                               transitions={'succeeded': 'navigate2'})

        smach.StateMachine.add('navigate2',
                               smach_ros.SimpleActionState('move_base',
                                                           MoveBaseAction,
                                                           goal=move_base_goal2,
                                                           result_cb=navigator_done_cb,
                                                           output_keys=['navigate1_output']),
                               transitions={'succeeded': 'approach'})

        smach.StateMachine.add('approach', Approaching(),
                               transitions={'Approach_Complete': 'drop_off',
                                            'Approach_false': 'aborted'})

        smach.StateMachine.add('drop_off',
                               smach_ros.SimpleActionState('manip_task_action_service',
                                                           ManipTaskAction,
                                                           goal=goal6,
                                                           result_cb=mani_result_cb,
                                                           output_keys=['drop_off_output']),
                               transitions={'succeeded': 'succeeded'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()


if __name__ == "__main__":
    main()
