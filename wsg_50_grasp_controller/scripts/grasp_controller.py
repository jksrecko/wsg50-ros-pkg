#!/usr/bin/env python
import roslib; roslib.load_manifest('wsg_50_grasp_controller')
import sys
import rospy
import actionlib
from wsg_50_common.srv import Move
from wsg_50_common.msg import Cmd
from object_manipulation_msgs.action import GraspHandPostureExecution
from object_manipulation_msgs.srv import GraspStatus

class WSG50GraspController:

    def __init__(self):

        # Check for gripper grasp services
        gripper_service_grasp_name = rospy.resolve_name('gripper_service_grasp_name')
        while(True):
            try:
                rospy.wait_for_service(gripper_service_grasp_name, 2)
                rospy.loginfo('Found service on %s' % gripper_service_grasp_name)
                gripper_grasp_srv = rospy.ServiceProxy(gripper_service_grasp_name, Move)
                break;
            except rospy.ROSException, e:
                rospy.loginfo('Waiting for service on %s' gripper_service_grasp_name)

        gripper_service_move_name = rospy.resolve_name('gripper_service_move_name')
        while(True):
            try:
                rospy.wait_for_service(gripper_service_move_name, 2)
                rospy.loginfo('Found service on %s' % gripper_service_move_name)
                gripper_move_srv = rospy.ServiceProxy(gripper_service_move_name, Move)
                break;
            except rospy.ROSException, e:
                rospy.loginfo('Waiting for service on %s' gripper_service_move_name)

        self.gripper_closed_gap_value = rospy.get_param('~gripper_closed_gap_value', 10)
        self.gripper_max_effort = rospy.get_param('~gripper_max_effort', 10)
        self.gripper_opened_gap_value = rospy.get_param('~gripper_opened_gap_value', 110)


        # grasp_query_name service Publisher
        grasp_query_name = rospy.resolve_name('grasp_query_name')
        self.query_pub = rospy.Service(grasp_query_name, GraspStatus, self.service_callback)
        rospy.loginfo('WSG50 grasp query service started on topic %s' % grasp_query_name)

        # start action server
        posture_action_name = rospy.resolve_name('posture_action_name')
        self.action_server = actionlib.SimpleActionServer(posture_action_name, GraspHandPostureExecution, execute_cb=self.execute_cb)
        self.action_server.start()
        rospy.loginfo('WSG50 grasp hand posture action server started on topic %s' % posture_action_name)


    def execute_cb(self, goal):

        gripper_command = Cmd()

        ### Driver simulator do not implement speed regulation
        gripper_command.speed = 1

        if goal.goal == GraspHandPostureExecution.GRASP:

            if not goal.grasp.grasp_posture.position:
                rospy.logerror("WSG grasp controller: position vector empty in requested grasp")
                self.action_server.set_aborted()
                return

            gripper_command.mode = GraspHandPostureExecution.GRASP
            # maybe joint to gap!!

########### Not implemented in driver simulation and in Move message!!! ############
#            if not goal.grasp.grasp_posture.effort:
#                if goal.max_contact_force > 0:
#                    gripper_command.max_effort = goal.max_contact_force
#                    rospy.loginfo("WSG grasp controller: limiting max contact force to %f" % gripper_command.max_effort)
#                else:
#                    gripper_command.max_effort = self.gripper_max_effort
#                    rospy.logwarn("WSG grasp controller: effort vector empty in requested grasp, using max force")

#            else:
#                if (goal.grasp.grasp_posture.effort[0] < goal.max_contact_force || goal.max_contact_force == 0):
#                    gripper_command.max_effort = goal.grasp.grasp_posture.effort[0]
#                else:
#                    gripper_command.max_effort = goal.max_contact_force

        elif goal.goal == GraspHandPostureExecution.PRE_GRASP:

            if not goal.grasp.pre_grasp_posture.position:
                rospy.logerror("WSG grasp controller: position vector empty in requested pre_grasp")
                self.action_server.set_aborted()
                return

            gripper_command.mode = GraspHandPostureExecution.PRE_GRASP
            gripper_command.pos = goal.grasp.pre_grasp_posture.position[0] * 2
            # maybe joint to gap!!

########### Not implemented in driver simulation and in Move message!!! ############
#            if not goal.grasp.pre_grasp_posture.effort:
#                gripper_command.max_effort = self.gripper_max_effort
#                rospy.logwarn("WSG grasp controller: effort vector empty in requested grasp, using max force")
#            else:
#                gripper_command.max_effort = goal.grasp.pre_grasp_posture.effort[0]

        elif goal.goal = GraspHandPostureExecution.RELEASE:
            gripper_command.mode = GraspHandPostureExecution.RELEASE
            gripper_command.width = self.gripper_opened_gap_value
            #gripper_command.max_effort = self.gripper_max_effort

        else:
            rospy.logerror("WSG grasp controller: unknown goal code (%d)" % goal.goal)
            action_server.set_aborted()
            return






    def service_callback(self, req):

        #TODO implementation see: pr2_gripper_grasp_controller.cpp

        return True





def main(args):

    rospy.init_node("wsg_50_grasp_controller")
    WSG50GraspController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("WSG50_grasp_controller  is closeing")

if __name__ == '__main__':
    main(sys.argv)
