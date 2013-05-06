#!/usr/bin/env python
import roslib; roslib.load_manifest('wsg_50_grasp_controller')
import sys
import rospy
import actionlib
from wsg_50_common.srv import Move
from wsg_50_common.msg import Cmd
from sensor_msgs.msg import JointState
from object_manipulation_msgs.msg import GraspHandPostureExecutionAction, GraspHandPostureExecutionGoal
from object_manipulation_msgs.srv import GraspStatus, GraspStatusResponse

class WSG50GraspController:

    def __init__(self):

        # Check for gripper grasp services
        gripper_service_grasp_name = rospy.resolve_name('gripper_service_grasp_name')
        while(True):
            try:
                rospy.wait_for_service(gripper_service_grasp_name, 2)
                rospy.loginfo('Found service on %s' % gripper_service_grasp_name)
                self.gripper_grasp_srv = rospy.ServiceProxy(gripper_service_grasp_name, Move)
                break;
            except rospy.ROSException, e:
                rospy.loginfo('Waiting for service on %s' % gripper_service_grasp_name)

        gripper_service_move_name = rospy.resolve_name('gripper_service_move_name')
        while(True):
            try:
                rospy.wait_for_service(gripper_service_move_name, 2)
                rospy.loginfo('Found service on %s' % gripper_service_move_name)
                self.gripper_move_srv = rospy.ServiceProxy(gripper_service_move_name, Move)
                break;
            except rospy.ROSException, e:
                rospy.loginfo('Waiting for service on %s' % gripper_service_move_name)


        self.gripper_virtual_joint_name = rospy.get_param('~gripper_virtual_joint_name', 'undefined')
        if self.gripper_virtual_joint_name == 'undefined':
            rospy.logerr('Fingers joint names not defined set parameters gripper_virtual_joint_name in the launch file')


        self.gripper_closed_gap_value = rospy.get_param('~gripper_closed_gap_value', 10)
        #self.gripper_max_effort = rospy.get_param('~gripper_max_effort', 10)
        self.gripper_opened_gap_value = rospy.get_param('~gripper_opened_gap_value', 110)
        self.gripper_object_presence_threshold = rospy.get_param('~gripper_object_presence_threshold', 1)

        # grasp_query_name service Publisher
        grasp_query_name = rospy.resolve_name('grasp_query_name')
        self.query_pub = rospy.Service(grasp_query_name, GraspStatus, self.service_callback)
        rospy.loginfo('WSG50 grasp query service started on topic %s' % grasp_query_name)

        # start action server
        posture_action_name = rospy.resolve_name('posture_action_name')
        self.action_server = actionlib.SimpleActionServer(posture_action_name, GraspHandPostureExecutionAction, execute_cb=self.execute_cb)
        self.action_server.start()
        rospy.loginfo('WSG50 grasp hand posture action server started on topic %s' % posture_action_name)


    def execute_cb(self, goal):

        gripper_command = Cmd()

        ### Driver simulator do not implement speed regulation
        gripper_command.speed = 1

        if goal.goal == GraspHandPostureExecutionGoal.GRASP:

            if not goal.grasp.grasp_posture.position:
                rospy.logerr("WSG grasp controller: position vector empty in requested grasp")
                self.action_server.set_aborted()
                return

            gripper_command.mode = GraspHandPostureExecutionGoal.GRASP

        elif goal.goal == GraspHandPostureExecutionGoal.PRE_GRASP:

            if not goal.grasp.pre_grasp_posture.position:
                rospy.logerr("WSG grasp controller: position vector empty in requested pre_grasp")
                self.action_server.set_aborted()
                return

            gripper_command.mode = GraspHandPostureExecutionGoal.PRE_GRASP
            gripper_command.pos = goal.grasp.pre_grasp_posture.position[1] * 2 * 1000 # in mm

        elif goal.goal == GraspHandPostureExecutionGoal.RELEASE:
            gripper_command.mode = GraspHandPostureExecutionGoal.RELEASE
            gripper_command.pos = self.gripper_opened_gap_value

        else:
            rospy.logerr("WSG grasp controller: unknown goal code (%d)" % goal.goal)
            self.action_server.set_aborted()
            return

        if gripper_command.mode == GraspHandPostureExecutionGoal.GRASP:
            response = self.gripper_grasp_srv(gripper_command.pos, gripper_command.speed)
        else:
            response = self.gripper_move_srv(gripper_command.pos, gripper_command.speed)

        if response == 0:
            self.action_server.set_succeeded()
        else:
            rospy.logwarn("WSG grasp controller: gripper goal not achieved for pre-grasp or release")
            self.action_server.set_succeeded()


    def service_callback(self, req):

        response = GraspStatusResponse()

        gripper_value = self.get_gripper_value()
        min_gripper_opening = self.gripper_object_presence_threshold
        if gripper_value < self.gripper_object_presence_threshold:
            rospy.loginfo("Gripper grasp query false: gripper value %f below threshold %f", gripper_value, min_gripper_opening)
            response.is_hand_occupied = False;

        else:
            rospy.logdebug("Gripper grasp query true: gripper value %f above threshold %f", gripper_value, min_gripper_opening)
            response.is_hand_occupied = True;

        return response


    def get_gripper_value(self):

        joint_states = rospy.wait_for_message('joint_states', JointState)

        if not joint_states:
            rospy.logerr("WSG grasp controller: joint states not received")
            return 0

        return joint_states.position[joint_states.name.index(self.gripper_virtual_joint_name)]*2*1000

def main(args):

    rospy.init_node("wsg_50_grasp_controller")
    WSG50GraspController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("WSG50_grasp_controller  is closeing")

if __name__ == '__main__':
    main(sys.argv)
