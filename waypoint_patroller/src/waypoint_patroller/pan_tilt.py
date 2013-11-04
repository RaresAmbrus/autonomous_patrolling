import rospy

import smach
import smach_ros

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from scitos_msgs.msg import PanTilt

from logger import Loggable

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from scitos_ptu.msg import *
from sensor_msgs.msg import JointState

class PanTiltState(smach.State, Loggable):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded', 'failure', 'not_defined'],
            input_keys=['goal_pan_tilt']

        )

        rospy.loginfo("Creating ptu movement client.")
        self.ptuClient = actionlib.SimpleActionClient(
            'ptu_pan_tilt',
            PanTiltAction
        )
        self.ptuClient.wait_for_server()
        rospy.loginfo("PanTilt client initialized")

    def execute(self,userdata):

        rospy.loginfo("PanTilting the Xtion.")
        if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
            return 'abort'
		
	pt = userdata.goal_pan_tilt

	print 'Pan Tilt parameters ',pt

        if (not (userdata.goal_pan_tilt.pan_increment == -1 and userdata.goal_pan_tilt.tilt_increment == -1)):
           pose = scitos_ptu.msg.PanTiltGoal()
           pose.pan_start = userdata.goal_pan_tilt.pan_start
           pose.pan_step = userdata.goal_pan_tilt.pan_increment
           pose.pan_end = userdata.goal_pan_tilt.pan_end
           pose.tilt_start = userdata.goal_pan_tilt.tilt_start
           pose.tilt_step = userdata.goal_pan_tilt.tilt_increment
           pose.tilt_end = userdata.goal_pan_tilt.tilt_end

          # START LOGGING HERE
#           logging_server = rospy.ServiceProxy('loggins_server', LoggingServer)
#           success = logging_server('start', '')

           self.ptuClient.send_goal(pose)
           self.ptuClient.wait_for_result()
           result=self.ptuClient.get_result()
           rospy.loginfo("PanTilt action server returned with state ")
           rospy.loginfo(result)

          # END LOGGIING HERE
 #          success = logging_server('stop', '')

           if result != GoalStatus.SUCCEEDED:
              return 'failure'
           else:
	      return 'succeeded'
        else:
            rospy.loginfo("Pan tilt action not defined for this waypoint.")
            return 'not_defined'


