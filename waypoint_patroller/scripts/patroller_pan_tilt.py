#! /usr/bin/env python
import time
from random import randint
import rospy
import csv


import smach
import smach_ros
from smach_ros import SimpleActionState

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from scitos_ptu.msg import *
from sensor_msgs.msg import JointState 


frame_id="/map"
is_random=1;


class PanTilt(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['success', 'failure'],
            input_keys=['ptu_pose']
            
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


	print userdata.ptu_pose

        self.ptuClient.send_goal(userdata.ptu_pose)

        self.ptuClient.wait_for_result()

        result=self.ptuClient.get_state()
        

    #    while result == GoalStatus.PENDING or result == GoalStatus.ACTIVE:
     #       result=self.baseClient.get_state()
      #      if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
       #         return 'abort'

        if result != GoalStatus.SUCCEEDED:
            return 'failure'
        

        time.sleep(0.3) #avoid jumping out of a state immediately after entering it - actionlib bug
        return 'success'

class GoTo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['success', 'failure'],
            input_keys=['goal_pose']
            
        )

        rospy.loginfo("Creating base movement client.")
        self.baseClient = actionlib.SimpleActionClient(
            'move_base',
            MoveBaseAction
        )
        self.baseClient.wait_for_server()
        rospy.loginfo("Base client initialized")

    def execute(self,userdata):

        rospy.loginfo("Moving the base.")
        if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
            return 'abort'


        self.baseClient.send_goal(userdata.goal_pose)

        self.baseClient.wait_for_result()

        result=self.baseClient.get_state()
        

    #    while result == GoalStatus.PENDING or result == GoalStatus.ACTIVE:
     #       result=self.baseClient.get_state()
      #      if rospy.is_shutdown(): # Exiting gracefully when ctrl-c is pressed
       #         return 'abort'

        if result != GoalStatus.SUCCEEDED:
            return 'failure'
        

        time.sleep(0.3) #avoid jumping out of a state immediately after entering it - actionlib bug
        return 'success'

      


class PointReader(smach.State):
    def __init__(self, waypoints_name):
        smach.State.__init__(self,
            outcomes    = ['goto_point','pan_tilt'],
            output_keys=['goal_pose','ptu_pose']
            
        )

	self.waypointReached = False
	self.points=[]
	with open(waypoints_name, 'rb') as csvfile:
	     	reader = csv.reader(csvfile, delimiter=',')
		for row in reader:
			current_row=[]
			for element in row:
				current_row.append(float(element))
        		self.points.append(current_row)

	
	self.current_point=0
	self.n_points=len(self.points)

    def execute(self,userdata):


	if (self.waypointReached or len(self.points[self.current_point]) < 12) :
		next_goal = move_base_msgs.msg.MoveBaseGoal()

		if is_random:
		  self.current_point=randint(0,self.n_points-1)
		  current_row=self.points[self.current_point]
		else:
		  current_row=self.points[self.current_point]
		  self.current_point=self.current_point+1
		  if self.current_point==self.n_points:
			  self.current_point=0	
	  
	
		next_goal.target_pose.header.frame_id = frame_id
		next_goal.target_pose.header.stamp = rospy.Time.now()
		next_goal.target_pose.pose.position.x=current_row[0]
		next_goal.target_pose.pose.position.y=current_row[1]
		next_goal.target_pose.pose.position.z=current_row[2]
		next_goal.target_pose.pose.orientation.x=current_row[3]
		next_goal.target_pose.pose.orientation.y=current_row[4]
		next_goal.target_pose.pose.orientation.z=current_row[5]
		next_goal.target_pose.pose.orientation.w=current_row[6]

		userdata.goal_pose=next_goal
		
		self.waypointReached = False
		return 'goto_point'
	else:
		pose = scitos_ptu.msg.PanTiltGoal()
		pose.target_ptu_pose = JointState()
		pose.target_ptu_pose.position = self.points[self.current_point][7:13]
		userdata.ptu_pose = pose
		self.waypointReached = True
		return 'pan_tilt'	




     	








def main():



    rospy.init_node('patroller')

    
    #Check if a waypoints file was given as argument
    if len(sys.argv)<2:
      rospy.logerr("No waypoints file given. Use rosrun waypoint_patroller patroller.py [path to csv waypoints file]. If you are using a launch file, see launch/patroller.launch for an example.")
      return 1
      
    waypoints_name=sys.argv[1]

    if len(sys.argv)>2:
      if sys.argv[2]=="false":
	is_random=0
	rospy.loginfo("Executing waypoint_patroller with sequential point selection.")
      else:
	rospy.loginfo("Executing waypoint_patroller with random point selection.")
    else:
      rospy.loginfo("Executing waypoint_patroller with random point selection.")

	  
  
	  
	
	
    
    
    frame_id="/map"



    #waypoints_name=rospy.get_param("/patroller/waypoints")

    # Create a SMACH state machine
    sm = smach.StateMachine(['succeeded','aborted','preempted'])
    with sm:
        smach.StateMachine.add('POINT_READER', PointReader(waypoints_name), 
                               transitions={'goto_point':'GOING_TO_POINT', 'pan_tilt':'PAN_TILT_XTION'},
                               remapping={'goal_pose':'goal_pose','ptu_pose':'ptu_pose'})

        smach.StateMachine.add('GOING_TO_POINT', GoTo(), 
                               transitions={'success':'POINT_READER','failure':'aborted'},
                               remapping={'goal_pose':'goal_pose'})

        smach.StateMachine.add('PAN_TILT_XTION', PanTilt(), 
                               transitions={'success':'POINT_READER','failure':'aborted'},
                               remapping={'ptu_pose':'ptu_pose'})


 
    

    # Execute SMACH plan

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()






0