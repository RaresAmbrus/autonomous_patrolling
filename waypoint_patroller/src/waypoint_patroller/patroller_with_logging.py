import smach
import smach_ros
import rospy
import os, datetime
from subprocess import Popen, PIPE
import shlex

from random import shuffle
from . import charging, navigation
from logger import Loggable
from pan_tilt import *


from scitos_msgs.msg import BatteryState
from scitos_msgs.msg import PanTilt

from geometry_msgs.msg import Pose

import strands_datacentre as datacentre
import strands_datacentre.util
got_pymongo = strands_datacentre.util.check_for_pymongo()
if got_pymongo:
    import pymongo

    
    
    
class VeryLowBattery(smach.State, Loggable):
    def __init__(self):
        smach.State.__init__(self)
        
    def execute(self, userdata):
        while not rospy.is_shutdown():
            rospy.sleep(1)
    
    

"""
The point chooser state selects which waypoint to visit next. It checks the
battery life, and if it is greater than CHARGE_BATTERY_TRESHOLD, it selects
a new patrol point. Otherwise, the robot is sent to the charging station
point (the point in the dataset with the name "charging_point". The ordering
of visitng the patrolling points is either sequential or random, the constructor
parameter, as is the number of iterations the robot should do until the state
machine terminates with success

outcomes: 	'patrol': 		a standard waypoint was selected
            'go_charge':	the charging station is the next point
            'succeeded':	if required no. of iterations achieved

ouput keys:	'goal_pose':		geometry_msgs/Pose that should drive to

"""
class PointChooser(smach.State, Loggable):
    """
    Constructor.
    :param waypoints_name: str, name of waypoints as appears in the datacentre
    :param is_random: bool, should the waypoints be visited in random order
    :param n_iterations: int, how many times to visit all the waypoints
    """
    def __init__(self, waypoints_name, is_random, n_iterations):
        smach.State.__init__(self,
                             outcomes=['patrol',
                                       'go_charge',
                                       'succeeded'],
                             output_keys=['goal_pose', 'goal_pan_tilt']
                             )
        
        self.LOW_BATTERY = 35

        self.points = self._get_points(waypoints_name)  # []
	self.pantilts = self._get_pantilts(waypoints_name)
        self.point_set = waypoints_name
        self.charing_station_pos = [waypoints_name, "charging_point"]

        self.current_point = -1
        self.n_points = len(self.points)

        self.is_random = is_random
        self.n_iterations = n_iterations
        self.iterations_completed = 0

        # rearranges the list of points to visit randomly
        if self.is_random:
            shuffle(self.points)

        self.battery_life = 100
        self.battery_monitor = rospy.Subscriber(
            "/battery_state", BatteryState, self.bat_cb)

	# LOGGING
	mydir = os.path.join(os.getcwd(), datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
	os.makedirs(mydir)
	process_argument = "roslaunch strandsbag record.launch bag_folder:="+mydir
	print 'Creating new logging folder at the beginning of the patrol run ',mydir, ' process argument ',process_argument
	self.process = Popen(shlex.split(process_argument))
	#self.process.communicate()

    """ Get a list of points in the given set """
    def _get_points(self, point_set):
        mongo = pymongo.MongoClient(rospy.get_param("datacentre_host"),
                                    rospy.get_param("datacentre_port"))
        points = []
        search =  {"meta.pointset": point_set}
        for point in mongo.autonomous_patrolling.waypoints.find(search):
            print point
            if point["meta"]["name"] != "charging_point":
                points.append([point_set, point["meta"]["name"]])
        return points

    """ Get a list of pan_tilts in the given set """
    def _get_pantilts(self, point_set):
        mongo = pymongo.MongoClient(rospy.get_param("datacentre_host"),
                                    rospy.get_param("datacentre_port"))
        pts = []
        search =  {"meta.pantiltset": point_set}
        for pt in mongo.autonomous_patrolling.waypoints.find(search):
            print pt
            pts.append([point_set, pt["meta"]["name"]])

	print "Pan tilts ",pts
        return pts

    """ Get a given waypoint pose """
    def _get_point(self, point_name, point_set):
        mongo = pymongo.MongoClient(rospy.get_param("datacentre_host"),
                                    rospy.get_param("datacentre_port"))
        search = {"meta.name": point_name,
                  "meta.pointset": point_set}
        p = mongo.autonomous_patrolling.waypoints.find(search)            
        p = p[0]
        meta, pose = strands_datacentre.util.document_to_msg(p, Pose)
        return pose

    def _get_pantilt(self, point_name, point_set):
        mongo = pymongo.MongoClient(rospy.get_param("datacentre_host"),
                                    rospy.get_param("datacentre_port"))
	pt = PanTilt()
	pt.pan_increment = -1
	pt.tilt_increment = -1
	# first check whether PT is defined for this waypoint	
	pt_name = point_name+"_pan_tilt"
	
	try:
	   index = self.pantilts.index([point_set,pt_name])
           search = {"meta.name": pt_name,
                     "meta.pantiltset": point_set}
           p = mongo.autonomous_patrolling.waypoints.find(search)            
           p = p[0]
           meta, pt = strands_datacentre.util.document_to_msg(p, PanTilt)
	except ValueError:
	   # No pan tilt defined for this waypoint	
	   print "Pan tilt not defined for ", point_name
        return pt

    """ /battery_state subscription callback"""
    def bat_cb(self, msg):
        self.battery_life = msg.lifePercent

    def execute(self, userdata):
        rospy.sleep(1)

        if self.battery_life > self.LOW_BATTERY + 5:
		
            self.current_point = self.current_point + 1
	    # LOGGING
	    if self.current_point == 0:
		# START LOGGING
		mydir = os.path.join(os.getcwd(), datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
		#os.makedirs(mydir)
		process_argument = "roslaunch strandsbag record.launch bag_folder:="+mydir
		print 'Creating new logging folder at the beginning of the patrol run ',mydir, ' process argument ',process_argument
		#self.process = Popen(shlex.split(process_argument))

	    if self.current_point == self.n_points:
		print '---------------------Stopping logging---------------------------'
		self.process.terminate()

	    print 'now visiting point ID ',self.current_point
		
            if self.current_point == self.n_points:
		# STOP LOGGING SERVER WITH THE NEW FOLDER
		# CREATE NEW LOGGING FOLDER
		# START LOGGING SERVER WITH THE NEW FOLDER
                self.iterations_completed = self.iterations_completed + 1
                if self.iterations_completed == self.n_iterations:
                    return 'succeeded'
                self.current_point = 0
                if self.is_random:
                    shuffle(self.points)

            current_pt = self.points[self.current_point]
            userdata.goal_pose = self._get_point(current_pt[1], current_pt[0])
	    current_pan_tilt = self._get_pantilt(current_pt[1], current_pt[0])
	    userdata.goal_pan_tilt = current_pan_tilt 

	    print "Pan Tilt for this point ", current_pan_tilt

	    
            self.get_logger().log_waypoint_visit(current_pt[1])
            return 'patrol'
        else:
            userdata.goal_pose = self._get_point("charging_point",
                                                 self.point_set)
	    userdata.goal_pan_tilt = self._get_pantilt("charging_point",
                                                 self.point_set) 
 
            self.get_logger().log_waypoint_visit("charging_point")
            return 'go_charge'
        
    """ 
    Set the battery level thresholds.
    """
    def set_patroller_thresholds(self, very_low_battery, low_battery,
                               charged_battery,max_bumper_recovery_attempts,max_move_base_recovery_attempts):
        self.LOW_BATTERY = low_battery

"""
A top level state machine, that implements waypoint patrolling. Waypoints are
chosen by a PointChooser state. Navigation to the point then occurs, and another
point is selected. If the battery is too low charging is started, and once
charged, patrolling continues.

outcomes:	'succeeded': 	required no. iterations of patrol points achieved
			'aborted':		something went wrong
"""
class WaypointPatroller(smach.StateMachine, Loggable):
    """
    Constructor.
    :param waypoints_name: str, name of waypoints as appears in the datacentre
    :param is_random: bool, should the waypoints be visited in random order
    :param n_iterations: int, how many times to visit all the waypoints
    """
    def __init__(self, waypoints_name, is_random, n_iterations):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])
        
        self._waypoints_name = waypoints_name
        
        self._point_chooser =  PointChooser(waypoints_name,
                                            is_random,
                                            n_iterations)
        self._high_level_move_base_patrol =  navigation.HighLevelMoveBase(False)
        self._high_level_move_base_charge =  navigation.HighLevelMoveBase(True)
        self._dock_undock = charging.HighLevelDockUndockBehaviour()
        self._very_low_battery=VeryLowBattery()
	self._pan_tilt_state = PanTiltState()
        
        with self:
            smach.StateMachine.add('POINT_CHOOSER',
                                   self._point_chooser,
                                   transitions={'patrol': 'PATROL_POINT',
                                                'go_charge': 'GO_TO_CHARGING_STATION',
                                                'succeeded': 'succeeded'})
            smach.StateMachine.add('PATROL_POINT',
                                   #navigation.HighLevelMoveBase(),
                                   self._high_level_move_base_patrol, 
                                   transitions={'succeeded': 'PAN_TILT_POINT',
                                                'battery_low': 'POINT_CHOOSER',
                                                'move_base_failure': 'POINT_CHOOSER'})
            smach.StateMachine.add('PAN_TILT_POINT',
                                   #navigation.HighLevelMoveBase(),
                                   self._pan_tilt_state,
				   transitions={'succeeded':'POINT_CHOOSER', 
						'failure':'POINT_CHOOSER', 
						'not_defined':'POINT_CHOOSER'}) 

            smach.StateMachine.add('GO_TO_CHARGING_STATION',
                                   #navigation.HighLevelMoveBase(),
                                   self._high_level_move_base_charge, 
                                   transitions={'succeeded': 'DOCK_AND_CHARGE',
                                                'battery_low': 'VERY_LOW_BATTERY',
                                                'move_base_failure': 'POINT_CHOOSER'})
            
            smach.StateMachine.add('DOCK_AND_CHARGE',
                                   self._dock_undock,
                                   transitions={'succeeded': 'POINT_CHOOSER',
                                                'failure': 'POINT_CHOOSER'})
            
            smach.StateMachine.add('VERY_LOW_BATTERY',
                                   self._very_low_battery)

    """ 
    Set the patoller thresholds.
    """
    def set_patroller_thresholds(self, very_low_battery, low_battery,
                               charged_battery,max_bumper_recovery_attempts,max_move_base_recovery_attempts):
        self._point_chooser.set_patroller_thresholds(very_low_battery,
                                                   low_battery, 
                                                   charged_battery,max_bumper_recovery_attempts,max_move_base_recovery_attempts)
        self._high_level_move_base_patrol.set_patroller_thresholds(very_low_battery,
                                                          low_battery, 
                                                          charged_battery,max_bumper_recovery_attempts,max_move_base_recovery_attempts)
        self._high_level_move_base_charge.set_patroller_thresholds(very_low_battery,
                                                          low_battery, 
                                                          charged_battery,max_bumper_recovery_attempts,max_move_base_recovery_attempts)                                                          
        self._dock_undock.set_patroller_thresholds(very_low_battery,
                                                          low_battery, 
                                                          charged_battery,max_bumper_recovery_attempts,max_move_base_recovery_attempts)
        rospy.loginfo("Updating patroller thresholds::")
        rospy.loginfo("V.Low="+str(very_low_battery))
        rospy.loginfo("Low="+str(low_battery))
        rospy.loginfo("Charged="+str(charged_battery))
        rospy.loginfo("Max bumper recovers before sending e-mail="+str(max_bumper_recovery_attempts))
        rospy.loginfo("Max move_base recovers before going to a new point="+str(max_move_base_recovery_attempts))
   

    def execute(self):
        self.get_logger().log_start_episode(self._waypoints_name)
        outcome = smach.StateMachine.execute(self)
        self.get_logger().log_finish_episode()
        return outcome
