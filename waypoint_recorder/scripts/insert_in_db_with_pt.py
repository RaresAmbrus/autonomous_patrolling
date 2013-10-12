#!/usr/bin/env python
#import roslib; roslib.load_manifest("waypoint_visualiser")
import rospy

from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Pose

import pymongo
import csv

import strands_datacentre.util

def way_points_file_to_datacentre(filename, dataset_name, map_name):
    host = rospy.get_param("datacentre_host")
    port = rospy.get_param("datacentre_port")
    print "Using datacentre  ",host,":", port
    client = pymongo.MongoClient(host, port)
    db=client.autonomous_patrolling
    points_db=db["waypoints"]
    
    points=[]
    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            current_row=[]
            for element in row:
                current_row.append(float(element))
            points.append(current_row)

    for i,point in enumerate(points):
        entry={}
        entry["map"]=map_name
        if i==0:
            entry["name"]="charging_point"
        else:
            entry["name"]="Point%d"%i
        entry["pointset"]=dataset_name

        p = Pose()
        p.position.x=point[0]
        p.position.y=point[1]
        p.position.z=point[2]
        p.orientation.x=point[3]
        p.orientation.y=point[4]
        p.orientation.z=point[5]
        p.orientation.w=point[6]
        strands_datacentre.util.store_message(points_db,p,entry)

	if (len(point) > 12):
		entry["name"]="Point%d_pan_start"%i
		pan_start = point[7]	
        	strands_datacentre.util.store_message(points_db,pan_start,entry)

		entry["name"]="Point%d_pan_increment"%i
		pan_inc = point[8]	
        	strands_datacentre.util.store_message(points_db,pan_inc,entry)

		entry["name"]="Point%d_pan_stop"%i
		pan_stop = point[9]	
        	strands_datacentre.util.store_message(points_db,pan_stop,entry)

		entry["name"]="Point%d_tilt_start"%i
		tilt_start = point[10]	
        	strands_datacentre.util.store_message(points_db,tilt_start,entry)
		
		entry["name"]="Point%d_tilt_increment"%i
		tilt_inc = point[11]	
        	strands_datacentre.util.store_message(points_db,tilt_inc,entry)

		entry["name"]="Point%d_tilt_stop"%i
		tilt_stop = point[12]	
        	strands_datacentre.util.store_message(points_db,tilt_stop,entry)

if __name__=="__main__":
    if len(sys.argv)!=4:
        print "Usage: rosrun waypoint_recorder 'csv_file' 'dataset_name' 'map_name'"
        sys.exit(1)
    way_points_file_to_datacentre(sys.argv[1],sys.argv[2],sys.argv[3])
