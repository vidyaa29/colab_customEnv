#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
import math
import numpy as np
import copy
import time
import threading
from std_srvs.srv import Trigger

merged_map=OccupancyGrid()
map_list=[]
_started=False
map_object=[]
merge_status=[]
call_from_Env=False

def return_merge_status(req):
    global call_from_Env
    global _bot_count
    global merge_status

    call_from_Env=True

    _time_goal=rospy.Time.now() + rospy.Duration(2.0) #Setting 2 seconds as timeout
    while rospy.Time.now() < _time_goal:
        if call_from_Env==False:
            break
        time.sleep(0.001)

    if call_from_Env==False:
        for i in range(_bot_count):
            merge_status[i]=0
        return {'success': True , 'message': "success"}
    else:
        #for i in range(_bot_count):
        #    print("Agent_merge_"+str(i)+" : "+str(merge_status[i]) )
        call_from_Env=False
        for i in range(_bot_count):
                    print("Agent_merge_"+str(i)+" : "+str(merge_status[i]) )
        for i in range(_bot_count):
            merge_status[i]=0
        return {'success': False , 'message': "timedout"}

def local_map_sub(_local_map):
    global _bot_id
    global _bot_count
    global map_list
    global map_object
    global _started
    global pub
    global _base_link_frame
    global mbot_frame
    global merged_map
    global map_length
    global merge_status
    global call_from_Env

    if _started==False:

        merged_map.header.seq=0
        merged_map.header.frame_id="map"

        merged_map.info=copy.deepcopy(_local_map.info)

        map_length = merged_map.info.width*merged_map.info.height
        merged_map.data = [-1]*map_length

        for id in range(_bot_count):
            if id==_bot_id:
                map_list.append(_local_map.data[:])
                merge_status.append(2 if call_from_Env else 0)
                continue
            map_list.append([-1]*map_length)
            merge_status.append(0)
            map_object.append(map_Subscriber(id))

        _started=True

    else:
        map_list[_bot_id]=_local_map.data[:]
        if call_from_Env:
            merge_status[_bot_id]=2
    return ()

def pub_map():
    global map_list
    global pub
    global _bot_count
    global _started
    global pub_rate
    global map_length
    global call_from_Env
    global merge_status

    sub_check=False
    while _started==False:
        time.sleep(0.1)

    while not rospy.is_shutdown():

        map_time=rospy.get_rostime()
        merged_map.header.stamp.secs=map_time.secs
        merged_map.header.stamp.nsecs=map_time.nsecs
        
        #Map merging loop
        for i in range(map_length):
            max_value=map_list[0][i]
            for j in range(1,_bot_count):
                if map_list[j][i]>max_value:
                    max_value=map_list[j][i]
            merged_map.data[i]=max_value

        pub.publish(merged_map)

        if sub_check:
            call_from_Env=False
            sub_check=False

        #pub_rate.sleep()

        if call_from_Env:
            sub_check=True
            for i in range(_bot_count):
                if merge_status[i]==0:
                    sub_check=False
                    break

        merged_map.header.seq = merged_map.header.seq + 1

    return ()

class map_Subscriber():
    def __init__(self,_agent_id):
        global _bot_ns
        global _odom_frame
        global mbot_frame
        global mbot_origin
        global _bot_count

        self.agent_id=_agent_id

        self.listener = tf.TransformListener()

        self.sbot_frame= "/" + _bot_ns + str(self.agent_id) + "/" + _base_link_frame
        self.sbot_origin= "/" + _bot_ns + str(self.agent_id) + "/" + _odom_frame

        self.listener.waitForTransform(mbot_frame, mbot_origin, rospy.Time(0) , rospy.Duration( (_bot_count+1)*7 ) , polling_sleep_duration = rospy.Duration(0.01))
        self.listener.waitForTransform(self.sbot_frame, self.sbot_origin, rospy.Time(0) , rospy.Duration( (_bot_count+1)*7 ) , polling_sleep_duration = rospy.Duration(0.01))

        rospy.Subscriber("/"+_bot_ns+str(self.agent_id)+"/map", OccupancyGrid, self.callback)

    def callback(self,_map):
        global map_list
        global _base_link_frame
        global mbot_frame
        global _bot_id
        global _bot_ns
        global _max_comm_dist
        global mbot_origin
        global call_from_Env
        global merge_status

        try:
            (trans_s,rot) = self.listener.lookupTransform(mbot_frame, mbot_origin, rospy.Time(0) )
            (trans_m,rot) = self.listener.lookupTransform(self.sbot_frame, self.sbot_origin, rospy.Time(0) )
            #distance=math.sqrt((trans[0]**2)+(trans[1]**2))
            distance=math.sqrt( ( (trans_s[0]-trans_m[0])**2)+( (trans_s[1]-trans_m[1])**2) )
            #print("Agent_"+str(_bot_id)+"-"+str(self.agent_id)+" :"+str(distance) )

            if distance <= _max_comm_dist:
                map_list[self.agent_id]=_map.data[:]

            if call_from_Env:
                if merge_status[self.agent_id]==0:
                    merge_status[self.agent_id]=1
                elif merge_status[self.agent_id]==1:
                    merge_status[self.agent_id]=2

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return ()

if __name__ == '__main__':
    global _bot_id
    global _bot_count
    global _max_comm_dist
    global _bot_ns
    global _base_link_frame
    global mbot_frame
    global pub
    global pub_rate
    global _max_comm_dist
    global mbot_origin
    global _odom_frame
    
    try:
        rospy.init_node('local_map_transmitter', anonymous=True)

        _bot_id = rospy.get_param('~bot_id', 0)
        _bot_count = rospy.get_param('~bot_count', 1)
        _max_comm_dist = rospy.get_param('~max_comm_dist', 3.0)
        _bot_ns = rospy.get_param('~bot_ns', "Agent_")
        _base_link_frame = rospy.get_param('~base_link_frame', "base_link")
        _odom_frame = rospy.get_param('~odom_frame', "odom")

        mbot_frame= "/" + _bot_ns + str(_bot_id) + "/" + _base_link_frame
        mbot_origin= "/" + _bot_ns + str(_bot_id) + "/" +  _odom_frame

        pub=rospy.Publisher('map', OccupancyGrid , queue_size=10)
        
        rospy.Subscriber('local_map', OccupancyGrid, local_map_sub)

        pub_rate=rospy.Rate(10)
        pub_thread = threading.Thread( target=pub_map, args=() )
        pub_thread.start()

        #Init Merge Status Feedback Service for the Environment to call
        service = rospy.Service("/" + _bot_ns + str(_bot_id) + "/merge_status", Trigger, return_merge_status)

        rospy.spin()
        service.shutdown('Program Terminated')

    except rospy.ROSInterruptException:
        service.shutdown('ROS Interrupt Shutdown request received')
        pass    
