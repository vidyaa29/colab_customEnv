import gym
import cv2
import random
import numpy as np

import rospy
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger

import time
import threading
import math
import os

def flood_fill_single(im, seed_point):
    """Perform a single flood fill operation.

    # Arguments
        image: an image. the image should consist of white background, black lines and black fills.
               the white area is unfilled area, and the black area is filled area.
        seed_point: seed point for trapped-ball fill, a tuple (integer, integer).
    # Returns
        an image after filling.
    """
    pass1 = np.full(im.shape, 255, np.uint8)

    im_inv = cv2.bitwise_not(im)

    mask1 = cv2.copyMakeBorder(im_inv, 1, 1, 1, 1, cv2.BORDER_CONSTANT, 0)
    size, pass1, _, _ = cv2.floodFill(pass1, mask1, seed_point, 0, 0, 0, 4)

    return size , pass1

def max_flood(im):
	
	res=im.shape
	max=0
	img4=im
	for i in range(res[0]):
		for j in range(res[1]):
			if img4[i,j]==255:
				size, pass2 = flood_fill_single(im, (j,i) )
				
				img4=img4&pass2
				
				if(size > max):
					max=size
					img3=pass2
	return max , img3


class CustomEnv(gym.Env):
	def __init__(self):
		print('Environment initialized')

		#Start roscore
		roscore_t = threading.Thread( target=self.roscore_thread, args=() )
		roscore_t.start()

		#self.viewer = None
		#self.agent_spawn = True

		#Environment map parameters		
		self.size=(32,32)
		self.scaling_factor=20
		self.block_side=0.4	#1 Block side Length in meters
		self.agents_count=5

		def nothing(x):
			pass

		cv2.namedWindow('window')

		# create trackbars for obstacle_density
		cv2.createTrackbar('ObsDen','window',20,100,nothing)

		#Generating Random Map
		self.map_gen()
		#Find largest contour in map for future agents spawning and to move within
		max_size, self.img2 = max_flood(self.img)

		self.largest_contour=[]
		for i in range(self.size[0]):
			for j in range(self.size[1]):
				if(self.img2[i,j]==0):
					self.img[i,j]=127
					self.largest_contour.append((i,j))
		self.spawn_contour=self.largest_contour[:]	#Possible coordinates to spawn agents

		#Resizing image for diplay (init code for self.render() )
		self.resized_img=np.zeros((self.size[0]*self.scaling_factor,self.size[1]*self.scaling_factor, 3),dtype=np.uint8)
		#res = cv2.resize(self.img, (512, 512))

		for i in range(0,self.size[0]*self.scaling_factor,self.scaling_factor):
			for j in range(0,self.size[1]*self.scaling_factor,self.scaling_factor):
				shade=self.img[int(i/self.scaling_factor),int(j/self.scaling_factor)]
				self.resized_img[i:i+self.scaling_factor,j:j+self.scaling_factor]=(shade,shade,shade)

		#Initializing ROS Node
		rospy.init_node('PEPPER_Environment', anonymous=True)

		#Time Parameters
		#self.start_time=time.time()

		self._clock = Clock()
		self.pub_clock = rospy.Publisher('/clock', Clock, queue_size=10)
		#self.clock_rate = rospy.Rate(100) # 100hz

		self.clk_pub = threading.Thread( target=self.clock_pub, args=() )
		self.clk_pub.start()

		#Initilizing TF Publishers
		self.pub_tf_static = rospy.Publisher('/tf_static', TFMessage, queue_size=100)
		self.tf_static_rate = rospy.Rate(self.agents_count) # 1hz per agent
		self.pub_tf = rospy.Publisher('/tf', TFMessage, queue_size=1000)
		self.tf_rate = rospy.Rate(2*self.agents_count) # 2 hz per agent

		#CREATE AGENT OBJECTS & SPAWN THEM IN MAP
		self.agent=[]
		for id in range(self.agents_count):
			self.agent.append(agent(self,id))

		self.map_merge_t = threading.Thread( target=self.map_merge_thread, args=() )
		self.map_merge_t.start()

		
	def map_merge_thread(self):
		#Set params for map_merging
		for id in range(0,self.agents_count):
			os.system("rosparam set /Agent_"+str(id)+"/map_merge/init_pose_x 0.0")
			os.system("rosparam set /Agent_"+str(id)+"/map_merge/init_pose_y 0.0")
			os.system("rosparam set /Agent_"+str(id)+"/map_merge/init_pose_z 0.0")
			os.system("rosparam set /Agent_"+str(id)+"/map_merge/init_pose_yaw 0.0")

		origin_args=" x_origin:="+str(0.5*self.block_side*self.size[0])+" y_origin:="+str(0.5*self.block_side*self.size[0])
		os.system("roslaunch pepper env_map_merge_global.launch" + origin_args)
		return ()

	def roscore_thread(self):
		os.system("roscore")
		return ()

	def clock_pub(self):
		last_time=rospy.get_time() #Gets rostime in seconds as float
		while not rospy.is_shutdown():
			#clk=time.time()-self.start_time
			#self._clock.clock.secs=int(clk)
			#self._clock.clock.nsecs=int((clk-float(self._clock.clock.secs))*1000000000.0)
			#self.clock_rate.sleep()

			clk=rospy.get_rostime()
			self._clock.clock.secs=clk.secs
			self._clock.clock.nsecs=clk.nsecs

			while rospy.get_time() - last_time < 0.01:
				time.sleep(0.001)
			
			self.pub_clock.publish(self._clock)
			
			last_time=last_time+0.01

	def map_gen(self):

		obstacle_density = (cv2.getTrackbarPos('ObsDen','window'))/100.0

		#print(obstacle_density)

		self.img=np.ones(self.size,dtype=np.uint8)
		self.img=self.img*255

		x=random.sample(range(self.size[0]*self.size[1]),int(self.size[0]*self.size[1]*obstacle_density))

		for i in x:
			self.img[int(i/self.size[1]),i%self.size[1]]=0

	def step(self):
		return ()

	def render(self):
		output_img=self.resized_img.copy()
		#Show all agents
		for id in range(self.agents_count):
			agent_center = ( int( (self.agent[id].agent_pos[1]+0.5)*self.scaling_factor), int( (self.agent[id].agent_pos[0]+0.5)*self.scaling_factor) )
			output_img = cv2.circle(output_img, agent_center , int(self.scaling_factor*0.5) , self.agent[id].color , -1 ) 

		cv2.imshow('window',output_img)

	def reset(self):
		print('reset')

	def close(self):
		cv2.destroyAllWindows()
		print('success')

class agent():
	def __init__(self,_env,_agent_id):
		self.env=_env
		self.agent_id=_agent_id
		self.group_ns="Agent_"+str(_agent_id)

		#Spawn agent
		self.agent_pos1=random.sample(self.env.spawn_contour,1)	#Select a random position from the Largest Contour
		self.agent_pos=[self.agent_pos1[0][0],self.agent_pos1[0][1]]
		self.env.spawn_contour.remove( (self.agent_pos[0],self.agent_pos[1]) ) #Other agents cannot spawn in the same place
		self.color= ( random.randint(0,255),random.randint(0,255),random.randint(0,255) )

		#Start gmapping node
		gmapping_t = threading.Thread( target=self.gmapping_thread , args=() )
		gmapping_t.start()
		#time.sleep(2.0)

		#Initializing Odom
		self.agent_spawn_pos=self.agent_pos[:]

		self.pub_odom = rospy.Publisher(self.group_ns + "/odom", Odometry, queue_size=1000)
		#self.odom_rate = rospy.Rate(5) # 30hz

		self._odom = Odometry()
		self._odom.header.seq=0
		self._odom.header.frame_id = self.group_ns + "/odom"
		self._odom.child_frame_id = self.group_ns + "/base_link"
		
		self._odom.pose.pose.position.z=0.0		
		self._odom.pose.pose.orientation.x=0.0
		self._odom.pose.pose.orientation.y=0.0
		self._odom.pose.pose.orientation.z=-1.0/math.sqrt(2)	#We have assumed that the bot is always facing the +ve y axis in this code
		self._odom.pose.pose.orientation.w=-1.0/math.sqrt(2)
		self._odom.pose.covariance=[]
		for i in range(36):
			self._odom.pose.covariance.append(0.0)

		self._odom.twist.twist.linear.x=0
		self._odom.twist.twist.linear.y=0
		self._odom.twist.twist.linear.z=0
		self._odom.twist.twist.angular.x=0
		self._odom.twist.twist.angular.y=0
		self._odom.twist.twist.angular.z=0
		self._odom.twist.covariance=[]
		for i in range(36):
			self._odom.twist.covariance.append(0.0)

		#Laser Scan Parameters
		self.laser_scan_started=0
		self.bsh=self.env.block_side*0.5
		self.angle_steps=2  #Angle precesion of Laser Scanner in Degree

		self._scan = LaserScan()
		self.pub_scan = rospy.Publisher(self.group_ns + "/scan", LaserScan, queue_size=10000)
		self.laser_rate = rospy.Rate(30) # 30hz

		self.laser_id=0
		self._scan.header.frame_id=self.group_ns + "/base_scan"
		self._scan.angle_min=0.0
		self._scan.angle_max=6.283185307179586
		self._scan.angle_increment=0.017453292519943*self.angle_steps
		self._scan.time_increment=0.0
		self._scan.scan_time=0.0
		self._scan.range_min=self.bsh
		self._scan.range_max=5.5
		self._scan.intensities=[]
		for i in range(0,360,self.angle_steps):
			self._scan.intensities.append(0.0)
		self.Dr=self.env.block_side/10.0
		self.laser_gen_map=self.env.img.copy()

		self.laser_pub = threading.Thread( target=self.laser_gen, args=() )
		self.laser_pub.start()

		#Intitializing TF Parameters
		self._tf_static=TFMessage()
		self._tf=TFMessage()

		self._tf_bf_cl=TransformStamped()
		self._tf_bf_cl.header.seq=0
		self._tf_bf_cl.header.frame_id=self.group_ns + "/base_link"
		self._tf_bf_cl.child_frame_id=self.group_ns + "/base_scan"
		self._tf_bf_cl.transform.translation.x=0.0
		self._tf_bf_cl.transform.translation.y=0.0
		self._tf_bf_cl.transform.translation.z=0.0
		self._tf_bf_cl.transform.rotation.x=0.0
		self._tf_bf_cl.transform.rotation.y=0.0
		self._tf_bf_cl.transform.rotation.z=0.0
		self._tf_bf_cl.transform.rotation.w=1.0

		self._tf_odom_bf=TransformStamped()
		self._tf_odom_bf.header.seq=0
		self._tf_odom_bf.header.frame_id=self.group_ns + "/odom"
		self._tf_odom_bf.child_frame_id=self.group_ns + "/base_link"
		#self._tf_odom_bf.transform.translation.x=0.0
		#self._tf_odom_bf.transform.translation.y=0.0
		self._tf_odom_bf.transform.translation.z=0.0
		self._tf_odom_bf.transform.rotation.x=0.0
		self._tf_odom_bf.transform.rotation.y=0.0
		self._tf_odom_bf.transform.rotation.z=self._odom.pose.pose.orientation.z
		self._tf_odom_bf.transform.rotation.w=self._odom.pose.pose.orientation.w

		self.tf_static_pub_thread = threading.Thread( target=self.tf_static_pub , args=() )
		self.tf_pub_thread = threading.Thread( target=self.tf_pub , args=() )

		self.tf_static_pub_thread.start()
		
		time.sleep(2.0)
		self.map_merge_service_init()

		self.odom_update(self.agent_pos)

		self.tf_pub_thread.start()

	def gmapping_thread(self):
		id_arg=" agent_id:="+str(self.agent_id)
		n_arg=" n_agents:="+str(self.env.agents_count)
		map_size_args_min=" min_x:="+str(-0.5*self.env.block_side*self.env.size[0])+" min_y:="+str(-0.5*self.env.block_side*self.env.size[1])
		map_size_args_max=" max_x:="+str(1.5*self.env.block_side*self.env.size[0])+" max_y:="+str(1.5*self.env.block_side*self.env.size[1])

		os.system("roslaunch pepper gmapping_simple_sim.launch" + id_arg + n_arg + map_size_args_min + map_size_args_max)

		return ()

	def map_check(self,x,y):
		if x<self.env.size[0] and x>=0 and y<self.env.size[1] and y>=0:
			return self.laser_gen_map[int(x),int(y)]
		else:
			return (0)

	def tf_static_pub(self):
		while not rospy.is_shutdown():
			#self._tf_map_odom.header.stamp.secs=self._clock.clock.secs
			#self._tf_map_odom.header.stamp.nsecs=self._clock.clock.nsecs

			self._tf_bf_cl.header.stamp.secs=self.env._clock.clock.secs
			self._tf_bf_cl.header.stamp.nsecs=self.env._clock.clock.nsecs

			self._tf_static=[]
			#self._tf_static.append(self._tf_map_odom)
			self._tf_static.append(self._tf_bf_cl)

			self.env.pub_tf_static.publish(self._tf_static)
			self.env.tf_static_rate.sleep()
		return ()

	def tf_pub(self):
		while not rospy.is_shutdown():
			if self.odom_updating==False:
				self._odom.header.stamp.secs = self.env._clock.clock.secs
				self._odom.header.stamp.nsecs = self.env._clock.clock.nsecs

				self.pub_odom.publish(self._odom)

				self._odom.header.seq = self._odom.header.seq + 1

				self._tf_odom_bf.header.stamp.secs=self._odom.header.stamp.secs
				self._tf_odom_bf.header.stamp.nsecs=self._odom.header.stamp.nsecs

				self._tf=[]
				self._tf.append(self._tf_odom_bf)

				#self._tf_bf_cl.header.stamp.secs=self._odom.header.stamp.secs
				#self._tf_bf_cl.header.stamp.nsecs=self._odom.header.stamp.nsecs

				#self._tf.append(self._tf_bf_cl)

				self.env.pub_tf.publish(self._tf)
			
			self.env.tf_rate.sleep()
		return ()

	def odom_update(self, pos):
		
		self.odom_updating=True
		self._odom.header.stamp.secs = self.env._clock.clock.secs
		self._odom.header.stamp.nsecs = self.env._clock.clock.nsecs
		self.agent_pos=pos[:]

		self._odom.pose.pose.position.x=self.agent_pos[0]*self.env.block_side
		self._odom.pose.pose.position.y=self.agent_pos[1]*self.env.block_side
		#self.odom_rate.sleep()

		self.pub_odom.publish(self._odom)

		self._odom.header.seq = self._odom.header.seq + 1

		self._tf_odom_bf.header.stamp.secs=self._odom.header.stamp.secs
		self._tf_odom_bf.header.stamp.nsecs=self._odom.header.stamp.nsecs
		self._tf_odom_bf.transform.translation.x=self._odom.pose.pose.position.x
		self._tf_odom_bf.transform.translation.y=self._odom.pose.pose.position.y

		self._tf=[]
		self._tf.append(self._tf_odom_bf)

		#self._tf_bf_cl.header.stamp.secs=self._odom.header.stamp.secs
		#self._tf_bf_cl.header.stamp.nsecs=self._odom.header.stamp.nsecs

		#self._tf=[]
		#self._tf.append(self._tf_bf_cl)

		#self.enc.tf_rate.sleep()
		self.env.pub_tf.publish(self._tf)

		self.odom_updating=False

		self.laser_pub.join()
		self.laser_pub = threading.Thread( target=self.laser_gen, args=() )
		self.laser_pub.start()

		return ()

	def laser_gen(self):
		if not rospy.is_shutdown():
			self._scan.header.stamp.secs= self._odom.header.stamp.secs
			self._scan.header.stamp.nsecs= self._odom.header.stamp.nsecs
			_agent_pos=self.agent_pos[:]
			
			self._scan.header.seq=self.laser_id
			
			self._scan.ranges=[]
			for i in range(0,360,self.angle_steps):
				sn=np.sin(np.radians(i))
				cs=np.cos(np.radians(i))
				r=self._scan.range_min	#Minimum r value to start increment from in below while loop
				prev_DX=math.floor((self.bsh-r*sn)/self.env.block_side)
				prev_DY=math.floor((r*cs+self.bsh)/self.env.block_side)
				while r<self._scan.range_max:
					DX=math.floor((self.bsh-r*sn)/self.env.block_side)
					DY=math.floor((r*cs+self.bsh)/self.env.block_side)

					if self.map_check(_agent_pos[0]+DX,_agent_pos[1]+DY)==0:
						r=r-self.Dr
						for j in range(1,2):
							SDr=self.Dr/pow(10,j)
							for k in range(10):
								r=r+SDr
								DX=math.floor((self.bsh-r*sn)/self.env.block_side)
								DY=math.floor((r*cs+self.bsh)/self.env.block_side)

								if(self.map_check(_agent_pos[0]+DX,_agent_pos[1]+DY)==0):
									r=r-SDr
									break
						break

					elif prev_DX!=DX and prev_DY!=DY:
						if( self.map_check(_agent_pos[0]+prev_DX,_agent_pos[1]+DY)==0 or self.map_check(_agent_pos[0]+DX,_agent_pos[1]+prev_DY)==0 ):
							r=r-self.Dr
							for j in range(1,2):
								SDr=self.Dr/pow(10,j)
								for k in range(10):
									r=r+SDr
									DX=math.floor((self.bsh-r*sn)/self.env.block_side)
									DY=math.floor((r*cs+self.bsh)/self.env.block_side)

									if prev_DX!=DX and prev_DY!=DY:
										r=r-SDr
										break
							break

					r=r+self.Dr
					prev_DX=DX
					prev_DY=DY
				
				if r>= self._scan.range_max:
					r=float('inf')
				
				self._scan.ranges.append(r)

			if not rospy.is_shutdown():
				self.laser_rate.sleep()
				self.pub_scan.publish(self._scan)

			self.laser_id=self.laser_id+1
		return ()

	def map_merge_service_init(self):
		map_merge_srv_id="/Agent_" + str(self.agent_id) + "/merge_status"
		rospy.wait_for_service(map_merge_srv_id)
		self.map_merge_srv = rospy.ServiceProxy(map_merge_srv_id, Trigger)
		return ()

	def map_merge_service(self):
		srv_resp = self.map_merge_srv()
		if srv_resp.success==False:
			print("Agent_" + str(self.agent_id)+": " + srv_resp.message)
		return ()

	def step(self):
		print("Step: "+str(self.agent_id))

		for x in self.env.largest_contour:
			self.odom_update([x[0],x[1]])
			self.map_merge_service()
			self.env.render()
			if cv2.waitKey(1) & 0xFF == ord('x'):
				break

		return()
