import pybullet as p
import numpy as np
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.5f}".format(x)})

from parameter import *
import time
import pybullet_data
from functions import *
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2, PointField
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import struct
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

import threading

targetPosition = [3.5,2.9,0,0,0,0,0,0,0];
joint_trajectory=[];

pi = np.pi
near = 0.01
far = 1000
fov = 60

focal_length_x = 1/(math.tan((fov/180.0*pi)/2)*2/640)
focal_length_y = 1/(math.tan((fov/180.0*pi)/2)*2/480)
#print(focal_length_x)
#print(focal_length_y)
obastaclePose = [0.0,0.0,0.0,0.0,0.0,0.0,1];
def callback(data):

	for i in range(len(data.position)):
		targetPosition[i] = data.position[i]
	print(targetPosition)
def ostacleCallback(data):
	obastaclePose[0]= data.position.x;
	obastaclePose[1]= data.position.y;
	obastaclePose[2]= data.position.z;
	obastaclePose[3]= data.orientation.x;
	obastaclePose[4]= data.orientation.y;
	obastaclePose[5]= data.orientation.z;
	obastaclePose[6]= data.orientation.w;
	

	
def convert_depth_frame_to_pointcloud(depth_image):
	camera_intrinsics ={"fx":focal_length_x,"ppx": 320,"fy":focal_length_y,"ppy":240}
	[height, width] = depth_image.shape
	nx = np.linspace(0, width-1, width)
	ny = np.linspace(0, height-1, height)
	u, v = np.meshgrid(nx, ny)
	x = (u.flatten() - camera_intrinsics["ppx"])/camera_intrinsics["fx"]
	y = (v.flatten() - camera_intrinsics["ppy"])/camera_intrinsics["fy"]

	z = depth_image.flatten() / 1000.0;
	x = np.multiply(x,z)
	y = np.multiply(y,z)

	x = x[np.nonzero(z)]
	y = y[np.nonzero(z)]
	z = z[np.nonzero(z)]
	return x, y, z
def getCameraImage(cam_pos,cam_orn):

	aspect = 640/480
	angle = 0.0;
	q = p.getQuaternionFromEuler(cam_orn)
	cam_orn = np.reshape(p.getMatrixFromQuaternion(q ),(3,3));
	view_pos = np.matmul(cam_orn,np.array([0.001,0,0.0]).T)
	view_pos = np.array(view_pos+cam_pos)
	view_matrix = p.computeViewMatrix([cam_pos[0],cam_pos[1],cam_pos[2]], view_pos, [0,0,1])
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	images = p.getCameraImage(640,
					480,
					view_matrix,
					projection_matrix,
					shadow=False,
					renderer=p.ER_BULLET_HARDWARE_OPENGL)
	return images
def publishPointCloud(d435Id,d435Id2):
	global pub
	global pub_joint
	while 1:
		#print("pub_start")
		d435pos, d435orn = p.getBasePositionAndOrientation(d435Id)
		d435quat = d435orn
		d435orn =  p.getEulerFromQuaternion(d435orn)
		image = getCameraImage(d435pos,d435orn)
		depth_img = np.array(image[3],dtype=np.float)
		depth_img = far * near / (far - (far - near) * depth_img)
		color_img = image[2]
		color_img = np.reshape(color_img,[640*480,4])
		depth = np.transpose(np.array(convert_depth_frame_to_pointcloud(depth_img),dtype=np.float))
		points = []
		roll = 0;
		pitch = 0;
		yaw = 0;
		Rx = np.array([[1 ,0 ,0],[0, math.cos(roll), -math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
		Ry = np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
		Rz = np.array([[math.cos(yaw) ,-math.sin(yaw) ,0],[math.sin(yaw), math.cos(yaw), 0],[0 ,0,1]])
		R2 = np.matmul(np.matmul(Rx,Ry),Rz);
		R = np.eye(3);
		R = np.matmul(R,R2);
		T = np.array([0.0,0.0,0.0])
		for i in range(0,len(depth),8):
		    x = (R[0,0]*depth[i,0]*1000.0+R[0,1]*depth[i,1]*1000.0+R[0,2]*depth[i,2]*1000.0+T[0])
		    y = (R[1,0]*depth[i,0]*1000.0+R[1,1]*depth[i,1]*1000.0+R[1,2]*depth[i,2]*1000.0+T[1])
		    z = (R[2,0]*depth[i,0]*1000.0+R[2,1]*depth[i,1]*1000.0+R[2,2]*depth[i,2]*1000.0+T[2])
		    r = int(color_img[i,0])
		    g = int(color_img[i,1])
		    b = int(color_img[i,2])
		    a = 255
		    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
		    pt = [x, y, z, rgb]
		    points.append(pt)

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
			  PointField('y', 4, PointField.FLOAT32, 1),
			  PointField('z', 8, PointField.FLOAT32, 1),
			  # PointField('rgb', 12, PointField.UINT32, 1),
			  PointField('rgba', 12, PointField.UINT32, 1),
			  ]
		header = Header()
		header.frame_id = "map"
		pc2 = point_cloud2.create_cloud(header, fields, points)
		pc2.header.stamp = rospy.Time.now()

		pub.publish(pc2)
		js = JointState()
		js.name.append("joint0")
		js.name.append("joint1")
		js.name.append("joint2")
		js.name.append("joint3")
		js.name.append("joint4")
		js.name.append("joint5")
		for i in range(0,5):
			js.position.append(targetPosition[i])
		pub_joint.publish(js)
def getHomogeneousMatrix(Id):
	pos, orn = p.getBasePositionAndOrientation(Id)
	T = np.eye(4);
	R =  np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
	R = R
	T[0:3,0:3] = R
	T[0,3] = pos[0];
	T[1,3] = pos[1];
	T[2,3] = pos[2];
	return T

def getJointState(robotId,ArmJoint):
	jointState = p.getJointStates(robotId,ArmJoint)
	q = [jointState[0][0],jointState[1][0],jointState[2][0],jointState[3][0],jointState[4][0],jointState[5][0]]
	qdot =[jointState[0][1],jointState[1][1],jointState[2][1],jointState[3][1],jointState[4][1],jointState[5][1]]
	return q,qdot

def JointTrajectoryCallback(data):
	global indy7IdList
	joint_trajectory=[];
	for i in range(len(data.points)): 
		jt=joint_trajectory.append(data.points[i].positions)
		print(jt)
		for j in range(1,6):
			p.resetJointState(indy7IdList[i], j, jt[j])
	print(len(joint_trajectory))
		
def main():
	global pub
	global pub_joint

	p.connect(p.GUI)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	useRealTimeSim = False
	p.setRealTimeSimulation(useRealTimeSim)
	p.setTimeStep(1/240.0)

	robotPATH = "urdf/mm_hyu/right_sim.urdf"
	indy7robotPATH = "urdf/indy7/indy7.urdf"
	#obstaclePATH = "urdf/obstacle/obstacle.urdf"
	
	p.setGravity(0, 0, -9.8)
	tableId=p.loadURDF("./urdf/shelfandtable/shelfandtable.urdf", [0, 0, 0.0])


	indy7robotId = p.loadURDF(indy7robotPATH,[0.0,0,0.0], p.getQuaternionFromEuler([0,0,0]))
	#obstacleId = p.loadURDF(obstaclePATH,[0,0,0],p.getQuaternionFromEuler([0,0,0]))
	d435Id = p.loadURDF("./urdf/d435/d435.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(d435Id, [1.5 ,-0.5, 1.2],p.getQuaternionFromEuler([0,pi/4,pi-pi/8]))


	MobileJoint = [0,1,2]
	ArmJoint = [12,13,14,15,16,17]
	FTsensor = [11]
	x=1.0;
	y=0;
	wz = 0;

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/joint_smc", JointState, callback)
	rospy.Subscriber("/obstacle_pose", Pose, ostacleCallback)
	rospy.Subscriber("/joint_trajectory", JointTrajectory, JointTrajectoryCallback)



	pub = rospy.Publisher("/camera/depth/color/points", PointCloud2, queue_size=2)
	pub_joint = rospy.Publisher("/joint_smc2", JointState, queue_size=2)
	
	t = threading.Thread(target=publishPointCloud, args=(d435Id,d435Id))
	t.start()
	print(getHomogeneousMatrix(d435Id));
	rate=rospy.Rate(10);
	x_Id = p.addUserDebugParameter("x", 0, 7, 0.663)
	y_Id = p.addUserDebugParameter("y", 0, 5, 0.0)
	z_Id = p.addUserDebugParameter("z", 0, 3, 0.263)

	j0_Id = p.addUserDebugParameter("j0", -3.05432619099, 3.05432619099, 0.0)
	j1_Id = p.addUserDebugParameter("j1", -3.05432619099, 3.05432619099, -15.0*3.141592/180)
	j2_Id = p.addUserDebugParameter("j2", -3.05432619099, 3.05432619099, -90.0*3.141592/180)
	j3_Id = p.addUserDebugParameter("j3", -3.05432619099, 3.05432619099, 0)
	j4_Id = p.addUserDebugParameter("j4", -3.05432619099, 3.05432619099, -75.0*3.141592/180)
	j5_Id = p.addUserDebugParameter("j5", -3.14, 3.14, 0.263)
	joint_trajectory=[];
	indy7IdList=[];
	global indy7IdList
	for i in range(0,30):
		indy7IdList.append(p.loadURDF(indy7robotPATH,[0.0,0,0.0], p.getQuaternionFromEuler([0,0,0])))

	while(1):
		rate.sleep()
		


if __name__ == "__main__":
    # execute only if run as a script
    main()
