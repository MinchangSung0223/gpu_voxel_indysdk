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
from trajectory_msgs.msg import JointTrajectory
import math
import struct
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

import threading
joint_trajectory=[[-0.269328,-0.799814,-0.999361,-0.0167498,-1.36748,-0.271398],
[-0.269328,-0.799815,-0.999363,-0.0167498,-1.36748,-0.271398],
[-0.269328,-0.799819,-0.999375,-0.0167498,-1.36746,-0.271398],
[-0.269328,-0.799831,-0.999407,-0.0167498,-1.36742,-0.271398],
[-0.269328,-0.799854,-0.999468,-0.0167498,-1.36734,-0.271398],
[-0.269328,-0.79989,-0.999567,-0.0167498,-1.3672,-0.271398],
[-0.269328,-0.799944,-0.999711,-0.0167498,-1.367,-0.271398],
[-0.269328,-0.800017,-0.999908,-0.0167498,-1.36673,-0.271398],
[-0.269328,-0.800112,-1.00016,-0.0167498,-1.36638,-0.271397],
[-0.269328,-0.800231,-1.00049,-0.0167498,-1.36594,-0.271397],
[-0.269328,-0.800377,-1.00088,-0.0167499,-1.3654,-0.271397],
[-0.269328,-0.800551,-1.00135,-0.0167499,-1.36476,-0.271397],
[-0.269328,-0.800755,-1.0019,-0.0167499,-1.364,-0.271397],
[-0.269328,-0.800991,-1.00254,-0.01675,-1.36313,-0.271397],
[-0.269328,-0.801259,-1.00326,-0.01675,-1.36214,-0.271397],
[-0.269328,-0.801562,-1.00408,-0.0167501,-1.36102,-0.271397],
[-0.269328,-0.8019,-1.00499,-0.0167501,-1.35977,-0.271396],
[-0.269328,-0.802273,-1.006,-0.0167502,-1.35838,-0.271396],
[-0.269328,-0.802683,-1.00711,-0.0167503,-1.35687,-0.271396],
[-0.269328,-0.80313,-1.00832,-0.0167504,-1.35521,-0.271395],
[-0.269328,-0.803615,-1.00962,-0.0167505,-1.35342,-0.271395],
[-0.269328,-0.804137,-1.01103,-0.0167506,-1.35149,-0.271395],
[-0.269328,-0.804696,-1.01254,-0.0167507,-1.34942,-0.271394],
[-0.269328,-0.805293,-1.01416,-0.0167508,-1.34721,-0.271394],
[-0.269328,-0.805928,-1.01587,-0.0167509,-1.34486,-0.271394],
[-0.269328,-0.806599,-1.01768,-0.016751,-1.34238,-0.271393],
[-0.269328,-0.807307,-1.01959,-0.0167512,-1.33976,-0.271393],
[-0.269328,-0.808051,-1.0216,-0.0167513,-1.337,-0.271392],
[-0.269328,-0.80883,-1.02371,-0.0167514,-1.33412,-0.271392],
[-0.269328,-0.809644,-1.0259,-0.0167516,-1.33111,-0.271391],
[-0.269328,-0.810491,-1.02819,-0.0167517,-1.32798,-0.271391],
[-0.269328,-0.811371,-1.03057,-0.0167519,-1.32472,-0.27139],
[-0.269328,-0.812282,-1.03303,-0.0167521,-1.32135,-0.271389],
[-0.269328,-0.813224,-1.03557,-0.0167523,-1.31787,-0.271389],
[-0.269328,-0.814194,-1.03819,-0.0167524,-1.31427,-0.271388],
[-0.269328,-0.815192,-1.04088,-0.0167526,-1.31058,-0.271387],
[-0.269328,-0.816216,-1.04365,-0.0167528,-1.30679,-0.271387],
[-0.269328,-0.817265,-1.04648,-0.016753,-1.30291,-0.271386],
[-0.269328,-0.818337,-1.04938,-0.0167532,-1.29894,-0.271385],
[-0.269328,-0.819431,-1.05233,-0.0167534,-1.2949,-0.271385],
[-0.269328,-0.820544,-1.05534,-0.0167536,-1.29078,-0.271384],
[-0.269328,-0.821675,-1.05839,-0.0167538,-1.28659,-0.271383],
[-0.269328,-0.822823,-1.06149,-0.016754,-1.28235,-0.271382],
[-0.269523,-0.826763,-1.05904,-0.0163914,-1.281,-0.271146],
[-0.269524,-0.82819,-1.06172,-0.0164116,-1.2769,-0.271076],
[-0.269524,-0.829647,-1.06439,-0.0164323,-1.27277,-0.271005],
[-0.269524,-0.831131,-1.06706,-0.0164534,-1.26862,-0.270934],
[-0.269524,-0.83264,-1.06971,-0.0164749,-1.26446,-0.270862],
[-0.269524,-0.834171,-1.07234,-0.0164967,-1.26029,-0.270791],
[-0.269524,-0.835723,-1.07495,-0.0165189,-1.25613,-0.270718],
[-0.269524,-0.837293,-1.07754,-0.0165414,-1.25198,-0.270646],
[-0.269525,-0.838877,-1.08009,-0.0165642,-1.24784,-0.270574],
[-0.269525,-0.840474,-1.08261,-0.0165872,-1.24373,-0.270502],
[-0.269525,-0.842079,-1.08509,-0.0166103,-1.23964,-0.270431],
[-0.269525,-0.843692,-1.08753,-0.0166336,-1.23559,-0.27036],
[-0.269525,-0.845307,-1.08992,-0.016657,-1.23158,-0.270289],
[-0.269525,-0.846923,-1.09227,-0.0166805,-1.22762,-0.270219],
[-0.269525,-0.848536,-1.09456,-0.0167039,-1.22371,-0.27015],
[-0.269526,-0.850143,-1.0968,-0.0167274,-1.21987,-0.270081],
[-0.269526,-0.85174,-1.09899,-0.0167507,-1.21608,-0.270014],
[-0.269526,-0.853325,-1.10112,-0.0167739,-1.21237,-0.269947],
[-0.269526,-0.854895,-1.10318,-0.0167969,-1.20874,-0.269882],
[-0.269526,-0.856445,-1.10519,-0.0168197,-1.20518,-0.269818],
[-0.269526,-0.857974,-1.10713,-0.0168422,-1.20171,-0.269755],
[-0.269527,-0.859477,-1.10901,-0.0168644,-1.19833,-0.269694],
[-0.269527,-0.860952,-1.11082,-0.0168862,-1.19505,-0.269634],
[-0.269527,-0.862396,-1.11256,-0.0169076,-1.19186,-0.269576],
[-0.269527,-0.863806,-1.11424,-0.0169286,-1.18877,-0.26952],
[-0.269527,-0.865179,-1.11584,-0.016949,-1.1858,-0.269465],
[-0.269527,-0.866512,-1.11738,-0.0169689,-1.18293,-0.269412],
[-0.269528,-0.867804,-1.11885,-0.0169882,-1.18017,-0.269362],
[-0.269528,-0.86905,-1.12024,-0.0170068,-1.17752,-0.269313],
[-0.269528,-0.87025,-1.12157,-0.0170248,-1.175,-0.269266],
[-0.269528,-0.871401,-1.12283,-0.0170421,-1.17259,-0.269221],
[-0.269528,-0.872502,-1.12402,-0.0170586,-1.1703,-0.269179],
[-0.269528,-0.87355,-1.12513,-0.0170744,-1.16813,-0.269138],
[-0.269528,-0.874544,-1.12618,-0.0170894,-1.16609,-0.2691],
[-0.269528,-0.875484,-1.12716,-0.0171036,-1.16417,-0.269064],
[-0.269529,-0.876367,-1.12808,-0.0171169,-1.16237,-0.26903],
[-0.269529,-0.877194,-1.12893,-0.0171294,-1.1607,-0.268999],
[-0.269529,-0.877963,-1.12971,-0.0171411,-1.15915,-0.26897],
[-0.269529,-0.878676,-1.13043,-0.0171519,-1.15772,-0.268943],
[-0.269529,-0.87933,-1.13109,-0.0171619,-1.1564,-0.268918],
[-0.269529,-0.879928,-1.13168,-0.0171709,-1.15521,-0.268895],
[-0.269529,-0.88047,-1.13222,-0.0171792,-1.15413,-0.268875],
[-0.269529,-0.880956,-1.1327,-0.0171866,-1.15317,-0.268857],
[-0.269529,-0.881388,-1.13312,-0.0171932,-1.15231,-0.26884],
[-0.269529,-0.881768,-1.13349,-0.017199,-1.15156,-0.268826],
[-0.269529,-0.882098,-1.13381,-0.017204,-1.15091,-0.268814],
[-0.269529,-0.882379,-1.13408,-0.0172083,-1.15036,-0.268803],
[-0.269529,-0.882615,-1.13431,-0.0172119,-1.14989,-0.268794],
[-0.269529,-0.882808,-1.1345,-0.0172148,-1.14951,-0.268787],
[-0.269529,-0.882962,-1.13465,-0.0172172,-1.14921,-0.268781],
[-0.269529,-0.88308,-1.13476,-0.017219,-1.14898,-0.268777],
[-0.269529,-0.883167,-1.13484,-0.0172203,-1.14881,-0.268774],
[-0.269529,-0.883226,-1.1349,-0.0172212,-1.14869,-0.268772],
[-0.269529,-0.883263,-1.13494,-0.0172218,-1.14862,-0.26877],
[-0.269529,-0.883282,-1.13495,-0.0172221,-1.14858,-0.26877],
[-0.269529,-0.883289,-1.13496,-0.0172222,-1.14857,-0.268769],
[-0.269529,-0.88329,-1.13496,-0.0172222,-1.14857,-0.268769]
]
targetPosition = [-0.32724917, -0.66689017, -1.08646723, -0.00715585, -1.29468497,       -0.33440502];
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
def jointTrajectoryCallback(data):
	
	for i in range(0,len(data.points)):
		joint_temp= data.points[i].positions
		joint_trajectory.append(joint_temp)
	print(joint_trajectory)
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
	obstaclePATH = "urdf/obstacle/obstacle.urdf"
	
	p.setGravity(0, 0, -9.8)
	tableId=p.loadURDF("./urdf/shelfandtable/shelfandtable.urdf", [0, 0, 0.0])


	indy7robotId = p.loadURDF(indy7robotPATH,[0.0,0,0.0], p.getQuaternionFromEuler([0,0,0]))
	obstacleId = p.loadURDF(obstaclePATH,[0,0,0],p.getQuaternionFromEuler([0,0,0]))
	d435Id = p.loadURDF("./urdf/d435/d435.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(d435Id, [0 ,0.85, 0.86],p.getQuaternionFromEuler([0,pi/6+pi/64,-pi/4-pi/15]))


	MobileJoint = [0,1,2]
	ArmJoint = [12,13,14,15,16,17]
	FTsensor = [11]
	x=1.0;
	y=0;
	wz = 0;

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/joint_states_desired", JointState, callback)
	rospy.Subscriber("/obstacle_pose", Pose, ostacleCallback)



	pub = rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=2)
	pub_joint = rospy.Publisher("/joint_smc", JointState, queue_size=2)
	
	t = threading.Thread(target=publishPointCloud, args=(d435Id,d435Id))
	t.start()
	print(getHomogeneousMatrix(d435Id));
	rate=rospy.Rate(10);
	x_Id = p.addUserDebugParameter("x", 0, 7, 0.553)
	y_Id = p.addUserDebugParameter("y", -5, 5, -0.105)
	z_Id = p.addUserDebugParameter("z", -5, 5, 0.263)

	j0_Id = p.addUserDebugParameter("j0", -3.05432619099, 3.05432619099, 0.0)
	j1_Id = p.addUserDebugParameter("j1", -3.05432619099, 3.05432619099, -15.0*3.141592/180)
	j2_Id = p.addUserDebugParameter("j2", -3.05432619099, 3.05432619099, -90.0*3.141592/180)
	j3_Id = p.addUserDebugParameter("j3", -3.05432619099, 3.05432619099, 0)
	j4_Id = p.addUserDebugParameter("j4", -3.05432619099, 3.05432619099, -75.0*3.141592/180)
	j5_Id = p.addUserDebugParameter("j5", -3.14, 3.14, 0.263)


	
	for i in range(0,len(joint_trajectory)):
		obastaclePose[0] = p.readUserDebugParameter(x_Id)
		obastaclePose[1] = p.readUserDebugParameter(y_Id)
		obastaclePose[2] = p.readUserDebugParameter(z_Id)	

		p.resetBasePositionAndOrientation(obstacleId, [obastaclePose[0],obastaclePose[1],obastaclePose[2]], 
		[obastaclePose[3],obastaclePose[4],obastaclePose[5],obastaclePose[6]])	
		
		print(joint_trajectory[i])
		for j in range(1,6):
			p.resetJointState(indy7robotId, j, joint_trajectory[i][j-1])

		time.sleep(0.1);


if __name__ == "__main__":
    # execute only if run as a script
    main()
