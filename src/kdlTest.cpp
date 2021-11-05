#include <stdio.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include <kdl/frames_io.hpp>
#include <jsoncpp/json/json.h>
#include "CppIkSolver/solver.h"
#include "../include/modern_robotics.h"


#pragma comment(lib, "jsoncpp.lib")

#define JOINTNUM 6

typedef std::vector<std::array<double,JOINTNUM>> JT;
typedef std::array<double,JOINTNUM> J;

using namespace std;
using namespace mr;
bool ReadFromFile(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
void printCatesianKDLFrame(KDL::Frame frame,char* str ){
    std::cout<<"======="<<str<<"=======\n\n"<<endl;
    for(int i =0;i<4;i++){
        for(int j=0;j<4;j++)
            std::cout<<frame(i,j)<<"\t";
        std::cout<<"\n"<<std::endl;
    }
}

JT smcCartesianTrajectory(KDL::JntArray q_start,KDL::JntArray q_min,KDL::JntArray q_max, float x,float y ,float z, double Tf, int N, int method,KDL::Chain my_chain) {
    JT joint_trajectory;
    joint_trajectory.clear();
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    KDL::Frame kdl_Xstart;
    fk_solver.JntToCart(q_start, kdl_Xstart);
    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);    
    KDL::ChainIkSolverPos_NR_JL iksolver1(my_chain,q_min,q_max,fk_solver,iksolver1v,2000,0.001);

    Eigen::MatrixXd Xstart(4,4);
    for(int i =0;i<4;i++)
        for(int j=0;j<4;j++)
            Xstart(i,j) = kdl_Xstart(i,j);
    Eigen::MatrixXd Xend(4,4);        
    for(int i =0;i<4;i++)
        for(int j=0;j<4;j++)
            Xend(i,j) = kdl_Xstart(i,j);   
    Xend(0,3) = Xend(0,3) +x;
    Xend(1,3) = Xend(1,3) +y;
    Xend(2,3) = Xend(2,3) +z;
    
    std::cout <<kdl_Xstart<<std::endl;
    std::cout <<Xstart<<std::endl;
    std::cout <<Xend<<std::endl;
    
    
    std::vector<Eigen::MatrixXd> Task_traj =  CartesianTrajectory(Xstart,Xend, Tf, N, method);
    Eigen::MatrixXd tempMatrix(4,4);
    KDL::JntArray prev_joint(JOINTNUM);
    
    prev_joint(0) = q_start(0);
    prev_joint(1) = q_start(1);
    prev_joint(2) = q_start(2);
    prev_joint(3) = q_start(3);
    prev_joint(4) = q_start(4);
    prev_joint(5) = q_start(5);
    
    for(int i =0 ;i<Task_traj.size();i++){
        KDL::JntArray q_result(JOINTNUM);
        J j_temp;

        tempMatrix = Task_traj.at(i);        
        KDL::Frame goal_pose(KDL::Rotation(tempMatrix(0,0),tempMatrix(1,0),tempMatrix(2,0),
            tempMatrix(0,1),tempMatrix(1,1),tempMatrix(2,1),
            tempMatrix(0,2),tempMatrix(1,2),tempMatrix(2,2)),KDL::Vector(tempMatrix(0,3),tempMatrix(1,3),tempMatrix(2,3)));
        bool ret = iksolver1.CartToJnt(prev_joint,goal_pose,q_result);
        if(ret>=0){
            for(int j = 0;j<JOINTNUM;j++)
                j_temp.at(j) = q_result(j);
            joint_trajectory.push_back(j_temp);
        }
    }
    return joint_trajectory;
}


int  main(int argc, char **argv){

KDL::Tree my_tree;
KDL::Chain my_chain;

cout<<argv[1]<<endl;

//load Json file name
  const char* JSON_FILE= argv[1];
//load robot name
  std::string robot_name = argv[2];

  const int BufferLength = 102400;
  char readBuffer[BufferLength] = {0,};
  if (false == ReadFromFile(JSON_FILE, readBuffer, BufferLength)) 
      return 0;
  std::string config_doc = readBuffer;
  Json::Value rootr;
  Json::Reader reader;
  bool parsingSuccessful = reader.parse(config_doc,rootr);
  if ( !parsingSuccessful ) { 
    std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
    return 0; 
  }

  std::string base_link_name =rootr[robot_name]["base_link_name"].asString();
  std::string tcp_link_name =rootr[robot_name]["tcp_link_name"].asString();

  int jointnum = rootr[robot_name]["JOINTNUM"].asInt();


  KDL::JntArray q_min(jointnum);
  KDL::JntArray q_max(jointnum);

  for(int j = 0;j<jointnum;j++){
      q_min(j) = rootr[robot_name]["lower_limit"][j].asFloat();
      q_max(j) = rootr[robot_name]["upper_limit"][j].asFloat();         
  }

  float target[7];
  target[0]=rootr[robot_name]["target"][0].asFloat();
  target[1]=rootr[robot_name]["target"][1].asFloat();
  target[2]=rootr[robot_name]["target"][2].asFloat();
  target[3]=rootr[robot_name]["target"][3].asFloat();
  target[4]=rootr[robot_name]["target"][4].asFloat();
  target[5]=rootr[robot_name]["target"][5].asFloat();
  target[6]=rootr[robot_name]["target"][6].asFloat();
  
   std::cout<<"target: "<<target[0]<<","<<target[1]<<","<<target[2]<<","<<target[3]<<"\n"<<target[4]<<","<<target[5]<<","<<target[6]<<endl;
	std::string urdf_name = rootr[robot_name]["urdf_location"].asString();
	if (!kdl_parser::treeFromFile(urdf_name, my_tree)){
            // LOGGING_INFO(Gpu_voxels,"Failed to construct kdl tree");
		 std::cout<<"Failed to construct kdl tree"<<std::endl;
    }
    std::cout<< "\n\nKDL Number of Joints : "<<my_tree.getNrOfJoints() <<"\n"<< endl;

    std::cout<< "\n\nKDL Chain load : "<<base_link_name<<","<<tcp_link_name<<my_tree.getChain(base_link_name,tcp_link_name,my_chain) <<"\n"<< std::endl;

Eigen::MatrixXd Xstart(4,4) ;
Xstart= Eigen::MatrixXd::Identity(4,4);
Xstart(0,3) =1;
Eigen::MatrixXd Xend(4,4);
Xend= Eigen::MatrixXd::Identity(4,4);
KDL::JntArray q_start(6);
q_start(0) = -2.76285563e-01;
q_start(1) = -7.64454053e-01;
q_start(2) = -1.05993823e+00;
q_start(3) = -1.74532889e-04;
q_start(4) =-1.28752912e+00;
q_start(5) =  -2.82743280e-01;
JT jjjj = smcCartesianTrajectory(q_start,q_min,q_max,0,0,-0.1,5,100,5,my_chain);
std::cout<<jjjj.size()<<std::endl;
for(int i  =0;i<jjjj.size();i++){
    J temp = jjjj.at(i);
    std::cout<<"["<< temp.at(0) <<","<< temp.at(1) <<","<< temp.at(2) <<","<< temp.at(3) <<","<< temp.at(4) <<","<< temp.at(5) <<"]"<<std::endl;
}
/*
std::vector<Eigen::MatrixXd> traj =  CartesianTrajectory(Xstart,Xend, 5, 100, 5);

std::cout<<traj.at(0)<<std::endl;
Eigen::MatrixXd temp = traj.at(0);
tf2::Matrix3x3 Rtemp(temp(0,0),temp(0,1),temp(0,2),temp(1,0),temp(1,1),temp(1,2),temp(2,0),temp(2,1),temp(2,2));
tf2::Quaternion temp_quat;
Rtemp.getRotation(temp_quat);
std::cout<<temp_quat[0]<<","<<temp_quat[1]<<","<<temp_quat[2]<<","<<temp_quat[3]<<std::endl;


std::cout<<traj.at(50)<<std::endl;
std::cout<<traj.at(99)<<std::endl;
*/

/*

kdlSolver solver(urdf_name,base_link_name,tcp_link_name, 100000, 125, 1e-6);

    solver.getJointLimits(q_min, q_max);
    std::vector<double> p_targ (7); //quaternion
    p_targ.at(0) = target[0];
    p_targ.at(1) = target[1];
    p_targ.at(2) = target[2];
    p_targ.at(3) = target[3];
    p_targ.at(4) = target[4];
    p_targ.at(5) = target[5];
    p_targ.at(6) = target[6];
    normalizeQuaternion(p_targ);

    

 	KDL::JntArray q_start(jointnum);  
    KDL::JntArray q_result(jointnum);
double jnt_dist_threshold = 1.5;
    std::vector<double> q_init (jointnum);
    std::vector<double> q_targ (jointnum);
     for(int i=0;i<jointnum;i++){
        q_start(i) = rootr[robot_name]["joint_init"][i].asFloat();
        q_init[i] = q_start(i);
        q_result(i) = 0.0;
    }



        printf("\nJoints as calculated by IK:\n------------------------\n");
    bool verbose = true;
    solver.solvePoseIk(q_init, q_targ, p_targ, verbose);
    
    printDoubleVec(q_targ);


    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    KDL::Frame cartesian_pos;
    KDL::Frame cartesian_pos_result;
    
    KDL::Frame goal_pose(KDL::Rotation::Quaternion(target[0],target[1],target[2],target[3]),KDL::Vector(target[4],target[5],target[6]));

    fk_solver.JntToCart(q_start, cartesian_pos);
    printCatesianKDLFrame(cartesian_pos,"catesian pos start");
    
    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);
    KDL::ChainIkSolverPos_NR_JL iksolver1(my_chain,q_min,q_max,fk_solver,iksolver1v,2000,0.01);



    bool ret = iksolver1.CartToJnt(q_start,goal_pose,q_result);
    std::cout<<"ik ret : "<<ret<<std::endl;
    std::cout<<"ik q : "<<q_result(0)<<","<<q_result(1)<<","<<q_result(2)
                        <<","<<q_result(3)<<","<<q_result(4)<<","
                        <<q_result(5)<<std::endl;

    fk_solver.JntToCart(q_result, cartesian_pos_result);

    printCatesianKDLFrame(goal_pose,"goal pos");

    printCatesianKDLFrame(cartesian_pos_result,"pos result");

*/
return 0;
}
