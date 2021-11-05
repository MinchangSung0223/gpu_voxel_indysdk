// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2018-01-07
 *
 */
//----------------------------------------------------------------------/*

#include "gvl_ompl_planner_helper.h"
#include <Eigen/Dense>
#include <time.h>
#include <signal.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
//#include "Poco/Net/Net.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
//#include <gpu_voxels/logging/logging_gpu_voxels.h>
#include <thread>

#include <stdio.h>
#include <iostream>
#define IC_PERFORMANCE_MONITOR
#include <icl_core_config/Config.h>
#include <icl_core_performance_monitor/PerformanceMonitor.h>
#include <vector>
#include "spline.h"
using namespace gpu_voxels;
namespace bfs = boost::filesystem;
#define PI 3.141592
#define D2R 3.141592/180.0
#define R2D 180.0/3.141592
#define RED BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (10 % 249) )
#define GREEN BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (249) )

#define PURPLE BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (150 % 249) )
#define BLUE BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (200 % 249))
#define YELLOW BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (1 % 249))
using namespace NRMKIndy::Service::DCP;
using namespace mr;
using namespace std;
IndyDCPConnector connector("192.168.0.7", ROBOT_INDY7);

double task_goal_values[7] = {0,0,0,1,3.0,2.0,1.35}; 
unsigned int errCount=0;

double calcErr(KDL::JntArray q1, double* q2){
    double sum=0;
    double err = 9999;
    for(int i = 0;i<jointnum;i++){
        sum+=(q1(i)-q2[i])*(q1(i)-q2[i]);
    }
    err = sqrt(sum);
    return err;
}
double calcErr(J q1, double* q2){
    double sum=0;
    double err = 9999;
    for(int i = 0;i<jointnum;i++){
        sum+=(q1.at(i)-q2[i])*(q1.at(i)-q2[i]);
    }
    err = sqrt(sum);
    return err;
}

double calcErr(KDL::JntArray q1,KDL::JntArray q2){
    double sum=0;
    double err = 9999;
    for(int i = 0;i<jointnum;i++){
        sum+=(q1(i)-q2(i))*(q1(i)-q2(i));
    }
    err = sqrt(sum);
    return err;
}

Vector3ui map_dimensions(300,300,300);
bool cmdMove=false;
void printCatesianKDLFrame(KDL::Frame frame,char* str ){
    std::cout<<"======="<<str<<"=======\n\n"<<endl;
    for(int i =0;i<4;i++){
        for(int j=0;j<4;j++)
            std::cout<<frame(i,j)<<"\t";
        std::cout<<"\n"<<std::endl;
    }
}
JT smcCartesianTrajectory(J q_start_,KDL::JntArray q_min,KDL::JntArray q_max, float x,float y ,float z, double Tf, int N, int method,KDL::Chain my_chain) {
    JT joint_trajectory;
    joint_trajectory.clear();
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    KDL::Frame kdl_Xstart;
    KDL::JntArray q_start(jointnum);
    q_start(0) = q_start_.at(0);
    q_start(1) = q_start_.at(1);
    q_start(2) = q_start_.at(2);
    q_start(3) = q_start_.at(3);
    q_start(4) = q_start_.at(4);
    q_start(5) = q_start_.at(5);
    
    
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
    
   // std::cout <<kdl_Xstart<<std::endl;
   // std::cout <<Xstart<<std::endl;
   // std::cout <<Xend<<std::endl;
    
    
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

JT splineJointTrajectory(JT q_list,double Tf, double dt, int deriv_num) {

   std::cout<<"splineJointTrajectory"<<std::endl;   
   int N = q_list.size();
   std::cout<<"splineJointTrajectory1   :  "<<N<<std::endl;    
   if(N<3){
      q_list.push_back(q_list.at(N-1));
      N=N+1;
   }
   std::cout<<"splineJointTrajectory2"<<std::endl;     
   std::vector<double> Tlist;

   for(int j=0;j<N;j++){
    Tlist.push_back(double(Tf/(N-1)*j/1.0));
   }        
      std::cout<<"splineJointTrajectory3"<<std::endl;      
   std::vector<std::vector<double>> all_spline_thetalist;
   std::vector<std::vector<double>> all_spline_dthetalist;
    
   for(int j=0;j<JOINTNUM;j++){
       std::vector<double> thetalist;
       for(int i = 0;i<N;i++){
           std::array<double,JOINTNUM> temp=q_list.at(i);
           thetalist.push_back(temp[j]);
       }
       tk::spline s(Tlist,thetalist,tk::spline::cspline,false, tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
           std::vector<double> spline_thetalist;
           std::vector<double> spline_dthetalist;

       for(double t=0+dt;t<=Tf;){
        spline_thetalist.push_back(s(t));
        spline_dthetalist.push_back(s.deriv(1,t));
        t = t+dt;
       }
        all_spline_thetalist.push_back(spline_thetalist);
        all_spline_dthetalist.push_back(spline_dthetalist);
   }
      std::cout<<"splineJointTrajectory4"<<std::endl;   
   std::vector<std::array<double,JOINTNUM>>  spline_q_list;
   std::vector<std::array<double,JOINTNUM>>  spline_dq_list;

   for(int i=0;i<all_spline_thetalist.at(0).size()+1;i++){
           std::array<double,JOINTNUM> temp;
       for(int j=0;j<JOINTNUM;j++){
         std::vector<double>temp_ = all_spline_thetalist.at(j);
         temp[j]=temp_[i];
       }
       spline_q_list.push_back(temp);
   }

   for(int i=0;i<all_spline_dthetalist.at(0).size()+1;i++){
           std::array<double,JOINTNUM> temp;
       for(int j=0;j<JOINTNUM;j++){
         std::vector<double>temp_ = all_spline_dthetalist.at(j);
         temp[j]=temp_[i];
       }
       spline_dq_list.push_back(temp);
   }
   if (deriv_num==0){
       return spline_q_list;

   }
   else if (deriv_num==1){
       return spline_dq_list;
   }

}
JT velsplineJointTrajectory(JT q_list,double Tf, double dt, int deriv_num) {

  // std::cout<<"splineJointTrajectory"<<std::endl;   
   int N = q_list.size();
  // std::cout<<"splineJointTrajectory1   :  "<<N<<std::endl;    
   if(N<3){
      q_list.push_back(q_list.at(N-1));
      N=N+1;
   }
   //std::cout<<"splineJointTrajectory2"<<std::endl;     
   std::vector<double> Tlist;

   for(int j=0;j<N;j++){
    Tlist.push_back(double(Tf/(N-1)*j/1.0));
   }        
  //    std::cout<<"splineJointTrajectory3"<<std::endl;      
   std::vector<std::vector<double>> all_spline_thetalist;
   std::vector<std::vector<double>> all_spline_dthetalist;
    
   for(int j=0;j<JOINTNUM;j++){
       std::vector<double> thetalist;
       for(int i = 0;i<N;i++){
           std::array<double,JOINTNUM> temp=q_list.at(i);
           thetalist.push_back(temp[j]);
       }
       tk::spline s(Tlist,thetalist,tk::spline::cspline,false, tk::spline::first_deriv,0.0,tk::spline::first_deriv,0.0);
           std::vector<double> spline_thetalist;
           std::vector<double> spline_dthetalist;

       for(double t=0+dt;t<=Tf;){
        spline_thetalist.push_back(s(t));
        spline_dthetalist.push_back(s.deriv(1,t));
        t = t+dt;
       }
        all_spline_thetalist.push_back(spline_thetalist);
        all_spline_dthetalist.push_back(spline_dthetalist);
   }
   //   std::cout<<"splineJointTrajectory4"<<std::endl;   
   std::vector<std::array<double,JOINTNUM>>  spline_q_list;
   std::vector<std::array<double,JOINTNUM>>  spline_dq_list;

   for(int i=0;i<all_spline_thetalist.at(0).size()+1;i++){
           std::array<double,JOINTNUM> temp;
       for(int j=0;j<JOINTNUM;j++){
         std::vector<double>temp_ = all_spline_thetalist.at(j);
         temp[j]=temp_[i];
       }
       spline_q_list.push_back(temp);
   }

   for(int i=0;i<all_spline_dthetalist.at(0).size()+1;i++){
           std::array<double,JOINTNUM> temp;
       for(int j=0;j<JOINTNUM;j++){
         std::vector<double>temp_ = all_spline_dthetalist.at(j);
         temp[j]=temp_[i];
       }
       spline_dq_list.push_back(temp);
   }
   if (deriv_num==0){
       return spline_q_list;

   }
   else if (deriv_num==1){
       return spline_dq_list;
   }

}

std::vector<KDL::JntArray>  GvlOmplPlannerHelper::doTaskPlanning(double goal_values[7],KDL::JntArray start_values,ob::PathPtr path){

     clock_t start_t, end_t;
    double result;
    start_t = clock();
    PERF_MON_INITIALIZE(100, 1000);
    PERF_MON_ENABLE("planning");
    auto space(std::make_shared<ob::RealVectorStateSpace>(jointnum));
    ob::RealVectorBounds bounds(jointnum);
      for(int j = 0;j<jointnum;j++){
          bounds.setLow(j,q_min(j));
          bounds.setHigh(j,q_max(j));
      }


    std::vector<KDL::JntArray> q_list;
    q_list.clear();
    space->setBounds(bounds);

      this->si_->setStateValidityChecker(this->getptr());
      this->si_->setMotionValidator(this->getptr());
      this->si_->setup();

    og::PathSimplifier simp(this->si_);

    KDL::JntArray q_start(jointnum);  
    KDL::JntArray q_result(jointnum);  
    
    KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(my_chain);
    KDL::Frame cartesian_pos;
    KDL::Frame cartesian_pos_result;
    
    KDL::Frame goal_pose( KDL::Rotation::Quaternion(goal_values[0],goal_values[1],goal_values[2],goal_values[3]),KDL::Vector(goal_values[4],goal_values[5],goal_values[6]));

    fk_solver.JntToCart(q_start, cartesian_pos);

    KDL::ChainIkSolverVel_pinv iksolver1v(my_chain);
    KDL::ChainIkSolverPos_NR_JL iksolver1(my_chain,q_min,q_max,fk_solver,iksolver1v,2000,0.1);

    for(int i=0;i<jointnum;i++){
        q_start(i) = start_values(i);
    }

    //bool ret = iksolver1.CartToJnt(q_start,goal_pose,q_result);
    //std::cout<<"ik ret : "<<ret<<std::endl;
  
    


    fk_solver.JntToCart(q_result, cartesian_pos_result);

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    /*
    if(ret==false){
        for(int i = 0;i<jointnum;i++){
        start[i] = q_start(i);
        goal[i] = task_joint_values[i];
         }

    }else{

    for(int i = 0;i<jointnum;i++){
            start[i] = q_start(i);
            goal[i] = q_result(i);
        }
    }
    */

    for(int i = 0;i<jointnum;i++){
        start[i] = q_start(i);
        goal[i] = task_joint_values[i];
         }
    if(calcErr(joint_states,task_joint_values)<0.03)
        return q_list;
    //std::system("clear");
    LOGGING_INFO(Gpu_voxels, "PDEF \n" << endl);
    auto pdef(std::make_shared<ob::ProblemDefinition>(this->si_));
    pdef->setStartAndGoalStates(start, goal);
    LOGGING_INFO(Gpu_voxels, "PLANNER \n" << endl);

    //auto planner(std::make_shared<og::KPIECE1>(this->si_));

    auto planner(std::make_shared<og::ABITstar>(this->si_));

    
    planner->setProblemDefinition(pdef);
    planner->setup();
    int succs = 0;
    
    LOGGING_INFO(Gpu_voxels, "WHILE \n" << endl);
    float solveTime = 1;
    int no_succs_count = 0;
     while(succs<1)
    {
        double sum = 0.0;
        for(int k = 0;k<jointnum;k++){
            sum += sqrt((start[k]-goal[k])*(start[k]-goal[k]));
        }
        if(sum< 0.01){
            LOGGING_INFO(Gpu_voxels, "COMPLETE MOTION PLANNING \n" << endl);
            break;
        }
        try{
            planner->clear();
            LOGGING_INFO(Gpu_voxels,"SUCCESS : " <<succs<<", START PLANNING \n" << endl);

            const std::function< bool()> ptc;

            ob::PlannerStatus  solved = planner->ob::Planner::solve(solveTime);

            LOGGING_INFO(Gpu_voxels, "end PLANNING \n" << endl);

            PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("planner", "Planning time", "planning");


            if (solved)
            {
                ++succs;
                path = pdef->getSolutionPath();
                std::cout << "Found solution:" << std::endl;
                path->print(std::cout);
                simp.simplifyMax(*(path->as<og::PathGeometric>()));


            }else{
                std::cout << "No solution could be found "<<no_succs_count<<"  solveTime "<<solveTime << std::endl;
                no_succs_count++;
                solveTime +=0.1; 
                if(no_succs_count>=5)return q_list;
            }

            PERF_MON_SUMMARY_PREFIX_INFO("planning");
            std::cout << "END OMPL" << std::endl;
           }
        catch(int expn){
            std::cout << "ERRORROROROROR" << std::endl;
             return q_list;
        }

    }

            std::cout << "ENDdddddd OMPL" << std::endl;


    og::PathGeometric* solution= path->as<og::PathGeometric>();

    solution->interpolate(interpolate_num);
    int step_count = solution->getStateCount();
    std::cout<<step_count<<std::endl;
    std::cout << "ENDdddddd OMPL" << std::endl;

    for(int i=0;i<step_count;i++){
        const double *values = solution->getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
        double *temp_values = (double*)values;
        KDL::JntArray temp_joints_value(jointnum);
        for(int j =0;j<jointnum;j++)
            temp_joints_value(j)=temp_values[j];    
        q_list.push_back(temp_joints_value);
    
     }

    std::cout << "END OMPL"<<"Plan time : "<<result << " ms "<< " Qlist SIZE : "<<q_list.size() << std::endl;
 
    return q_list;


}




void rosjointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    gvl->clearMap("myRobotMap");
    gvl->clearMap("myRobotMapBitVoxel");
    
    gvl->clearMap("myRobotCollisionMap");
    gvl->clearMap("myRobotCollisionMapBitVoxel");
    
    
    for(size_t i = 0; i < msg->name.size(); i++)
    {
        myRobotJointValues[joint_names[i]] = msg->position[i];
        joint_states(i) = msg->position[i];
        joint_states_J[i] = msg->position[i];
        
    }
    gvl->setRobotConfiguration("myUrdfRobot",myRobotJointValues);
    gvl->setRobotConfiguration("myUrdfCollisionRobot",myRobotJointValues);
    gvl->insertRobotIntoMap("myUrdfRobot","myRobotMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfRobot","myRobotMapBitVoxel",YELLOW);
    //LOGGING_INFO(Gpu_voxels, "ROS JointState " << endl);

    gvl->insertRobotIntoMap("myUrdfCollisionRobot","myRobotCollisionMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfCollisionRobot", "myRobotCollisionMapBitVoxel", BLUE);
     new_data_received=true;

}

void rosDesiredPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    
    LOGGING_INFO(Gpu_voxels,msg->position.x<< endl);
     task_goal_values[0] = msg->orientation.x;
     task_goal_values[1] = msg->orientation.y;
     task_goal_values[2] = msg->orientation.z;
     task_goal_values[3] = msg->orientation.w;
     task_goal_values[4] = msg->position.x;
     task_goal_values[5] = msg->position.y;
     task_goal_values[6] = msg->position.z;
    new_pose_received=true;
}


int toggle = 1;




void rosMovingFlagCallback(const std_msgs::Bool::ConstPtr& msg){
    cmdMoveDone = msg->data;

}


void roscallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
    std::vector<Vector3f> point_data;
    point_data.resize(msg->points.size());
    for (uint32_t i = 0; i < msg->points.size(); i++)
    {
        point_data[i].x = msg->points[i].x;
        point_data[i].y = msg->points[i].y;
        point_data[i].z = msg->points[i].z;
    }
    my_point_cloud.update(point_data);
    my_point_cloud.transformSelf(&tf);
    Matrix4f base_tf=Matrix4f(1,0,0,base_x,0,1,0,base_y,0,0,1,base_z,0,0,0,1);
    my_point_cloud.transformSelf(&base_tf);


    new_data_received=true;
}

Eigen::Matrix4f GvlOmplPlannerHelper::loadBaseToCam(Eigen::Matrix4f TBaseToCamera){

    Eigen::Matrix3f Rx  = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Ry  = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f Rz  = Eigen::Matrix3f::Identity();
    float roll = cam_roll;//-PI/2.0;
    float pitch = cam_pitch;
    float yaw = cam_yaw;//-PI/2.0;

    Rx(1,1) = cos(roll);
    Rx(1,2) = -sin(roll);
    Rx(2,1) = sin(roll);
    Rx(2,2) = cos(roll);
    Ry(0,0) = cos(pitch);
    Ry(0,2) = sin(pitch);
    Ry(2,0) = -sin(pitch);
    Ry(2,2) = cos(pitch);
    Rz(0,0) = cos(yaw);
    Rz(0,1) = -sin(yaw);
    Rz(1,0) = sin(yaw);
    Rz(1,1) = cos(yaw);

    Eigen::Matrix3f R = Rz*Ry*Rx;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f TBaseToCameraTemp = TBaseToCamera;
    T(0,0)=R(0,0);
    T(0,1)=R(0,1);
    T(0,2)=R(0,2);
    T(1,0)=R(1,0);
    T(1,1)=R(1,1);
    T(1,2)=R(1,2);
    T(2,0)=R(2,0);
    T(2,1)=R(2,1);
    T(2,2)=R(2,2);
    TBaseToCamera(0,3)=0;
    TBaseToCamera(1,3)=0;
    TBaseToCamera(2,3)=0;
    
    TBaseToCamera = TBaseToCamera*T;
    TBaseToCamera(0,3) = TBaseToCameraTemp(0,3);
    TBaseToCamera(1,3) = TBaseToCameraTemp(1,3);
    TBaseToCamera(2,3) = TBaseToCameraTemp(2,3);
    
    return TBaseToCamera;
}




int pop_count = 0;
void GvlOmplPlannerHelper::tcpIter(){
    enum {
        SIZE_HEADER = 52,
        SIZE_COMMAND = 4,
        SIZE_HEADER_COMMAND = 56,
        SIZE_DATA_MAX =52,
        SIZE_DATA_ASCII_MAX = 32,
        BUFF_SIZE= 52,
            DATA_SIZE = 13

    };

    union Data{
        unsigned char byte[SIZE_DATA_MAX];
        float float6dArr[DATA_SIZE];
    };











    double result;
    clock_t start, end;

    signal(SIGINT, ctrlchandler);
    signal(SIGKILL, killhandler);
    signal(SIGTERM, killhandler);

    int cnt = 0;
    int flag_return = 0;
    int rpt_cnt = 1;
    double glob_time, loop_time;
    Data data_rev, data;
    unsigned char readBuff[BUFF_SIZE];
    unsigned char writeBuff[BUFF_SIZE];
    cmdMove=false;

try
    {
        try
        {
            cout << "Trying to connect server..." << endl;
            ss.connect(SocketAddress("192.168.0.7", 9911));
            Timespan timeout(1, 0);
            while (ss.poll(timeout, Poco::Net::Socket::SELECT_WRITE) == false){
                cout << "Connecting to server..." << endl;
            }
            cout << "Complete to connect server" << endl;
        }
        catch(int e){
            cout << "Failed to connect server" << endl;
        }
        
        
        Data data_rev, data;
        unsigned char readBuff[BUFF_SIZE];
        unsigned char writeBuff[BUFF_SIZE];
            
        data.float6dArr[0] = 100;
        memcpy(writeBuff, data.byte, SIZE_DATA_MAX);    
        try{
            ss.sendBytes(writeBuff, BUFF_SIZE);
        }catch(int e){
            std::cout<<"SEND ERROR"<<std::endl;
        }
        try{
            ss.receiveBytes(readBuff, BUFF_SIZE);
        }catch(int e){
            std::cout<<"RECV ERROR"<<std::endl;
        }
        memcpy(data_rev.byte, readBuff, SIZE_DATA_MAX);
        joint_states(0) =  data_rev.float6dArr[1];
        joint_states(1) =  data_rev.float6dArr[2];
        joint_states(2) =  data_rev.float6dArr[3];
        joint_states(3) =  data_rev.float6dArr[4];
        joint_states(4) =  data_rev.float6dArr[5];
        joint_states(5) =  data_rev.float6dArr[6];      

        for(size_t i = 0; i < jointnum; i++){
            myRobotJointValues[joint_names[i]] = joint_states(i);
            joint_states_J[i] = joint_states(i);
        }
             

        std::cout<<"START trajectorySubscriber"<<std::endl;
        q_list.clear();
        J q_goal = job_states1;
        J q_start;
        q_start.at(0) = joint_states(0); 
        q_start.at(1) = joint_states(1); 
        q_start.at(2) = joint_states(2); 
        q_start.at(3) = joint_states(3); 
        q_start.at(4) = joint_states(4); 
        q_start.at(5) = joint_states(5); 
        
        q_list.push_back(q_start);
        q_list.push_back(q_goal);
        vel_trajectory = splineJointTrajectory(q_list,5,0.1, 1);
        trajectory = splineJointTrajectory(q_list,5,0.1, 0);
        for(int i = 0;i<trajectory.size();i++){

             std::array<double, JOINTNUM> q = trajectory.at(0);
              std::array<double, JOINTNUM> qdot= vel_trajectory.at(0);

                reverse(trajectory.begin(), trajectory.end());
                reverse(vel_trajectory.begin(), vel_trajectory.end());

                trajectory.pop_back();
                vel_trajectory.pop_back();
                reverse(trajectory.begin(), trajectory.end());
                reverse(vel_trajectory.begin(), vel_trajectory.end());;

        

            try{
                data.float6dArr[0] = 107;
                data.float6dArr[1] = q.at(0);
                data.float6dArr[2] = q.at(1);
                data.float6dArr[3] = q.at(2);
                data.float6dArr[4] = q.at(3);
                data.float6dArr[5] = q.at(4);
                data.float6dArr[6] = q.at(5);

                data.float6dArr[7] = qdot.at(0);
                data.float6dArr[8] = qdot.at(1);
                data.float6dArr[9] = qdot.at(2);
                data.float6dArr[10] = qdot.at(3);
                data.float6dArr[11] = qdot.at(4);
                data.float6dArr[12] = qdot.at(5);   
                memcpy(writeBuff, data.byte, SIZE_DATA_MAX);                            
                ss.sendBytes(writeBuff, BUFF_SIZE);
            }catch(int e){
                std::cout<<"SEND ERROR"<<std::endl;
            }
            try{
            ss.receiveBytes(readBuff, BUFF_SIZE);
            }catch(int e){
                std::cout<<"RECV ERROR"<<std::endl;
            }
                
        }       
        trajectory.clear();
        vel_trajectory.clear();


         std::cout << "TCP is ready" << std::endl;

        while(true)
        {
          usleep(10000);
          std::cout<<"TJ Size :"<<trajectory.size()<<" STATE : "<<STATE <<std::endl;

           try{

            if(trajectory.size()>1 ){
               cmdMoveDone=true;
               
                std::array<double, JOINTNUM> q = trajectory.at(0);
                std::array<double, JOINTNUM> qdot = vel_trajectory.at(0);

                
                reverse(trajectory.begin(), trajectory.end());
                reverse(vel_trajectory.begin(), vel_trajectory.end());

                if(cmdMove==true){
                    trajectory.pop_back();
                    vel_trajectory.pop_back();
                }
                else if(cmdMove==false){
                        qdot.at(0) = 0.0;
                        qdot.at(1) = 0.0;
                        qdot.at(2) = 0.0;
                        qdot.at(3) = 0.0;
                        qdot.at(4) = 0.0;
                        qdot.at(5) = 0.0;
                        
                }
                reverse(trajectory.begin(), trajectory.end());
                reverse(vel_trajectory.begin(), vel_trajectory.end());
                try {
                       // printq("JS :",q);
                    data.float6dArr[0] = 107;
                    data.float6dArr[1] = q.at(0);
                    data.float6dArr[2] = q.at(1);
                    data.float6dArr[3] = q.at(2);
                    data.float6dArr[4] = q.at(3);
                    data.float6dArr[5] = q.at(4);
                    data.float6dArr[6] = q.at(5);

                    data.float6dArr[7] = qdot.at(0);
                    data.float6dArr[8] = qdot.at(1);
                    data.float6dArr[9] = qdot.at(2);
                    data.float6dArr[10] = qdot.at(3);
                    data.float6dArr[11] = qdot.at(4);
                    data.float6dArr[12] = qdot.at(5);
                                
                    memcpy(writeBuff, data.byte, SIZE_DATA_MAX);
                    ss.sendBytes(writeBuff, BUFF_SIZE);

        
                }
                catch (int expn) {
                    cout << "SEND ERROR" << endl;
                }
                if (flag_return == 0)
                {
                    ss.receiveBytes(readBuff, BUFF_SIZE);
                    memcpy(data_rev.byte, readBuff, SIZE_DATA_MAX);

                   joint_states(0) =  data_rev.float6dArr[1];
                    joint_states(1) =  data_rev.float6dArr[2];
                    joint_states(2) =  data_rev.float6dArr[3];
                    joint_states(3) =  data_rev.float6dArr[4];
                    joint_states(4) =  data_rev.float6dArr[5];
                    joint_states(5) =  data_rev.float6dArr[6];      
                       for(size_t i = 0; i < jointnum; i++){
                            myRobotJointValues[joint_names[i]] = joint_states(i);
                            joint_states_J[i] = joint_states(i);
                        }
                             
                
                }
                else{
                    cout << "No response from server..." << endl;
                    break;
                }
            }
            else if(trajectory.size()<=1){
                if(trajectory.size()==1){
                    STATE++;
                    if(STATE >= task_joint_values_list.size()){
                        STATE = 0;

                    }

                    new_pose_received=true;
                    usleep(1000000);
                }
                cmdMoveDone = true;
                    try {
                    data.float6dArr[0] = 100;
                                
                    memcpy(writeBuff, data.byte, SIZE_DATA_MAX);
                    ss.sendBytes(writeBuff, BUFF_SIZE);
        
                }
                catch (int expn) {
                    cout << "SEND ERROR" << endl;
                }
                if (flag_return == 0)
                {
                    ss.receiveBytes(readBuff, BUFF_SIZE);
                    memcpy(data_rev.byte, readBuff, SIZE_DATA_MAX);

                    joint_states(0) =  data_rev.float6dArr[1];
                    joint_states(1) =  data_rev.float6dArr[2];
                    joint_states(2) =  data_rev.float6dArr[3];
                    joint_states(3) =  data_rev.float6dArr[4];
                    joint_states(4) =  data_rev.float6dArr[5];
                    joint_states(5) =  data_rev.float6dArr[6];      
                   for(size_t i = 0; i < jointnum; i++){
                        myRobotJointValues[joint_names[i]] = joint_states(i);
                        joint_states_J[i] = joint_states(i);
                    }
                         

                
                }
                else{
                    cout << "No response from server..." << endl;
                    break;
                }
            }
            }
            catch(int e){
            }

        }

        ss.close();

    }

    catch (Poco::Exception& exc)

    {

        cout << "Fail to connect server..." << exc.displayText() << endl;

    }



}









void GvlOmplPlannerHelper::rosIter(){
    int argc;
    char **argv;
    ros::init(argc,argv,"gpu_voxel");

    const Vector3f camera_offsets(0.0f,
                                 0.0f, 
                                 0.0f); 


        
    TBaseToCamera = GvlOmplPlannerHelper::loadBaseToCam(TBaseToCamera);
    tf = Matrix4f(TBaseToCamera(0,0),TBaseToCamera(0,1),TBaseToCamera(0,2),TBaseToCamera(0,3)
        ,TBaseToCamera(1,0),TBaseToCamera(1,1),TBaseToCamera(1,2),TBaseToCamera(1,3)
        ,TBaseToCamera(2,0),TBaseToCamera(2,1),TBaseToCamera(2,2),TBaseToCamera(2,3)
        ,TBaseToCamera(3,0),TBaseToCamera(3,1),TBaseToCamera(3,2),TBaseToCamera(3,3));
    std::cout<<"==========TBaseToCmaera==========\n"<<std::endl;
    tf.print();
  std::cout << "Press Enter Key if ready!" << std::endl;
  std::cin.ignore();
    ros::NodeHandle nh;
    ros::NodeHandle nh2;

    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 1, rosjointStateCallback); 
    ros::Subscriber desiredPose_sub = nh.subscribe("/desired_pose", 1, rosDesiredPoseCallback); 
    ros::Subscriber moving_flag = nh.subscribe("/cmdMoveDone", 1, rosMovingFlagCallback); 

    
    //ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/color/points", 1,roscallback);
    std::cout<<point_topic_name<<std::endl;
    const char* point_topic_name_ =point_topic_name.c_str();
    ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(point_topic_name_, 1,roscallback);

    //ros::Subscriber point_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >("/cam_0_zf", 1,roscallback);
    ros::Publisher pub_joint =  nh.advertise<sensor_msgs::JointState>("/joint_states_desired", 1000);
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);

    ros::Publisher cmdMove_pub = nh2.advertise<std_msgs::Bool>("/cmdMove", 100); 
    ros::Publisher cmdGripper_pub = nh2.advertise<std_msgs::Bool>("/cmdGripper", 100); 
    
    ros::Rate r(100);
    new_data_received = true; // call visualize on the first iteration
    new_pose_received=true;
    size_t num_colls = 0;
    size_t num_colls_2 = 0;
    
    countingVoxelList = dynamic_pointer_cast<CountingVoxelList>(gvl->getMap("countingVoxelList"));

    myRobotCollisionMapBitVoxel = dynamic_pointer_cast<BitVectorVoxelList>(gvl->getMap("myRobotCollisionMapBitVoxel"));
    STATE = 0;
     trajectory_msgs::JointTrajectory jointTrajectory;
trajectory_msgs::JointTrajectoryPoint points;
    float prev_error = 999999;
    //usleep(5000000);
    unsigned int colls_count = 0;
    while (ros::ok())
    {

        //init
        if(new_data_received){
          //  LOGGING_INFO(Gpu_voxels, "Recived Pointcloud " << endl);

            countingVoxelList->clearMap();
            myEnvironmentMap->clearMap();
            myEnvironmentMap2->clearMap();




            countingVoxelList->insertPointCloud(my_point_cloud,eBVM_OCCUPIED);
            myEnvironmentMap2->merge(countingVoxelList);
            //num_colls_2 = countingVoxelList->as<gpu_voxels::voxellist::CountingVoxelList>()->collideWith(myRobotCollisionMapBitVoxel->as<gpu_voxels::voxellist::BitVectorVoxelList>(), 1.0f);
            countingVoxelList->as<gpu_voxels::voxellist::CountingVoxelList>()->subtractFromCountingVoxelList(myRobotCollisionMapBitVoxel->as<gpu_voxels::voxellist::BitVectorVoxelList>(),Vector3f());
            myEnvironmentMap->merge(countingVoxelList);
            num_colls = gvl->getMap("countingVoxelList")->as<gpu_voxels::voxellist::CountingVoxelList>()->collideWith(gvl->getMap("mySolutionMap")->as<gpu_voxels::voxellist::BitVectorVoxelList>(), 1.0f);

      

            if(num_colls>=coll_threshold2){
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;
                std::cout << "!!!!!!!!!!!!!!!Detected Collision!!!!!!!!! " << num_colls << " collisions " << std::endl;  
                
                cmdMove=false;
                new_pose_received = true;
            }else{
                cmdMove=true;
                std_msgs::Bool cmdMove_msg;
                cmdMove_msg.data=cmdMove;
                cmdMove_pub.publish(cmdMove_msg);          
                ros::spinOnce();
                r.sleep();
            }

            doVis();
            new_data_received = false;
        }
     

        J temp = task_joint_values_list.at(STATE);
        for(int i = 0;i<jointnum;i++)
            task_joint_values[i]=temp.at(i);

       if(new_pose_received){

            collision_count = 0;
            std::cout<<"START PLANNING ITER STATE : "<<STATE<<std::endl;
            std::cout<<"joint states : ";

            for (int i = 0;i<jointnum;i++)
                std::cout<<joint_states(i)<<",";
            std::cout<<""<<std::endl;
            std::cout<<"target joint Velue : ";
            for (int i = 0;i<jointnum;i++)
                std::cout<<task_joint_values[i]<<",";
            std::cout<<""<<std::endl;


            if(calcErr(joint_states_J,task_joint_values)<0.03){
                STATE++;
              if(STATE >= task_joint_values_list.size()){
                    STATE = 0;

                }

                new_pose_received=true;
                continue;
            }

            std::cout<<"START PLANNING ";
            ob::PathPtr path;
            joint_trajectory=GvlOmplPlannerHelper::doTaskPlanning(task_goal_values,joint_states,path);
           
            if(joint_trajectory.size()>0){
                       q_list.clear();
                       for(int j= 0;j<joint_trajectory.size();j++){
                                KDL::JntArray temp = joint_trajectory.at(j);
                                J temp2;
                                temp2.at(0) = temp(0);
                                temp2.at(1) = temp(1);
                                temp2.at(2) = temp(2);
                                temp2.at(3) = temp(3);
                                temp2.at(4) = temp(4);
                                temp2.at(5) = temp(5);
                                std::cout<<temp2.at(0)<<","<<temp2.at(1)<<","<<temp2.at(2)<<","<<temp2.at(3)<<","<<temp2.at(4)<<","<<temp2.at(5)<<std::endl;
                                q_list.push_back(temp2);
                        }

                        
            }


            new_pose_received = false;
        }
        if(q_list.size()>0){
                float  dt = 0.01;

                JT temp_trajectory = splineJointTrajectory(q_list,1.0,dt, 0);
                JT temp_veltrajectory;
                temp_veltrajectory.clear();
                 temp_veltrajectory= velsplineJointTrajectory(q_list,1.0,dt, 1);
                //JT temp_veltrajectory = splineJointTrajectory(q_list,1.0,0.01, 1);
                vel_trajectory.clear();
                trajectory.clear();

                if(trajectory.size()==0){
                    trajectory.assign( temp_trajectory.begin(), temp_trajectory.end() );
                   vel_trajectory.assign(temp_veltrajectory.begin(),temp_veltrajectory.end());
                    
                }
                q_list.clear();


        }

        if(joint_trajectory.size()>0){
            visualizeSolution(joint_trajectory);
            reverse(joint_trajectory.begin(), joint_trajectory.end());
            joint_trajectory.pop_back();
            reverse(joint_trajectory.begin(), joint_trajectory.end());
        }
        if(joint_trajectory.size()>0){
            visualizeSolution(joint_trajectory);
            reverse(joint_trajectory.begin(), joint_trajectory.end());
            joint_trajectory.pop_back();
            reverse(joint_trajectory.begin(), joint_trajectory.end());

        }


        ros::spinOnce();
        r.sleep();


    }




















    ss.close();
    exit(EXIT_SUCCESS);
}



GvlOmplPlannerHelper::GvlOmplPlannerHelper(const ob::SpaceInformationPtr &si)
    : ob::StateValidityChecker(si)
    , ob::MotionValidator(si)
{


    si_ = si;
    stateSpace_ = si_->getStateSpace().get();

    assert(stateSpace_ != nullptr);

    gvl = gpu_voxels::GpuVoxels::getInstance();
    gvl->initialize(map_dimensions.x,map_dimensions.y, map_dimensions.z, voxel_side_length);

    // We add maps with objects, to collide them
    gvl->addMap(MT_PROBAB_VOXELMAP,"myRobotMap");
    gvl->addMap(MT_BITVECTOR_VOXELLIST, "myRobotMapBitVoxel");


    gvl->addMap(MT_PROBAB_VOXELMAP,"myRobotCollisionMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentAllMap");

    //gvl->insertPointCloudFromFile("myEnvironmentAllMap", "./binvox/environment_all.binvox", true,
    //                                  gpu_voxels::eBVM_OCCUPIED, true, gpu_voxels::Vector3f(0.0, 0.0, -0.01),1);
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myEnvironmentMap2");
    
    gvl->addMap(MT_BITVECTOR_VOXELLIST, "myRobotCollisionMapBitVoxel");
    myEnvironmentMap = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("myEnvironmentMap"));
    myEnvironmentMap2 = dynamic_pointer_cast<ProbVoxelMap>(gvl->getMap("myEnvironmentMap2"));


    gvl->addMap(MT_BITVECTOR_VOXELLIST,"mySolutionMap");
    gvl->addMap(MT_PROBAB_VOXELMAP,"myQueryMap");
    gvl->addMap(MT_COUNTING_VOXELLIST,"countingVoxelList");
    gvl->addMap(MT_COUNTING_VOXELLIST,"countingVoxelList2");
    
    gvl->addRobot("myUrdfCollisionRobot",colilsion_urdf_name , true);
    gvl->addRobot("myUrdfRobot",urdf_name , true);
    

     if (!kdl_parser::treeFromFile(urdf_name, my_tree)){
             LOGGING_INFO(Gpu_voxels,"Failed to construct kdl tree");
    }

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Number of Joints : "<<my_tree.getNrOfJoints() <<"\n"<< endl);

    LOGGING_INFO(Gpu_voxels, "\n\nKDL Chain load : "<<my_tree.getChain(base_link_name,tcp_link_name,my_chain) <<"\n"<< endl);



    PERF_MON_ENABLE("pose_check");
    PERF_MON_ENABLE("motion_check");
    PERF_MON_ENABLE("motion_check_lv");
}



GvlOmplPlannerHelper::~GvlOmplPlannerHelper()
{
    gvl.reset(); // Not even required, as we use smart pointers.
}

void GvlOmplPlannerHelper::moveObstacle()
{

}

void GvlOmplPlannerHelper::doVis()
{
     //LOGGING_INFO(Gpu_voxels, "Dovis " << endl);


        gvl->clearMap("myRobotMap");
    gvl->clearMap("myRobotMapBitVoxel");
    
    gvl->clearMap("myRobotCollisionMap");
    gvl->clearMap("myRobotCollisionMapBitVoxel");
    
    
 
    gvl->setRobotConfiguration("myUrdfRobot",myRobotJointValues);
    gvl->setRobotConfiguration("myUrdfCollisionRobot",myRobotJointValues);
    gvl->insertRobotIntoMap("myUrdfRobot","myRobotMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfRobot","myRobotMapBitVoxel",YELLOW);
    //LOGGING_INFO(Gpu_voxels, "ROS JointState " << endl);

    gvl->insertRobotIntoMap("myUrdfCollisionRobot","myRobotCollisionMap",eBVM_OCCUPIED);
    gvl->insertRobotIntoMap("myUrdfCollisionRobot", "myRobotCollisionMapBitVoxel", BLUE);




     gvl->visualizeMap("myEnvironmentMap");
     gvl->visualizeMap("myEnvironmentMap2");
     
     gvl->visualizeMap("myEnvironmentAllMap");
     
 
    
    gvl->visualizeMap("myRobotMap");    
    gvl->visualizeMap("myRobotMapBitVoxel");    
        gvl->visualizeMap("mySolutionMap");

    gvl->visualizeMap("myRobotCollisionMapBitVoxel");
    //gvl->visualizeMap("myRobotCollisionMap");


    gvl->visualizeMap("countingVoxelList");
    gvl->visualizeMap("countingVoxelList2");
    
    //gvl->insertBoxIntoMap(Vector3f(1.0, 0.8 ,-0.01), Vector3f(2.7, 2.21 ,0.000), "myEnvironmentMap", eBVM_OCCUPIED, 2);
    
    gvl->insertBoxIntoMap(Vector3f(0.8, 0.0 ,0.0), Vector3f(0.81, 2.21 ,1.5), "myEnvironmentMap", eBVM_OCCUPIED, 1);
    gvl->insertBoxIntoMap(Vector3f(0.8, 2.15 ,0.0), Vector3f(2.0, 2.16 ,1.5), "myEnvironmentMap", eBVM_OCCUPIED, 1);
   // gvl->insertBoxIntoMap(Vector3f(1.0, 1.0,1.09), Vector3f(2.2, 2.0  ,1.1), "myEnvironmentMap", eBVM_OCCUPIED, 2);
   // gvl->insertBoxIntoMap(Vector3f(1.0, 1.0,0.0), Vector3f(2.2, 2.0 ,-0.001), "myEnvironmentMap", eBVM_OCCUPIED, 1);
    
     //gvl->insertBoxIntoMap(Vector3f(1.0, 0.8 ,0.0), Vector3f(2.7, 2.21 ,-0.01), "myEnvironmentMap", eBVM_OCCUPIED, 2);
}


void GvlOmplPlannerHelper::visualizeSolution(ob::PathPtr path)
{
    gvl->clearMap("mySolutionMap");

    PERF_MON_SUMMARY_PREFIX_INFO("pose_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check");
    PERF_MON_SUMMARY_PREFIX_INFO("motion_check_lv");

    //std::cout << "Robot consists of " << gvl->getRobot("myUrdfRobot")->getTransformedClouds()->getAccumulatedPointcloudSize() << " points" << std::endl;

    og::PathGeometric* solution = path->as<og::PathGeometric>();
    solution->interpolate();


    for(size_t step = 0; step < solution->getStateCount(); ++step)
    {

        const double *values = solution->getState(step)->as<ob::RealVectorStateSpace::StateType>()->values;

        robot::JointValueMap state_joint_values;
        for(int j =0;j<jointnum;j++)
            state_joint_values[joint_names[j]] = values[j];
        // update the robot joints:
        gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap("myUrdfRobot", "mySolutionMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START + (step % 249) ));
    }

    gvl->visualizeMap("mySolutionMap");

}
void GvlOmplPlannerHelper::visualizeSolution(std::vector<KDL::JntArray> solution)
{
    gvl->clearMap("mySolutionMap");

   for(int j = 0;j<solution.size();j++)
    {

        KDL::JntArray temp_q = solution.at(j);

        robot::JointValueMap state_joint_values;
        for(int j =0;j<jointnum;j++)
            state_joint_values[joint_names[j]] = temp_q(j);
        // update the robot joints:
        gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
        // insert the robot into the map:
        gvl->insertRobotIntoMap("myUrdfRobot", "mySolutionMap", GREEN);
    }
    gvl->visualizeMap("mySolutionMap");

}

void GvlOmplPlannerHelper::insertStartAndGoal(const ompl::base::ScopedState<> &start, const ompl::base::ScopedState<> &goal) const
{
    gvl->clearMap("myQueryMap");

    robot::JointValueMap state_joint_values;
    for(int j =0;j<jointnum;j++)
            state_joint_values[joint_names[j]] = start[j];

    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    gvl->insertRobotIntoMap("myUrdfRobot", "myQueryMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START));

        for(int j =0;j<jointnum;j++)
            state_joint_values[joint_names[j]] = goal[j];
    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    gvl->insertRobotIntoMap("myUrdfRobot", "myQueryMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+1));
}

bool GvlOmplPlannerHelper::isValid(const ompl::base::State *state) const
{
    PERF_MON_START("inserting");

    std::lock_guard<std::mutex> lock(g_i_mutex);

    gvl->clearMap("myRobotMap");

    const double *values = state->as<ob::RealVectorStateSpace::StateType>()->values;

    robot::JointValueMap state_joint_values;
    for(int j =0;j<jointnum;j++)
        state_joint_values[joint_names[j]] = values[j];
    // update the robot joints:
    gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
    // insert the robot into the map:
    gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", eBVM_OCCUPIED);

    //gvl->setRobotConfiguration("myUrdfCollisionRobot",state_joint_values);
    //gvl->insertRobotIntoMap("myUrdfCollisionRobot","myRobotCollisionMap",eBVM_OCCUPIED);


    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Pose Insertion", "pose_check");

    PERF_MON_START("coll_test");
    size_t num_colls_pc = gvl->getMap("myRobotMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);
    gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+30));

   // size_t num_colls_pc2 = gvl->getMap("myRobotCollisionMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);
    //gvl->insertRobotIntoMap("myUrdfCollisionRobot", "myRobotCollisionMap", BitVoxelMeaning(eBVM_SWEPT_VOLUME_START+30));

    PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "pose_check");

    std::cout << "Validity check on state ["  << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << ", " << values[4] << ", " << values[5] << "] resulting in " <<  num_colls_pc << " colls." << std::endl;
    //std::cout << "Validity check on state2 ["  << values[0] << ", " << values[1] << ", " << values[2] << ", " << values[3] << ", " << values[4] << ", " << values[5] << "] resulting in " <<  num_colls_pc2 << " colls." << std::endl;

    
   // return num_colls_pc==0;
    if(num_colls_pc<coll_threshold){
        return true;
    }
    else if(num_colls_pc>=coll_threshold){
        return false;
    }

}

bool GvlOmplPlannerHelper::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                       std::pair< ompl::base::State*, double > & lastValid) const
{

    //    std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //    std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //    std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;


    std::lock_guard<std::mutex> lock(g_j_mutex);
    gvl->clearMap("myRobotMap");

    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    //std::cout << "Called interpolating motion_check_lv to evaluate " << nd << " segments" << std::endl;

    PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check_lv");
    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))

            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;

            }
        }

        si_->freeState(test);

    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;
}



bool GvlOmplPlannerHelper::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    std::lock_guard<std::mutex> lock(g_i_mutex);
    gvl->clearMap("myRobotMap");


    //        std::cout << "LongestValidSegmentFraction = " << stateSpace_->getLongestValidSegmentFraction() << std::endl;
    //        std::cout << "getLongestValidSegmentLength = " << stateSpace_->getLongestValidSegmentLength() << std::endl;
    //        std::cout << "getMaximumExtent = " << stateSpace_->getMaximumExtent() << std::endl;



    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    // not required with ProbabVoxels:
    //    if(nd > 249)
    //    {
    //        std::cout << "Too many intermediate states for BitVoxels" << std::endl;
    //        exit(1);
    //    }

    if (nd > 1)
    {
        PERF_MON_START("inserting");

        /* temporary storage for the checked state */
        ob::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);


            const double *values = test->as<ob::RealVectorStateSpace::StateType>()->values;

            robot::JointValueMap state_joint_values;
            for(int j =0;j<jointnum;j++)
                state_joint_values[joint_names[j]] = values[j];

            // update the robot joints:
            gvl->setRobotConfiguration("myUrdfRobot", state_joint_values);
            // insert the robot into the map:
            gvl->insertRobotIntoMap("myUrdfRobot", "myRobotMap", eBVM_OCCUPIED);

        }
        PERF_MON_ADD_DATA_NONTIME_P("Num poses in motion", float(nd), "motion_check");

        si_->freeState(test);

        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("insert", "Motion Insertion", "motion_check");

        //gvl->visualizeMap("myRobotMap");
        PERF_MON_START("coll_test");
        size_t num_colls_pc = gvl->getMap("myRobotMap")->as<voxelmap::ProbVoxelMap>()->collideWith(gvl->getMap("myEnvironmentMap")->as<voxelmap::ProbVoxelMap>(), 0.7f);

        std::cout << "CheckMotion1 for " << nd << " segments. Resulting in " << num_colls_pc << " colls." << std::endl;
        PERF_MON_SILENT_MEASURE_AND_RESET_INFO_P("coll_test", "Pose Collsion", "motion_check");

        //result = (num_colls_pc == 0);
         if(num_colls_pc<coll_threshold){
                result= true;
         }
            else if(num_colls_pc>=coll_threshold){
                result= false;
            }

    }


    if (result)
        valid_++;
    else
        invalid_++;


    return result;


}