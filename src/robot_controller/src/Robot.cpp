#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "sosdata.h"
#include "Robot.h"
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>
#include <ros/duration.h>
#include "BaseKinematicModel.h"
#include "ArmKinematicModel.h"
#include "DummyController.h"
#include "PController.h"
#include <iostream>

using namespace std;

namespace ift
{
	//
	// Constructors/Destructors
	//

	/**
	 * Constructor
	 * @param  robot_name
	 */
	Robot::Robot(string robot_name) : _robotName(robot_name), _filter(3, 3)
	{
		// message publisher and subscriber
		_cmd_base_pub = _n.advertise<Control> (std::string("base"), 1);
		_cmd_arm_pub = _n.advertise<Control> ("arm", 1);
		_traj_pub = _n.advertise<NavTraj> ("traj", 1);
		_goal_pub = _n.advertise<PoseStamped>("goal", 1);
		_goal_project_pub = _n.advertise<PoseStamped>("goal_project", 1); //2016-05-23, WT
		_pose_pub = _n.advertise<PoseStamped>("pose", 1);
    _slam_map_sub = _n.subscribe("map", 10, &Robot::mapCallback, this, ros::TransportHints().udp()); //Use UDP instead of TCP, 2016-01-17, WT
    //_slam_map_sub = _n.subscribe("map", 10, &Robot::mapCallback, this); //2017-02-09, WT, UDP makes worse
    _slam_pose_sub = _n.subscribe("slam_out_pose", 10, &Robot::poseCallback, this, ros::TransportHints().udp()); //Use UDP instead of TCP, 2016-01-17, WT
    //_slam_pose_sub = _n.subscribe("slam_out_pose", 10, &Robot::poseCallback, this); //2017-02-09, WT, UDP makes worse

		// initialize simulation time
		_start_time = ros::Time::now();

		//base transmission initialization
		ift::KinematicModelPtr base_kptr(new ift::BaseKinematicModel(3));
		ift::ControllerPtr base_cptr(new ift::PController);
		ift::TransmissionPtr base_bptr(new ift::Transmission("base", base_kptr, base_cptr));
		_trsmMap.insert(std::pair<std::string, ift::TransmissionPtr>("base", base_bptr));

		//arm transmission initialization
		ift::KinematicModelPtr arm_kptr(new ift::ArmKinematicModel());
		ift::ControllerPtr arm_cptr(new ift::DummyController);
		ift::TransmissionPtr arm_bptr(new ift::Transmission("arm", arm_kptr, arm_cptr));
		_trsmMap.insert(std::pair<std::string, ift::TransmissionPtr>("arm", arm_bptr));

		// set _navtraj frame
		_navtraj.header.frame_id = "ground";
		//initialize pos
        
        tfGround2Map.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );   //2016-10-28, WT
        tfGround2Map.setRotation( tf::Quaternion(0, 0, 0, 1) ); //2016-10-28, WT
	}

	Robot::~Robot() { }

	//
	// Methods
	//

	void Robot::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
	{
		// adjust global frame
	}

	void Robot::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		// obtain measurement
		MatrixWrapper::ColumnVector measurement(3);
		measurement(1) = msg->pose.position.x;
		measurement(2) = msg->pose.position.y;
		measurement(3) = tf::getYaw(msg->pose.orientation);

		// check if skip this message
		if(_msg_queue.size()>0 && msg->header.stamp.toSec() - _msg_queue.front().header.stamp.toSec() <  0.4)
		  return;
		  
		// obtain previous position from _queue
		ift::PoseStamped pre_pos;
		if (_msg_queue.size() < 5 && _msg_queue.size() > 0) 
		{
			pre_pos = _msg_queue.front();
		} 
		else if (_msg_queue.size() >= 5) 
		{
			pre_pos = _msg_queue.front();
			_msg_queue.pop();
		} 
		else 
		{
			pre_pos = _trsmMap["base"]->get_knmatcMdlPtr()->get_pos();
		}
		_msg_queue.push(*msg);

		// time elapsed
		float dT = msg->header.stamp.toSec() - pre_pos.header.stamp.toSec();

		// update filter parameter
		_filter.setB(dT);

		// obtain control
		MatrixWrapper::ColumnVector input(3);
		ift::VelStamped target_dir = _trsmMap["base"]->get_ctrlPtr()->_target_dir;
		input(1) = target_dir.pose.position.x;
		input(2) = target_dir.pose.position.y;
		input(3) = target_dir.pose.position.z;

		// filter measurment with previous estimation
		_filter.update(input, measurement);
		MatrixWrapper::ColumnVector result = _filter.getExpected();

		// obtain message
		ift::PoseStamped pos_in, pos_out;
        //tf::StampedTransform tfGround2Map; //, tf_pos_in, tf_pos_out; //2016-10-25, WT
		pos_in.header = msg->header;
		pos_in.pose.position.x = msg->pose.position.x;
		pos_in.pose.position.y = msg->pose.position.y;
		pos_in.pose.orientation.z = msg->pose.orientation.z;
		pos_in.pose.orientation.w = msg->pose.orientation.w;
        //std::cout << "slam_out_pose: (" <<  pos.pose.position.x << ", " << pos.pose.position.x << ")" << endl; // for test 2016-10-19, WT
        //std::cout << "frame_id of slam_out_pose: " << pos.header.frame_id << endl;
        //try
        //{
            //listener.waitForTransform("/ground", _robotName + "/map", ros::Time(0), ros::Duration(10.0));
            //listener.lookupTransform("/ground", _robotName + "/map", ros::Time(0), tfGround2Map);
            //listener.transformPose("/ground", pos_in, pos_out); //2016-10-21, WT
        //}
        //catch( tf::TransformException ex)
        //{
            //ROS_ERROR("transfrom exception : %s",ex.what());
            ////ros::Duration(1.0).sleep();
        //}
        if( _tfListener.canTransform("/ground", _robotName + "/map", msg->header.stamp ) )
		{			
            //_tfListener.lookupTransform("/ground", _robotName + "/map", ros::Time(0), tfGround2Map);
            _tfListener.lookupTransform("/ground", _robotName + "/map", msg->header.stamp, tfGround2Map);//2017-01-21, WT, http://answers.ros.org/question/182898/messagefilter-with-pointcloud-posemessage/
            std::cout << "(x,y,yaw): (" <<  tfGround2Map.getOrigin().x() << ", " << tfGround2Map.getOrigin().y() << ", " << tf::getYaw(tfGround2Map.getRotation()) << ")" << endl; // for test 2016-10-19, WT
            //_tfListener.transformPose("/ground", pos_in, pos_out); //2016-10-21, WT
		}
		else
		{
			std::cout << "no tf, (x,y,yaw): (" <<  tfGround2Map.getOrigin().x() << ", " << tfGround2Map.getOrigin().y()  << ", " << tf::getYaw(tfGround2Map.getRotation()) << ")" << endl; // for test 2016-10-19, WT
            ROS_DEBUG_STREAM( "No transformation from frame [" << _robotName + "/map" << "] to frame [" << "/ground" << "]!" );
		}
        
        //tf::transformStampedMsgToTF (pos_in, tf_pos_in);
        
        //tf_pos_out = tf_pos_in * tfGround2Map;
        
        //tf::transformStampedTFToMsg (tf_pos_out, pose_out);
        tf::Quaternion pos_out_Q;
        double theta_tf = tf::getYaw(tfGround2Map.getRotation());
        pos_out.header = pos_in.header;
        pos_out.header.frame_id = "ground";
        pos_out.pose.position.x = pos_in.pose.position.x * cos(theta_tf) - pos_in.pose.position.y * sin(theta_tf) + tfGround2Map.getOrigin().x();
        pos_out.pose.position.y = pos_in.pose.position.x * sin(theta_tf) + pos_in.pose.position.y * cos(theta_tf) + tfGround2Map.getOrigin().y();
        pos_out.pose.orientation = pos_in.pose.orientation;
        pos_out_Q = tf::Quaternion(pos_in.pose.orientation.x, pos_in.pose.orientation.y, pos_in.pose.orientation.z, pos_in.pose.orientation.w);
        pos_out_Q *= tfGround2Map.getRotation();
        //pos_out.pose.orientation = pos_out_Q;
        pos_out.pose.orientation.x = pos_out_Q.x();
        pos_out.pose.orientation.y = pos_out_Q.y();
        pos_out.pose.orientation.z = pos_out_Q.z();
        pos_out.pose.orientation.w = pos_out_Q.w();
        
        //std::cout << "(x,y): (" <<  tfGround2Map.getOrigin().x() << ", " << tfGround2Map.getOrigin().y() << ")" << endl; // for test 2016-10-19, WT
        
        //std::cout << "frame_id of pose_out: " << pos_out.header.frame_id << endl;
        std::cout << "pose_in: (" <<  pos_in.pose.position.x << ", " << pos_in.pose.position.y << ", " << tf::getYaw(pos_in.pose.orientation) << ")" << endl; // for test 2016-10-23, WT
        std::cout << "pose_out: (" <<  pos_out.pose.position.x << ", " << pos_out.pose.position.y << ", " << tf::getYaw(pos_out.pose.orientation) << ")" << endl; // for test 2016-10-23, WT
		// update position
		_trsmMap["base"]->get_knmatcMdlPtr()->set_pos(pos_out);

		/*
		std::cout << dT << "----" << std::endl;
		std::cout << result << std::endl;
		*/

		// update velocity estimation
		ift::VelStamped vel, pre_vel;
		vel.header = msg->header;
        vel.header.frame_id = "ground"; //2016-10-23, WT
		pre_vel = _trsmMap["base"]->get_knmatcMdlPtr()->get_spd();
		vel.pose.position.x = (pos_out.pose.position.x - pre_pos.pose.position.x) / dT * 1.0 + 0.0 * pre_vel.pose.position.x;
		vel.pose.position.y = (pos_out.pose.position.y - pre_pos.pose.position.y) / dT * 1.0 + 0.0 * pre_vel.pose.position.y;
		float dangle = tf::getYaw(pos_out.pose.orientation) - tf::getYaw(pre_pos.pose.orientation);
		vel.pose.position.z = atan2(sin(dangle), cos(dangle)) / dT * 1.0 + 0.0 * pre_vel.pose.position.z;
		//std::cout << "vel.pose.position: " << sqrt(pow(vel.pose.position.x, 2) + pow(vel.pose.position.y, 2)) << " " << vel.pose.position.x << " " << vel.pose.position.y << " " << vel.pose.position.z << std::endl;
        //listener.transformPose("ground", vel_in, vel_out); //2016-10-23, WT
		_trsmMap["base"]->get_knmatcMdlPtr()->set_spd(vel);
	}

	void Robot::control()
	{
		//updateGoalNow ();

		// get control for base and arm
        _trsmMap["base"]->getKd(_kd); //2016-10-07, WT  
        _trsmMap["base"]->getKp(_kp); //2016-10-07, WT
        _trsmMap["base"]->getKu(_ku); //2016-10-07, WT
        _trsmMap["base"]->getAngularSpeedMax(_angular_speed_max); //2016-10-07, WT
        _trsmMap["base"]->getxySpeedConstraint(_xy_speed_constraint); //2016-10-07, WT
		_trsmMap["base"]->updateGoal(_goalNow);
		_trsmMap["arm"]->updateGoal(_goalNow);
		ift::Control base_msg = _trsmMap["base"]->control();
		ift::Control arm_msg = _trsmMap["arm"]->control();

		// publish both messages
		if (_base_ctrl_rate < 10)
		  _base_ctrl_rate++;
		else
		{ 
			_base_ctrl_rate = 0;
			_cmd_base_pub.publish(base_msg);
		}
		
		_cmd_arm_pub.publish(arm_msg);
	}

	void Robot::pubTraj()
	{
		// publush target trajectory
		_traj_pub.publish(_navtraj);
	}

	void Robot::pubGoal()
	{
		//publish goal at current time
		PoseStamped p;
        p.header.frame_id = "ground"; // frame
		p.pose.position.x = _goalNow.positions[0];
		p.pose.position.y = _goalNow.positions[1];
		p.pose.position.z = _goalNow.positions[2];
		p.pose.orientation.x = 0;
		p.pose.orientation.y = 0;
		p.pose.orientation.z = sin(_goalNow.positions[3] / 2.0);
		p.pose.orientation.w = cos(_goalNow.positions[3] / 2.0);
		_goal_pub.publish(p);
		
		//publish goal at current time projected on ground (frame map?) //added 2016-05-23 WT
		PoseStamped p_p;
        p_p.header.frame_id = _robotName + "/map"; // frame
		p_p.pose.position.x = _goalNow.positions[0];
		p_p.pose.position.y = _goalNow.positions[1];
		p_p.pose.position.z = 0;
		p_p.pose.orientation.x = 0;
		p_p.pose.orientation.y = 0;
		p_p.pose.orientation.z = sin(_goalNow.positions[3] / 2.0);
		p_p.pose.orientation.w = cos(_goalNow.positions[3] / 2.0);
		_goal_project_pub.publish(p_p);
		
        PoseStamped pp;
		PoseStamped pre_pos = _trsmMap["base"]->get_knmatcMdlPtr()->get_pos();        
        pp.header.frame_id = "ground"; // frame
		pp.pose.position.x = pre_pos.pose.position.x;
		pp.pose.position.y = pre_pos.pose.position.y;
        pp.pose.position.z = _goalNow.positions[2]; //Add 2016-10-19, WT
		pp.pose.orientation.x = 0;
		pp.pose.orientation.y = 0;
		pp.pose.orientation.z = pre_pos.pose.orientation.z;
		pp.pose.orientation.w = pre_pos.pose.orientation.w;
		_pose_pub.publish(pp);        
        //std::cout << "pose: (" <<  pp.pose.position.x << ", " << pp.pose.position.x << ")" << endl; // for test 2016-10-19, WT
	}

	/**
	*/
	void Robot::updateGoalNow()
	{
		// time elapsed from start of simulation
		ros::Duration escape_time = ros::Time::now() - _start_time;
		double secs = escape_time.toSec();

		// update goal given time elapsed
		int goal_indx = static_cast<int>(secs / _dt);
		
		if (goal_indx > _traj.points.size() - 1)
		{
			goal_indx = _traj.points.size() - 1;
			_start_time = ros::Time::now();
		}
		
		_goalNow = _traj.points[goal_indx];
	}
    
     //2016-10-07, WT
    void Robot::loadPIDGains(double kd, double kp, double ku, double xy_speed_constraint, double angular_speed_max)
    {
        _kd = kd;
        _kp = kp;
        _ku = ku;
        _angular_speed_max = angular_speed_max;
        _xy_speed_constraint = xy_speed_constraint;
    }


	void Robot::loadTrajectory(string trajFile)
	{
		//param
		double dt = _dt;
		double eta = _eta;
		double dt_m = _dt_m;
		_traj.joint_names.push_back(_robotName + "_trajectory");

		// variable to record largest pos (raduis) and velocity
		double vel_max = 0;
		double pos_max = 0;

		// open file
		ifstream myfile(trajFile);
		std::string delimiter = " ";
		if (myfile.is_open()) 
		{
			string line;
			getline(myfile, line);
			double data_i = 0; // time index
			while (getline(myfile, line)) 
			{
				// split string
				vector <string> fields;
				boost::split(fields, line, boost::is_any_of(delimiter), boost::token_compress_on);

				// read position and velocity information
				trajectory_msgs::JointTrajectoryPoint p;
				p.positions.resize(4);
				
				for (unsigned int i = 0; i < 3; ++i) 
				{
					p.positions[i] = std::stod(fields[i]);
					if (i == 2) p.positions[i] *= _z_xy_ratio;
				}
				
				p.velocities.resize(4);
				
				for (unsigned int i = 0; i < 3; ++i) 
				{
					p.velocities[i] = std::stod(fields[i + 3]);
					if (i == 2) p.velocities[i] *= _z_xy_ratio;
				}

				double vel_tmp = sqrt(pow(p.velocities[1], 2) + pow(p.velocities[0], 2));
				if (vel_max < vel_tmp) vel_max = vel_tmp;

				double pos_tmp = sqrt(pow(p.positions[1], 2) + pow(p.positions[0], 2));
				if (pos_max < pos_tmp) pos_max = pos_tmp;

				// heading and angular velocity
				p.positions[3] = atan2(p.velocities[1], p.velocities[0]);
				if (_traj.points.size() > 0) 
				{
					double dtheta = p.positions[3] - _traj.points.back().positions[3];
					p.velocities[3] = atan2(sin(dtheta), cos(dtheta)) / dt;
				} 
				else 
				{
					p.velocities[3] = 0;
				}

				// time index for each sample
				p.time_from_start = ros::Duration(dt * data_i);
				++data_i;

				// push each sample to _traj
				_traj.points.push_back(p);
			} //end while
			
			myfile.close();
		}

		// reshape data given eta, dt, and dt_m
		for (auto & e : _traj.points) 
		{
			for (unsigned int i = 0; i < 3 ; ++i) 
			{
				e.positions[i] = e.positions[i] / pos_max * eta; // / vel_max * eta / dt_m * dt;
				e.velocities[i] = e.velocities[i] / pos_max * eta * dt_m / dt;
			}

			// prepare _navtraj
			PoseStamped np;
			np.header.stamp = ros::Time::now();
			np.pose.position.x = e.positions[0];
			np.pose.position.y = e.positions[1];
			np.pose.position.z = e.positions[2];
			np.pose.orientation.x = 0;
			np.pose.orientation.y = 0;
			np.pose.orientation.z = sin(e.positions[3] / 2.0);
			np.pose.orientation.w = cos(e.positions[3] / 2.0);
			_navtraj.poses.push_back(np);
		}
		
		//std::cout<< "test"<<std::endl;
		//std::cout << _traj.points.back() << std::endl;
	}
}; //end of package ift

int main(int argc, char **argv)
{
    // prepare ros
    ros::init(argc, argv, "Satellite_Orbit_System");
    ROS_INFO("IFT SOS control ");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // get parameters from ros launch
    std::string robot_name;
    std::string trajFile;
    double kd, kp, ku, angular_speed_max, xy_speed_constraint; //2016-10-07, WT
    std::cout << "updated" << endl;
    pn.param("trajectory_file", trajFile,
             std::string("/home/ift0015/catkin_ws/src/robot_controller/launch/Orbital_ECI.txt"));
    pn.param("robot_name", robot_name, std::string("robot1"));
    pn.param("kd", kd, 0.2); //2016-10-07, WT
    pn.param("kp", kp, 0.5); //2016-10-07, WT
    pn.param("ku", ku, 0.001); //2016-10-07, WT
    pn.param("angular_speed_max", angular_speed_max, 0.1);  //2016-10-07, WT 
    pn.param("xy_speed_constraint", xy_speed_constraint, 0.1);  //2016-10-07, WT 

    // setup a robot and load trajectory
    //std::cout << "robot_name before: " << robot_name << std::endl; //for test; 20161007, WT
    //std::cout << "trajFile before: " << trajFile << std::endl; //for test; 20161007, WT
    //std::cout << "trajFile before: " << trajFile << std::endl; //for test; 20161007, WT
    ift::Robot robot(robot_name);
    robot.loadTrajectory(trajFile);
    robot.loadPIDGains(kd, kp, ku, angular_speed_max, xy_speed_constraint); //2016-10-07, WT
    //std::cout << "robot_name after: " << robot_name << std::endl; //for test; 20161007, WT
    //std::cout << "trajFile after: " << trajFile << std::endl; //for test; 20161007, WT

    ros::Rate r(10.0);
    while (ros::ok()) 
    {
		ros::spinOnce();

		// control
        robot.updateGoalNow();
        robot.control();

		// display
		robot.pubGoal();
        robot.pubTraj();
        ros::Duration(0.05).sleep();
    }

    return 0;
}
