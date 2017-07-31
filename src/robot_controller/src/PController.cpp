#include "PController.h"
#include "sosdata.h"
#include <cmath>
#include <string>
#include <boost/concept_check.hpp>
#include <tf/transform_datatypes.h>

#define PI 3.14159265

// Constructors/Destructors
//

namespace ift
{
	PController::PController()
	{
		// Constructor
	}

	PController::~PController() { }

	//
	// Methods
	//

	ift::Control PController::getControl(const ift::KinematicModel &knmatcMdl, const Goal &goal, double kd, double kp, double ku, double angular_speed_max, double xy_speed_constraint)
	{
		// get position and velocity
		ift::PoseStamped pos = knmatcMdl.get_pos();
		ift::VelStamped spd = knmatcMdl.get_spd();
		
		// difference between goal position and current position
		ift::VelStamped vel_pos;
		vel_pos.pose.position.x = goal.positions[0] - pos.pose.position.x;
		vel_pos.pose.position.y = goal.positions[1] - pos.pose.position.y;
		//heading difference
		double pos_z = tf::getYaw(pos.pose.orientation);
		double dangle =  goal.positions[3] - pos_z;   
		//std::cout<<"dangle: "<<dangle*180/PI<<std::endl;
		vel_pos.pose.position.z = atan2(sin(dangle), cos(dangle));

		// velocity difference
		ift::VelStamped vel_vel;
		vel_vel.pose.position.x = goal.velocities[0] - spd.pose.position.x;
		vel_vel.pose.position.y = goal.velocities[1] - spd.pose.position.y;
		vel_vel.pose.position.z = goal.velocities[3] - spd.pose.position.z;

		// direction to move
        //std::cout << "kd: " << kd << std::endl; //for test; 20161007, WT
        //std::cout << "kp: " << kp << std::endl; //for test; 20161007, WT
        //std::cout << "ku: " << ku << std::endl; //for test; 20161007, WT
		ift::VelStamped vel;
		vel.header = pos.header;
		// u[n]=kd*deltaV[n]+kp*deltaX[n]+ku*u[n-1], controller structure, commented by Tao Wang, 2016-05-11
		//vel.pose.position.x = vel_vel.pose.position.x * 0.2 + vel_pos.pose.position.x * 0.5 + _target_dir.pose.position.x * 0.001;
		//vel.pose.position.y = vel_vel.pose.position.y * 0.2 + vel_pos.pose.position.y * 0.5 + _target_dir.pose.position.y * 0.001;
		//vel.pose.position.z = vel_vel.pose.position.z * 0.2 + vel_pos.pose.position.z * 0.5 + _target_dir.pose.position.z * 0.001;
		//vel.pose.position.x = vel_vel.pose.position.x * 0.2 + vel_pos.pose.position.x * 0.6 + _target_dir.pose.position.x * 0.001;
		//vel.pose.position.y = vel_vel.pose.position.y * 0.2 + vel_pos.pose.position.y * 0.6 + _target_dir.pose.position.y * 0.001;
		//vel.pose.position.z = vel_vel.pose.position.z * 0.2 + vel_pos.pose.position.z * 0.6 + _target_dir.pose.position.z * 0.001;
        vel.pose.position.x = vel_vel.pose.position.x * kd + vel_pos.pose.position.x * kp + _target_dir.pose.position.x * ku; //2016-10-07, WT
		vel.pose.position.y = vel_vel.pose.position.y * kd + vel_pos.pose.position.y * kp + _target_dir.pose.position.y * ku; //2016-10-07, WT
		vel.pose.position.z = vel_vel.pose.position.z * kd + vel_pos.pose.position.z * kp + _target_dir.pose.position.z * ku; //2016-10-07, WT
		
		// constrain angular speed
        //std::cout << "angular_speed_max: " << angular_speed_max << std::endl; //for test; 20161007, WT
		//double angular_speed_max = 0.1;
		if (fabs(vel.pose.position.z) > angular_speed_max)
			vel.pose.position.z = vel.pose.position.z > 0 ? angular_speed_max : -angular_speed_max;
		
		// constrain velocity in x and y direction
        //std::cout << "xy_speed_constraint: " << xy_speed_constraint << std::endl; //for test; 20161007, WT
		double len = pow(vel.pose.position.x, 2) + pow(vel.pose.position.y, 2);
		//std::cout<<"len:"<<len<<std::endl;
		len = sqrt(len);
		if (len > xy_speed_constraint) 
		{
			vel.pose.position.x = vel.pose.position.x * xy_speed_constraint / len;
			vel.pose.position.y = vel.pose.position.y * xy_speed_constraint / len;
			vel.pose.position.z = vel.pose.position.z;
		}
		
		/*
		std::cout<<"POS:" <<std::endl;
		std::cout<<"    x:    "<<pos.pose.position.x<<std::endl;
		std::cout<<"    y:    "<<pos.pose.position.y<<std::endl;
		std::cout<<"    theta:"<<pos.pose.position.z<<std::endl;

		std::cout<<"Goal:" <<std::endl;
		std::cout<<"    x:    "<<goal.positions[0]<<std::endl;
		std::cout<<"    y:    "<<goal.positions[1]<<std::endl;
		std::cout<<"    theta:"<<goal.positions[2]<<std::endl;

		std::cout<<"DIR:" <<std::endl;
		std::cout<<"    x:    "<<vel.pose.position.x<<std::endl;
		std::cout<<"    y:    "<<vel.pose.position.y<<std::endl;
		std::cout<<"    theta:"<<vel.pose.position.z<<std::endl;
		*/
		
		// update control
		_target_dir = vel;
		return knmatcMdl.inv_kinematicODE(vel); // get control message
	}
}; //end of package ift


