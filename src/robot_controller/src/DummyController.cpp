#include "DummyController.h"
#include "sosdata.h"
#include <cmath>
#include <string>
#include <boost/concept_check.hpp>
#include <tf/transform_datatypes.h>

namespace ift
{
	// Constructors/Destructors
	//  

	DummyController::DummyController () 
	{
		//
	}

	DummyController::~DummyController () { }

	//  
	// Methods
	// revised 2016-10-07, WT  

	ift::Control DummyController::getControl (const ift::KinematicModel & knmatcMdl, const Goal & goal, double kd, double kp, double ku, double angular_speed_max, double xy_speed_constraint)
	{
		ift::VelStamped vel;
		vel.pose.position.x = goal.positions[2];
		vel.pose.position.y = goal.velocities[2];
		//vel.pose.position.x = 0; // for test to avoid kadakada sound, Tao Wang, 2016-05-17
		//vel.pose.position.y = 0; // for test to avoid kadakada sound, Tao Wang, 2016-05-17
		return knmatcMdl.inv_kinematicODE(vel);
	}
}; //end of package ift


