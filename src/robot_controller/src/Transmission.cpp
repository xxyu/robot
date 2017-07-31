#include "Transmission.h"
#include <string>

// Constructors/Destructors
//  
namespace ift 
{
	Transmission::Transmission () 
	{
		initAttributes();
	}

	Transmission::~Transmission () { }

	Transmission::Transmission(string trsmName, KinematicModelPtr Kptr, ControllerPtr Cptr) 
	: _trsmName(trsmName), _knmatcMdlPtr(Kptr), _ctrlPtr(Cptr)
	{
	  //      
	}

	// Methods
	//  

	/**
	*/
	void Transmission::updateGoal (Goal goal)
	{
		_goal = goal;
	}  

	/** 2016-10-07, WT
	*/
	void Transmission::getKd (double kd)
	{
		_kd = kd;
	}  

	/** 2016-10-07, WT
	*/
	void Transmission::getKp (double kp)
	{
		_kp = kp;
	}  

	/** 2016-10-07, WT
	*/
	void Transmission::getKu (double ku)
	{
		_ku = ku;
	}  

	/** 2016-10-07, WT
	*/
	void Transmission::getxySpeedConstraint (double xy_speed_constraint)
	{
		_xy_speed_constraint = xy_speed_constraint;
	}  

	/** 2016-10-07, WT
	*/
	void Transmission::getAngularSpeedMax (double angular_speed_max)
	{
		_angular_speed_max = angular_speed_max;
	}  

	/**
	* control(): publish control topic
	*/
	ift::Control Transmission::control ()
	{
		return _ctrlPtr->getControl(*_knmatcMdlPtr, _goal, _kd, _kp, _ku, _angular_speed_max, _xy_speed_constraint); //2016-10-07, WT
	}  
  
} //end of package ift
