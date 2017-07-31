
#ifndef DUMMYCONTROLLER_H
#define DUMMYCONTROLLER_H

#include <string>
#include "Controller.h"

namespace ift 
{
	/**
	  * class DummyController
	  * 
	  */
	class DummyController : public Controller
	{
	public:
	  // Constructors/Destructors
	  //  

	  /**
	   * Empty Constructor
	   */
	  DummyController ();

	  /**
	   * Empty Destructor
	   */
	  virtual ~DummyController ();
	  
	  // methods
	  // revised 2016-10-07, WT
	  virtual ift::Control getControl (const ift::KinematicModel & knmatcMdl, const Goal & goal, double kd, double kp, double ku, double angular_speed_max, double xy_speed_constraint) ;
	};
}; // end of package namespace

#endif // DUMMYCONTROLLER_H
