#ifndef PCONTROLLER_H
#define PCONTROLLER_H

#include <string>
#include "Controller.h"

namespace ift 
{
	/**
	  * class PController
	  * 
	  */
	class PController : public Controller
	{
	public:
		// Constructors/Destructors
		//  

		/**
		* Empty Constructor
		*/
		PController ();

		/**
		* Empty Destructor
		*/
		virtual ~PController ();

	public:	  
		// methods
		// revised 2016-10-07, WT
		virtual ift::Control getControl (const ift::KinematicModel & knmatcMdl, const Goal & goal, double kd, double kp, double ku, double angular_speed_max, double xy_speed_constraint) ;
	};
}; // end of package namespace

#endif // PCONTROLLER_H
