
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>
#include "KinematicModel.h"
#include <boost/shared_ptr.hpp>

namespace ift 
{
	/**
	  * class Controller
	  * 
	  */
	class Controller;

	typedef boost::shared_ptr<Controller> ControllerPtr;

	class Controller
	{
	public:
		// Constructors/Destructors
		//  

		/**
		 * Empty Constructor
		 */
		Controller ();

		/**
		 * Empty Destructor
		 */
		virtual ~Controller ();	 

		/** revised 2016-10-07, WT
		 * @return ift::Control
		 * @param  knmatcMdl
		 */
		virtual ift::Control getControl (const ift::KinematicModel & knmatcMdl, const Goal & goal, double kd, double kp, double ku, double angular_speed_max, double xy_speed_constraint) = 0;

	private:
		// Static Private attributes
		//  

		// Private attributes
		//  

		KinematicModelPtr modelPtr;

	public:
		ift::VelStamped _target_dir;
		// Private attribute accessor methods
		//  
	};
}; // end of package namespace

#endif // CONTROLLER_H
