#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include <string>
#include "sosdata.h"
#include "KinematicModel.h"
#include "Controller.h"
#include <boost/shared_ptr.hpp>
using namespace std;

namespace ift 
{
	class Transmission;

	typedef boost::shared_ptr<Transmission> TransmissionPtr;

	/**
	  * class Transmission
	  * 
	  */
	class Transmission
	{
	public:
		// Constructors/Destructors
		//  

		/**
		 * Empty Constructor
		 */
		Transmission ();

		/**
		 * Constructor
		 */
		Transmission(string trsmName, KinematicModelPtr Kptr, ControllerPtr Cptr);

		/**
		 * Empty Destructor
		 */
		virtual ~Transmission ();

		//Methods
		//

		/**
		 */
		void updateGoal (Goal goal) ;
        void getKd (double kd); //2016-10-07, WT 
        void getKp (double kp); //2016-10-07, WT 
        void getKu (double ku); //2016-10-07, WT 
        void getAngularSpeedMax (double angular_speed_max); //2016-10-07, WT 
        void getxySpeedConstraint (double xy_speed_constraint); //2016-10-07, WT 

		/**
		 * control(): publish control topic
		 */
		ift::Control control ();

	private:
		ift::KinematicModelPtr _knmatcMdlPtr; // pointer to kinematic model
		ift::ControllerPtr _ctrlPtr; // pointer to controller 
		ift::TrajectoryPoint _goal; // goal
		string _trsmName;
        double _kd; //2016-10-07, WT 
        double _kp; //2016-10-07, WT 
        double _ku; //2016-10-07, WT 
        double _xy_speed_constraint; //2016-10-07, WT
        double _angular_speed_max; //2016-10-07, WT	  

	public:
		// Private attribute accessor methods
		//  

		/**
		 * Set the value of _knmatcMdl
		 * @param new_var the new value of _knmatcMdl
		 */
		void set_knmatcMdlPtr (ift::KinematicModelPtr new_var)  
		{
			_knmatcMdlPtr = new_var;
		}

		/**
		 * Get the value of _knmatcMdl
		 * @return the value of _knmatcMdl
		 */
		ift::KinematicModelPtr get_knmatcMdlPtr ()   
		{
			return _knmatcMdlPtr;
		}

		/**
		 * Set the value of _ctrl
		 * @param new_var the new value of _ctrl
		 */
		void set_ctrlPtr (ift::ControllerPtr new_var)   
		{
			_ctrlPtr = new_var;
		}

		/**
		 * Get the value of _ctrl
		 * @return the value of _ctrl
		 */
		ift::ControllerPtr get_ctrlPtr ()   
		{
			return _ctrlPtr;
		}

		/**
		 * Set the value of _goal
		 * @param new_var the new value of _goal
		 */
		void set_goal (ift::TrajectoryPoint new_var)   
		{
			_goal = new_var;
		}

		/**
		 * Get the value of _goal
		 * @return the value of _goal
		 */
		ift::TrajectoryPoint get_goal ()   
		{
			return _goal;
		}

		/** 2016-10-07, WT
		 * Set the value of _kd
		 * @param new_var the new value of _kd
		 */
		void set_kd (double new_var)   
		{
			_kd = new_var;
		}

		/** 2016-10-07, WT 
		 * Get the value of _kd
		 * @return the value of _kd
		 */
		double get_kd ()   
		{
			return _kd;
		}

		/** 2016-10-07, WT
		 * Set the value of _kp
		 * @param new_var the new value of _kp
		 */
		void set_kp (double new_var)   
		{
			_kp= new_var;
		}

		/** 2016-10-07, WT
		 * Get the value of _kp
		 * @return the value of _kp
		 */
		double get_kp ()   
		{
			return _kp;
		}

		/** 2016-10-07, WT
		 * Set the value of _ku
		 * @param new_var the new value of _ku
		 */
		void set_ku (double new_var)   
		{
			_ku= new_var;
		}

		/** 2016-10-07, WT
		 * Get the value of _ku
		 * @return the value of _ku
		 */
		double get_ku ()   
		{
			return _ku;
		}

		/** 2016-10-07, WT
		 * Set the value of _angular_speed_max
		 * @param new_var the new value of _angular_speed_max
		 */
		void set_angular_speed_max (double new_var)   
		{
			_angular_speed_max = new_var;
		}

		/** 2016-10-07, WT
		 * Get the value of _angular_speed_max
		 * @return the value of _angular_speed_max
		 */
		double get_angular_speed_max ()   
		{
			return _angular_speed_max;
		}

		/** 2016-10-07, WT
		 * Set the value of _kp
		 * @param new_var the new value of _kp
		 */
		void set_xy_speed_constraint (double new_var)   
		{
			_xy_speed_constraint = new_var;
		}

		/** 2016-10-07, WT
		 * Get the value of _xy_speed_constraint
		 * @return the value of _xy_speed_constraint
		 */
		double get_xy_speed_constraint ()   
		{
			return _xy_speed_constraint;
		}

		/**
		 * Set the value of _trsmName
		 * @param new_var the new value of _trsmName
		 */
		void set_port (string new_var)   
		{
		   _trsmName = new_var;
		}

		/**
		 * Get the value of _trsmName
		 * @return the value of _trsmName
		 */
		string get_port ()   
		{
			return _trsmName;
		}
	  
	private:
		void initAttributes () {}
	};
}; // end of package namespace

#endif // TRANSMISSION_H
