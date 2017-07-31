#include "BaseKinematicModel.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <cmath>
#include <vector>
#include <tf/transform_datatypes.h>

#define PI 3.14159265

namespace ift
{
	// Constructors/Destructors
	BaseKinematicModel::BaseKinematicModel()
	{
		// Constructor
	}

	BaseKinematicModel::BaseKinematicModel(unsigned int dim)
	{
		// initialize position and speed
		_dimension = dim;
		_pos.header.stamp = ros::Time::now();
		_spd.header.stamp = ros::Time::now();
		
		_pos.pose.position.x = 0;
		_pos.pose.position.y = 0;
		_pos.pose.position.z = 0;
		_pos.pose.orientation.w = 1;
		_pos.pose.orientation.z = 0;

		_spd.pose.position.x = 0;
		_spd.pose.position.y = 0;
		_spd.pose.position.z = 0;
	}

	BaseKinematicModel::~BaseKinematicModel() { }

	//
	// Methods
	//

	/**
	 * @param  ctrl
	 * @param  pos
	 * @param  accel
	 */
	void BaseKinematicModel::kinematicODE(ift::Control ctrl, geometry_msgs::Vector3 pos, ift::AccelStamped accel)
	{
		//not required here
	}


	/**
	 * @return ift::Control
	 * @param  vel
	 * @param  accel
	 */
	ift::Control BaseKinematicModel::inv_kinematicODE(ift::VelStamped vel) const
	{
		// calculate u = M*v
	  
		// prepare v
		using namespace boost::numeric;
		ublas::vector<double> v(_dimension);
		v[0] = vel.pose.position.x;
		v[1] = vel.pose.position.y;
		v[2] = vel.pose.position.z;
		
		// prepare M
		double theta = tf::getYaw(_pos.pose.orientation); // + PI / 6.0; //Minus 5*PI/6, 2016-05-19, WT
    //std::cout<<"theta: "<<theta*180/PI<<std::endl; //for debug, 2016-05-16, WT
		std::vector<double> vec {sin(theta), sin(PI / 3.0 - theta), -sin(theta + PI / 3.0),
								 -cos(theta), cos(PI / 3.0 - theta), cos(theta + PI / 3.0),
								 _L, _L, _L
								};
		ublas::unbounded_array<double> uvec(_dimension * _dimension);
		for (unsigned int i = 0; i < _dimension * _dimension; ++i) 
		{
			uvec[i] = vec[i];
		}
		ublas::matrix<double> M(_dimension, _dimension, uvec);

		// calculate u = M*v
		//ublas::vector<double> u = -ublas::prod(ublas::trans(M), v);
		ublas::vector<double> u = ublas::prod(ublas::trans(M), v); //for robot5 the direction of u, v, w are as in the report

		//std::cout<<M<<std::endl;
		//std::cout<<v<<std::endl;
		//std::cout<<u<<std::endl;
		
		// return control
		ift::Control result;
		result.x = u[0];
		result.y = u[1];
		result.z = u[2];
		//result.x = 0.0; //test for which is u, 2016-05-20, WT
		//result.y = 0.0; //test for which is u, 2016-05-20, WT
		//result.z = 0.0; //test for which is u, 2016-05-20, WT
		return result;
	}
}; // end of package ift

