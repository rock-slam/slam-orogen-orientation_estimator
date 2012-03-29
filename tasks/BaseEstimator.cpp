/* Generated from orogen/lib/orogen/templates/tasks/BaseEstimator.cpp */
/**\file BaseEstimator.cpp
 *
 * This class perform a OROCOS/ROCK component for Attitude (orientation) estimation
 * 
 * 
 * This BaseEstimator uses the StreamAlligner having two Agregators (Inputs)
 *
 * 1. Xsens quaternion orientation
 * 2. Fog read out of robot Z-axis (Up)
 * 
 * Each Agregator has its own callback function.
 * 
 * @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
 * @date June 2011.
 * @version 1.0.
 */

#include "BaseEstimator.hpp"

using namespace orientation_estimator;

/**
 * @brief Constructor
 * 
 * Memory allocation for BaseEstimator component and
 * initial values of task flags. 
 * 
 * @return void
 *
 */
BaseEstimator::BaseEstimator(std::string const& name)
    : BaseEstimatorBase(name)
{

  euler = new Eigen::Matrix <double,NUMAXIS,1>;
  head_q = new Eigen::Quaternion <double>;
  rbs_b_g = new base::samples::RigidBodyState;
  oldeuler = new Eigen::Matrix <double, NUMAXIS, 1>;
  
  flag_fog_time  = false;
  init_attitude = false;
}

/**
 * @brief Destructor
 * 
 * Free allocated memory by BaseEstimator class
 *
 *
 * @return void
 *
 */
BaseEstimator::~BaseEstimator()
{
  /** Free filter object **/
  delete euler;
  euler = NULL;
  
  delete head_q;
  head_q = NULL;
  
  delete oldeuler;
  oldeuler = NULL;
  
  delete rbs_b_g;
  rbs_b_g = NULL;
}

/**
 * @brief Fiber Optic Gyro callback function
 * 
 * This function performs the callback of the StreamAlligner for the DSP-3000
 * FOG. Moreover it performs the gyros integration using quaternion.
 * Following this quaternion integration the drift is 6 degrees/hour.
 *
 * @author Javier Hidalgo Carrio.
 *
 * @param[in] &ts timestamp
 * @param[in] &fog_samples_sample FOG angular velocty
 *
 * @return void
 *
 */
void BaseEstimator::fog_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &fog_samples_sample)
{
  
  Eigen::Matrix <double,NUMAXIS,1> fog_gyros; /**< Angular velocity for the FOG */ 
  
  if (init_attitude == true)
  {
    if (flag_fog_time == false)
    {
      fog_time = (double)fog_samples_sample.time.toMilliseconds();
      fog_gyros = fog_samples_sample.gyro;
      flag_fog_time = true;
    }
    else
    {     
      fog_dt = ((double)fog_samples_sample.time.toMilliseconds() - fog_time)/1000.00;
      fog_time = (double)fog_samples_sample.time.toMilliseconds();
      
      fog_gyros = fog_samples_sample.gyro;
      
      /** Substract the Earth Rotation from the FOG output */
      SubstractEarthRotation (&fog_gyros, head_q, _latitude.value());
      
      /** Only in the Yaw (Z-axis) are the FOG angular velocity ) */
      fog_gyros[0] = 0.00;
      fog_gyros[1] = 0.00;
      fog_gyros[2] = fog_gyros[2] - _gbiasof.get()[2];
      
      /** This integration step is good when dog_dt is small **/
      BaseEstimator::PropagateHeadingQuaternion (head_q, &fog_gyros, fog_dt);
      
      /** Get the Yaw from the FOG integration **/
      (*euler)[2] = head_q->toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
//        std::cout << "fog_gyros: "<< fog_gyros[2] <<"\n";
//       std::cout << "Heading(FOG): "<< (*euler)[2]*(180.00/PI) <<"\n";

    }
  }
  
  return;
}

/**
 * @brief Xsens orientation callback function
 * 
 * This function performs the callback of the StreamAlligner for the 
 * orientation coming from the Xsens quaternion
 * 
 * This function is used to initially set the quaternion.
 * Therefore, it initializes the original value for the orientation (init quaternion).
 *
 * @author Javier Hidalgo Carrio.
 *
 * @param[in] &ts timestamp
 * @param[in] &xsens_samples_sample Xsens sensor quaternion.
 *
 * @return void
 *
 */
void BaseEstimator::xsens_orientationCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xsens_orientation_sample)
{
  
  Eigen::Quaternion <double> attitude (xsens_orientation_sample.orientation.w(), xsens_orientation_sample.orientation.x(),
    xsens_orientation_sample.orientation.y(), xsens_orientation_sample.orientation.z());
  
   if (init_attitude == false)
   {
	std::cout << "******** Init Attitude BaseEstimator *******\n";
	/** Eliminate the Magnetic declination from the initial attitude quaternion **/
	CorrectMagneticDeclination (&attitude, _magnetic_declination.value(), _magnetic_declination_mode.value());
	(*head_q) = attitude;
	init_attitude = true;
	
	(*euler)[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
	(*euler)[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	(*euler)[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
	
	/** Store the euler angle for the first time **/
	(*oldeuler)= (*euler);

	std::cout << "Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
	std::cout << "(Roll, Pitch, Yaw)\n"<< (*euler)*(180.00/PI) <<"\n";
	std::cout << "**********************\n";
   }
   else
   {
	/** Get Picth and Roll from Xsens **/
	(*euler)[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
	(*euler)[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL

	std::cout << "BaseEstimator Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
	std::cout << "(Roll, Pitch, Yaw)\n"<< (*euler)*(180.00/PI) <<"\n";
	std::cout << "**********************\n";

	/** Write the Angular velocity (as the different between two orientations in radians)*/
	rbs_b_g->angular_velocity = ((*euler) - (*oldeuler))/fog_dt;
	
	/** Store the euler angle for the next iteration **/
	(*oldeuler)= (*euler);

	/** Convert attitude to quaternion **/
	rbs_b_g->orientation =  Eigen::Quaternion <double> (Eigen::AngleAxisd((*euler)[2], Eigen::Vector3d::UnitZ())*
			Eigen::AngleAxisd((*euler)[1], Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd((*euler)[0], Eigen::Vector3d::UnitX()));
	
	/** Also update the quaternion used by the fog callback function **/
	(*head_q) = rbs_b_g->orientation;
	
	/** Out in the Outports  */
	rbs_b_g->time = xsens_orientation_sample.time; //base::Time::now(); /** Set the timestamp */

	/** Write in the output port **/
	_attitude_b_g.write((*rbs_b_g));
    }
   
   return;
}



/**
 * @brief This function computes the discrete-time propagation of a quaternion
 */
void BaseEstimator::PropagateHeadingQuaternion( Eigen::Quaternion <double> *quat, Eigen::Matrix<double, NUMAXIS , 1>* angvelo, double dt)
{
  register int j, l;
  double auxvar;
  Eigen::Matrix <double, NUMAXIS, NUMAXIS> Crossproduct;
  Eigen::Matrix <double, QUATERSIZE, QUATERSIZE> Omega;
  Eigen::Matrix< double, QUATERSIZE , 1  > q;
  Eigen::Matrix <double, NUMAXIS, NUMAXIS> Upper;
  Eigen::Matrix <double, NUMAXIS, 1> psi;
  
  Crossproduct = Eigen::Matrix <double, NUMAXIS, NUMAXIS>::Zero();
  Omega = Eigen::Matrix <double, QUATERSIZE, QUATERSIZE>::Zero();
  
  /** If angular velocity is not zero **/
  if ((double)(*angvelo).norm() != 0.00)
  {
    /** Copy the quaternion **/
    q(3) = quat->w();
    q(0) = quat->x();
    q(1) = quat->y();
    q(2) = quat->z();

    /** Psi vector calculation **/
    auxvar = sin (0.5*(*angvelo).norm()*dt);
    psi(0) = (double) auxvar * ((*angvelo)(0)/(*angvelo).norm());
    psi(1) = (double) auxvar * ((*angvelo)(1)/(*angvelo).norm());
    psi(2) = (double) auxvar * ((*angvelo)(2)/(*angvelo).norm());

    /** Create the cross-product matrix from the angular velocity **/
    Crossproduct (0,1) = -psi(2);
    Crossproduct (1,0) = psi(2);
    Crossproduct (0,2) = psi(1);
    Crossproduct (2,0) = -psi(1);
    Crossproduct (1,2) = -psi(0);
    Crossproduct (2,1) = psi(0);

    /** Upper matrix **/
    Upper = (double)cos(0.5*(*angvelo).norm()*(dt)) * Eigen::Matrix <double, NUMAXIS, NUMAXIS>::Identity() - Crossproduct;

    /** Create the omega transition matrix **/
    for (j=0; j<QUATERSIZE; j++) /** Columns **/
    {
      for (l=0; l<QUATERSIZE; l++) /** Rows **/
      {
	if (l<3)
	{
	  if (j<3)
	    Omega (l,j) = Upper (l,j);
	  else
	    Omega (l,j) = psi (l);
	}
	else
	{
	  if (j<3)
	    Omega (l,j) = -psi (j);
	  else
	    Omega (l,j) = (double)cos(0.5*(*angvelo).norm()*(dt));
	}
      }
    }

    /** Propagate forward in time, y = (A) x + y **/
    q = Omega*q;
    
    /** Store the update quaternion in the argument quaternion **/
    quat->w() = q(3);
    quat->x() = q(0);
    quat->y() = q(1);
    quat->z() = q(2);
    
  }

  //std::cout <<"q: "<<(*q)<<"\n";
  
  return;
}

/**
* @brief Substract the Earth rotation from the gyroscopes readout
*/
void BaseEstimator::SubstractEarthRotation(Eigen::Matrix <double, NUMAXIS, 1> *u, Eigen::Quaternion <double> *qb_g, double latitude)
{
    Eigen::Matrix <double, NUMAXIS, 1> v (EARTHW*cos(latitude), 0, EARTHW*sin(latitude)); /**< vector of earth rotation components expressed in the geografic frame according to the latitude **/

    /** Compute the v vector expressed in the body frame **/
    v = (*qb_g) * v;

    /** Subtract the earth rotation to the vector of inputs (u = u-v**/
    (*u)  = (*u) - v;
    
    return;
}

/**
* @brief This computes the theoretical gravity value according to the WGS-84 ellipsoid earth model.
*/
double BaseEstimator::GravityModel(double latitude, double altitude)
{
    double g; /**< g magnitude at zero altitude **/

    /** Nominal Gravity model **/
    g = GWGS0*((1+GWGS1*pow(sin(latitude),2))/sqrt(1-pow(ECC,2)*pow(sin(latitude),2)));

    /** Gravity affects by the altitude (aprox the value r = Re **/
    g = g*pow(Re/(Re+altitude), 2);

    std::cout<<"Theoretical gravity for this location (WGS-84 ellipsoid model): "<< g<<" [m/s^2]\n";

    return g;

}

/**
* @brief Correct the magnetic declination of the North 
*/
int BaseEstimator::CorrectMagneticDeclination(Eigen::Quaternion< double >* quat, double magnetic_declination, int mode)
{
    Eigen::Matrix <double, NUMAXIS, 1> euler;
	
    euler[2] = quat->toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    euler[1] = quat->toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
    euler[0] = quat->toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
    
    if (mode == EAST)
    {
    std::cout << "[EAST] magnetic declination\n";
    euler[2] -= magnetic_declination; /** Magnetic declination is positive **/
    }
    else if (mode == WEST)
    {
    std::cout << "[WEST] magnetic declination\n";
    euler[2] += magnetic_declination; /** Magnetic declination is negative **/
    }
    else
    {
    std::cerr << "[ERROR] In the correction of the magnetic declination\n";
    return ERROR;
    }
    
    *quat = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())*
			Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()));
    
    return OK;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BaseEstimator.hpp for more detailed
// documentation about them.

bool BaseEstimator::configureHook()
{
  double latitude = (double)_latitude.value();
  double altitude = (double)_altitude.value();
  double g;
  
 
  /** Gravitational value according to the location **/
  g = GravityModel (latitude, altitude);
  
  /** Output port frames information */
  rbs_b_g->sourceFrame = "Body_Frame"; /** The body Frame in Source  */
  rbs_b_g->targetFrame = "Geographic_Frame (North-West-Up)"; /** The Geographic Frame in Target */
  
  return BaseEstimatorBase::configureHook();;  
}
bool BaseEstimator::startHook()
{
    if (! BaseEstimatorBase::startHook())
        return false;
    return true;
}
void BaseEstimator::updateHook()
{
    BaseEstimatorBase::updateHook();
    
}
void BaseEstimator::errorHook()
{
    BaseEstimatorBase::errorHook();
}
void BaseEstimator::stopHook()
{
    BaseEstimatorBase::stopHook();
}
void BaseEstimator::cleanupHook()
{
    BaseEstimatorBase::cleanupHook();
}

