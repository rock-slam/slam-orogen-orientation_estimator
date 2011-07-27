/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */
/**\file Task.cpp
 *
 * This class perform a OROCOS/ROCK component for Attitude (orientation) estimation
 * using the Idirect Kalman Filter (IKF) library.
 * IKF uses the measurement comming from the IMU sensor (Xsens MTi). It predict using
 * gyros read-out and correct using the Accelerometers and the Magnetometers (Yaw only).
 * The IKF is dynamics in the measurement step for accelerometers, therefore the attitude 
 * estimation for Pitcha and Roll is also valid when external acceleration are felt.
 * 
 * This Task uses the StreamAlligner having three Agregators (Inputs)
 * 1. The Xsens angular velocity, acceleration and magnetometers.
 * 2. Xsens quaternion orientation
 * 3. Fog read out of robot Z-axis (Up)
 * 
 * Each Agregator has its own callback function.
 * 
 * @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
 * @date June 2011.
 * @version 1.0.
 */

#include "Task.hpp"

using namespace orientation_estimator;
using namespace filter;

/**
 * @brief Constructor
 * 
 * Memory allocation for Task component and
 * initial values of task flags. 
 * 
 * @return void
 *
 */
Task::Task(std::string const& name)
    : TaskBase(name)
{
  
  xsens_gyros = new Eigen::Matrix <double,NUMAXIS,1>;
  xsens_acc = new Eigen::Matrix <double,NUMAXIS,1>;
  xsens_mag = new Eigen::Matrix <double,NUMAXIS,1>;
  fog_gyros = new Eigen::Matrix <double,NUMAXIS,1>;
  head_q = new Eigen::Quaternion <double>;
  rbs_b_g = new base::samples::RigidBodyState;
  oldeuler = new Eigen::Matrix <double, NUMAXIS, 1>;
  myikf = new ikf;
  
  flag_xsens_time  = false;
  flag_fog_time  = false;
  init_attitude = false;
}

/**
 * @brief Destructor
 * 
 * Free allocated memory by Task class
 *
 *
 * @return void
 *
 */
Task::~Task()
{
  /** Free filter object **/
  delete myikf;
  myikf = NULL;
  
  delete xsens_gyros;
  xsens_gyros = NULL;
  
  delete xsens_acc;
  xsens_acc = NULL;
  
  delete xsens_mag;
  xsens_mag = NULL;
  
  delete fog_gyros;
  fog_gyros = NULL;
  
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
void Task::fog_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &fog_samples_sample)
{
  
  Eigen::Matrix <double, NUMAXIS, 1> euler;
  double fog_dt;
  
  if (init_attitude == true)
  {
    if (flag_fog_time == false)
    {
      fog_time = (double)fog_samples_sample.time.toMilliseconds();
      (*fog_gyros) = fog_samples_sample.gyro;
      flag_fog_time = true;
    }
    else
    {     
      fog_dt = ((double)fog_samples_sample.time.toMilliseconds() - fog_time)/1000.00;
      fog_time = (double)fog_samples_sample.time.toMilliseconds();
      
      (*fog_gyros) = fog_samples_sample.gyro;
      
      /** Substract the Earth Rotation from the FOG output */
      myikf->SubstractEarthRotation (fog_gyros, head_q, _latitude.value());
      
      /** Only in the Yaw (Z-axis) are the FOG angular velocity ) */
      (*fog_gyros)[0] = 0.00;
      (*fog_gyros)[1] = 0.00;
      (*fog_gyros)[2] = (*fog_gyros)[2] - FOGBIAS;
      
      Task::PropagateHeadingQuaternion (head_q, fog_gyros, fog_dt);
      myikf->Quaternion2Euler(head_q, &euler);
      std::cout << "Heading(FOG): "<< euler[2]*R2D <<"\n";
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
void Task::xsens_orientationCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xsens_orientation_sample)
{
  
  Eigen::Quaternion <double> attitude (xsens_orientation_sample.orientation.w(), xsens_orientation_sample.orientation.x(),
    xsens_orientation_sample.orientation.y(), xsens_orientation_sample.orientation.z());
   Eigen::Matrix <double, NUMAXIS, 1> euler;
   
   if (init_attitude == false)
   {
     std::cout << "******** Init Attitude *******\n";
     /** Eliminate the Magnetic declination from the initial attitude quaternion **/
     myikf->CorrectMagneticDeclination (&attitude, _magnetic_declination.value(), _magnetic_declination_mode.value());
     
     /** Set the initial attitude quaternion of the IKF **/
     myikf->setAttitude (&attitude);
     init_attitude = true;
     
     /** Fog quaternion initial value is also the IKF initial quaternion **/
     (*head_q) = myikf->getAttitude ();
     
     myikf->Quaternion2Euler(&attitude, &euler);
     std::cout << "Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
     std::cout << "(Roll, Pitch, Yaw)\n"<< euler*R2D <<"\n";
     std::cout << "**********************\n";
     (*oldeuler) = euler;
     
   }
   
   return;
}

/**
 * @brief Xsens callback function
 * 
 * This function performs the callback of the StreamAlligner for the Xsens
 * angular velocity, accelerometers and magnetometers.
 * 
 * This function has the prediction, mesurement and correction steps of the IKF.
 * The quaternion output is combined with the Heading angle coming from the 
 * FOG Callback function and store in the Output port.
 *
 * @author Javier Hidalgo Carrio.
 *
 * @param[in] &ts timestamp
 * @param[in] &xsens_samples_sample Xsens sensor values.
 *
 * @return void
 *
 */
void Task::xsens_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &xsens_samples_sample)
{
  Eigen::Quaternion <double> qb_g;
  Eigen::Quaternion <double> auxq;
  Eigen::Matrix <double, NUMAXIS, 1> euler;
  Eigen::Matrix <double, NUMAXIS, 1> heading;
  double xsens_dt;
  
  if (init_attitude == true)
  {
  
    if (flag_xsens_time == false)
    {
      xsens_time = (double)xsens_samples_sample.time.toMilliseconds();
      (*xsens_gyros) = xsens_samples_sample.gyro;
      myikf->setOmega (xsens_gyros);
      flag_xsens_time = true;
    }
    else
    {
      xsens_dt = ((double)xsens_samples_sample.time.toMilliseconds() - xsens_time)/1000.00;
      xsens_time = (double)xsens_samples_sample.time.toMilliseconds();

      /** Copy the sensor information */
      (*xsens_gyros) = xsens_samples_sample.gyro;
      (*xsens_acc) = xsens_samples_sample.acc;
      (*xsens_mag) = xsens_samples_sample.mag;

      /** Substract the Earth Rotation from the gyros output */
      qb_g = myikf->getAttitude(); /** Rotation with respect to the geographic frame (North-Up-West) */
      myikf->SubstractEarthRotation (xsens_gyros, &qb_g, _latitude.value());
      
      /** Perform the Indirect Kalman Filter */
      myikf->predict (xsens_gyros, xsens_dt);
      myikf->update (xsens_acc, xsens_mag);
    }
    
    /** Get Attitude en Euler **/
    euler = myikf->getEuler();
    
    /** Out in the Outports  */
    rbs_b_g->time = xsens_samples_sample.time; /** Set the timestamp */
    
    /** Orientation (Pitch and Roll from IKF, Yaw from FOG) */
    myikf->Quaternion2Euler(head_q, &heading);
    euler[2] = heading[2];
    myikf->Quaternion2Euler(&(auxq), &euler);
    
    /** Copy to the rigid_body_state **/
    rbs_b_g->orientation = (base::Orientation) auxq;
    
    /** Write the Angular velocity (as the different between two orientations in radians)*/
    rbs_b_g->angular_velocity = (euler - (*oldeuler));
    
    /** Store the euler angle for the next iteration **/
    (*oldeuler)= euler;
    
    /** Write in the OROCOS Ports */
    _attitude_b_g.write((*rbs_b_g));
    
    
    
  }
  return;
}

/**
 * @brief This function computes the discrete-time propagation of a quaternion
 * 
 * Quaternion propagation using the angular velocities as input. The function
 * computes the function 28, 29, 30a and 30b of the paper
 * J. L. Crassidis and F. L. Markley "Unscented filtering for spacecraft attitude estimation"
 *
 * @author Javier Hidalgo Carrio.
 *
 * @param[in, out] *quat pointer to the quaternion to propagate.
 * @param[in] *angvelo angular velocity vector.
 * @param[in] dt delta time is seconds.
 *
 * @return void
 *
 */
void Task::PropagateHeadingQuaternion( Eigen::Quaternion <double> *quat, Eigen::Matrix<double, NUMAXIS , 1>* angvelo, double dt)
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


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
  
  Eigen::Matrix <double,NUMAXIS,NUMAXIS> Ra; /**< Measurement noise convariance matrix for acc */
  Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rg; /**< Measurement noise convariance matrix for gyros */
  Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rm; /**< Measurement noise convariance matrix for mag */
  double latitude = (double)_latitude.value();
  double altitude = (double)_altitude.value();
  double g;
  
  /** Fill the matrices **/
  Ra = Matrix<double,NUMAXIS,NUMAXIS>::Zero();
  Ra(0,0) = pow(XSENSRWACCX,2)/_delta_time.value();
  Ra(1,1) = pow(XSENSRWACCY,2)/_delta_time.value();
  Ra(2,2) = pow(XSENSRWACCZ,2)/_delta_time.value();
  
  Rg = Matrix<double,NUMAXIS,NUMAXIS>::Zero();
  Rg(0,0) = pow(XSENSRWGYROX,2)/_delta_time.value();
  Rg(1,1) = pow(XSENSRWGYROY,2)/_delta_time.value();
  Rg(2,2) = pow(XSENSRWGYROZ,2)/_delta_time.value();

  Rm = Matrix<double,NUMAXIS,NUMAXIS>::Zero();
  Rm(0,0) = pow(XSENSRWMAGX,2)/_delta_time.value();
  Rm(1,1) = pow(XSENSRWMAGY,2)/_delta_time.value();
  Rm(2,2) = pow(XSENSRWMAGZ,2)/_delta_time.value();
	
  /** Gravitational value according to the location **/
  g = myikf->GravityModel (latitude, altitude);
  
  /** Output port frames information */
  rbs_b_g->sourceFrame = "Body_Frame"; /** The body Frame in Source  */
  rbs_b_g->targetFrame = "Geographic_Frame (North-West-Up)"; /** The Geographic Frame in Target */
  
  /** Initial values for the IKF **/
  myikf->Init(&Ra, &Rg, &Rm, g, (double)_dip_angle.value());
  
  return TaskBase::configureHook();;  
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

