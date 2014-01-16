/* Generated from orogen/lib/orogen/templates/tasks/IKFEstimator.cpp */
/**\file IKFEstimator.cpp
 *
 * This class perform a OROCOS/ROCK component for Attitude (orientation) estimation
 * using the Idirect Kalman Filter (IKF) library.
 * IKF uses the measurement comming from the IMU sensor (Xsens MTi). It predict using
 * gyros read-out and correct using the Accelerometers and the Magnetometers (Yaw only).
 * The IKF is dynamics in the measurement step for accelerometers, therefore the attitude 
 * estimation for Pitcha and Roll is also valid when external acceleration are felt.
 * 
 * This IKFEstimator uses the StreamAlligner having three Agregators (Inputs)
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

#include "IKFEstimator.hpp"
#include "BaseEstimator.hpp"
#include <base/angle.h>

using base::Angle;
using namespace orientation_estimator;
using namespace filter;

/**
 * @brief Constructor
 * 
 * Memory allocation for IKFEstimator component and
 * initial values of task flags. 
 * 
 * @return void
 *
 */
IKFEstimator::IKFEstimator(std::string const& name)
    : IKFEstimatorBase(name)
{

  accidx = 0;
  imu_gyros = new Eigen::Matrix <double,NUMAXIS,1>;
  imu_acc = new Eigen::Matrix <double,NUMAXIS,1>;
  imu_mag = new Eigen::Matrix <double,NUMAXIS,1>;
  fog_gyros = new Eigen::Matrix <double,NUMAXIS,1>;
  rbs_b_g = new base::samples::RigidBodyState();
  rbs_b_g->invalidate();
  oldeuler = new Eigen::Matrix <double, NUMAXIS, 1>;
  init_acc = new Eigen::Matrix <double,NUMAXIS,NUMBER_INIT_ACC>;
  myikf = new filter::Ikf<double, true, false>;
  fogikf = new filter::Ikf<double, true, false>;

  backup = new base::samples::IMUSensors;
  
  flag_imu_time  = false;
  flag_fog_time  = false;
  init_attitude = false;
}

/**
 * @brief Destructor
 * 
 * Free allocated memory by IKFEstimator class
 *
 *
 * @return void
 *
 */
IKFEstimator::~IKFEstimator()
{
  /** Free filter objects **/
  delete myikf;
  myikf = NULL;
  
  delete fogikf;
  fogikf = NULL;
  
  delete imu_gyros;
  imu_gyros = NULL;
  
  delete imu_acc;
  imu_acc = NULL;
  
  delete imu_mag;
  imu_mag = NULL;
  
  delete fog_gyros;
  fog_gyros = NULL;
 
  delete oldeuler;
  oldeuler = NULL;
  
  delete rbs_b_g;
  rbs_b_g = NULL;
  
  delete init_acc;
  init_acc = NULL;
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
void IKFEstimator::fog_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &fog_samples_sample)
{
  Eigen::Quaternion <double> qb_g;
  Eigen::Matrix <double, NUMAXIS, 1> euler;
  (*fog_gyros) = fog_samples_sample.gyro;
    
  if (init_attitude == true)
  {
    if (flag_fog_time == false)
    {
      fog_time = (double)fog_samples_sample.time.toMilliseconds();      
      
      /** Becasue the rest of values are nan is a problem to definen the omega of
       * the filter therefore they need to be set to zero the axis withour values **/
      if ((*fog_gyros)[0] != (*fog_gyros)[0]) //If NaN?
	(*fog_gyros)[0] = 0.00;
      if ((*fog_gyros)[1] != (*fog_gyros)[1]) //If NaN?
	(*fog_gyros)[1] = 0.00;
      if ((*fog_gyros)[2] != (*fog_gyros)[2]) //If NaN?
	(*fog_gyros)[2] = 0.00;
      fogikf->setOmega (*fog_gyros);
      flag_fog_time = true;
    }
    else
    {     
      fog_dt = ((double)fog_samples_sample.time.toMilliseconds() - fog_time)/1000.00;
      fog_time = (double)fog_samples_sample.time.toMilliseconds();
    
      qb_g = fogikf->getAttitude();
      
      /** Commented because the integration is done at the xsens callback **/
      /** Substract the Earth Rotation from the FOG output */
      BaseEstimator::SubstractEarthRotation (fog_gyros, &qb_g, _latitude.value());
      
      /** Only in the Yaw (Z-axis) are the FOG angular velocity) */
      (*fog_gyros)[0] = 0.00;
      (*fog_gyros)[1] = 0.00;
      (*fog_gyros)[2] = (*fog_gyros)[2] - _gbiasof.get()[2];
      
      fogikf->predict (*fog_gyros, fog_dt);
      

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
 * @param[in] &imu_samples_sample Xsens sensor quaternion.
 *
 * @return void
 *
 */
void IKFEstimator::imu_orientationCallback(const base::Time &ts, const ::base::samples::RigidBodyState &imu_orientation_sample)
{
  
  Eigen::Quaternion <double> attitude (imu_orientation_sample.orientation.w(), imu_orientation_sample.orientation.x(),
    imu_orientation_sample.orientation.y(), imu_orientation_sample.orientation.z());
   Eigen::Matrix <double, NUMAXIS, 1> euler;
   

   if (init_attitude == false)
   {
     std::cout << "******** Init Attitude IKFEstimator *******\n";
     /** Eliminate the Magnetic declination from the initial attitude quaternion **/
     BaseEstimator::CorrectMagneticDeclination (&attitude, _magnetic_declination.value(), _magnetic_declination_mode.value());
     
     /** Set the initial attitude quaternion of the IKF **/
     myikf->setAttitude (attitude);
     fogikf->setAttitude (attitude);
     init_attitude = true;    
    
     euler[2] = base::getEuler(attitude)[0];//YAW
     euler[1] = base::getEuler(attitude)[1];//PITCH
     euler[0] = base::getEuler(attitude)[2];//ROLL
     
     std::cout << "Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
     std::cout << "(Roll, Pitch, Yaw)\n"<< Angle::rad2Deg(euler.x()) << "," << Angle::rad2Deg(euler.y()) << "," << Angle::rad2Deg(euler.z()) << "\n";
     std::cout << "**********************\n";
     
     (*oldeuler) = euler;
     
   }
   
   return;
}

/**
 * @brief IMU callback function
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
 * @param[in] &imu_samples_sample Xsens sensor values.
 *
 * @return void
 *
 */
void IKFEstimator::imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{
  Eigen::Quaternion <double> qb_g;
  Eigen::Quaternion <double> auxq;
  Eigen::Matrix <double, NUMAXIS, 1> euler;
  Eigen::Matrix <double, NUMAXIS, 1> heading;
  
  if (init_attitude == true)
  {
    /** Copy the sensor information */
    (*imu_gyros) = imu_samples_sample.gyro;          
    (*imu_acc) = imu_samples_sample.acc;
    (*imu_mag) = imu_samples_sample.mag;
  
    if (flag_imu_time == false)
    {
      imu_time = (double)imu_samples_sample.time.toMilliseconds();
      myikf->setOmega (*imu_gyros);
      flag_imu_time = true;
    }
    else
    {
      imu_dt = ((double)imu_samples_sample.time.toMilliseconds() - imu_time)/1000.00;
      imu_time = (double)imu_samples_sample.time.toMilliseconds();

//       std::cout << "Gyros(rad/sec)\n"<< (*imu_gyros)[0]<<","<< (*imu_gyros)[1]<<","<< (*imu_gyros)[2]<<"\n";
//       std::cout<<"Delta_time: "<<imu_dt<<"\n";
      
      /** Substract the Earth Rotation from the gyros output */
      qb_g = myikf->getAttitude(); /** Rotation with respect to the geographic frame (North-Up-West) */
      BaseEstimator::SubstractEarthRotation (imu_gyros, &qb_g, _latitude.value());
      
//       std::cout << "Gyros-Earth(rad/sec)\n"<< (*imu_gyros)[0]<<","<< (*imu_gyros)[1]<<","<< (*imu_gyros)[2]<<"\n";
      
      /** Orientation (Pitch and Roll from IKF, Yaw from FOG) */
      if (_fog_samples.connected())
	  (*imu_gyros)[2] = 0.00;
   
      /** Perform the Indirect Kalman Filter */
      myikf->predict (*imu_gyros, imu_dt);
      myikf->update (*imu_acc, true, *imu_mag, _use_magnetometers.value());
    }

    /** Get Attitude en Euler **/
    euler[2] = base::getEuler(myikf->getAttitude())[0];//YAW
    euler[1] = base::getEuler(myikf->getAttitude())[1];//PITCH
    euler[0] = base::getEuler(myikf->getAttitude())[2];//ROLL

    /** Out in the Outports  */
    rbs_b_g->time = imu_samples_sample.time; //base::Time::now(); /** Set the timestamp */
    
    if (_fog_samples.connected())
    {
	euler[2] = base::getEuler(fogikf->getAttitude())[0];//YAW
    }
    
//     std::cout << "IKFEstimator\n";
//     std::cout << "(Roll, Pitch, Yaw)\n"<< Angle::rad2Deg(euler[0])<<","<< Angle::rad2Deg(euler[1])<<","<< Angle::rad2Deg(euler[2]) <<"\n";
//     std::cout << "**********************\n";
     
    auxq = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
 			    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
 			    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
    
    /** Copy to the rigid_body_state **/
    rbs_b_g->orientation = (base::Orientation) auxq;
    
    /** Copy to covariance to the rigid_body_state **/
    rbs_b_g->cov_orientation = myikf->getCovariance().block<NUMAXIS, NUMAXIS>(0,0);
    
    /** Also update the quaternion used by the fog callback function **/
    if (_fog_samples.connected())
    {
	fogikf->setAttitude(auxq);
	
	/** Also the heading changed due to fog heading, update in the myikf extructure **/
	myikf->setAttitude(auxq);
    }
    
    /** Write the Angular velocity (as the different between two orientations in radians)*/
    rbs_b_g->angular_velocity = (euler - (*oldeuler))/imu_dt;
    
    /** Store the euler angle for the next iteration **/
    (*oldeuler)= euler;
    
    /** Write in the OROCOS Ports */
    _attitude_b_g.write((*rbs_b_g));
    
    /** Write inputs into output for backup **/
    backup->time = imu_samples_sample.time;
    backup->gyro[0] = (*imu_gyros)[0];
    backup->gyro[1] = (*imu_gyros)[1];
    backup->gyro[2] = (*fog_gyros)[2];
    backup->acc = (*imu_acc);
    
    _inputs_backup.write ((*backup));
    
    
    
  }
  else if (!_imu_orientation.connected())
  {
      /** Add one acc sample to the buffer **/
      init_acc->col(accidx) = imu_samples_sample.acc;
      accidx++;

      if (accidx == NUMBER_INIT_ACC)
      {
	Eigen::Matrix <double,NUMAXIS,1> meanacc;
	Eigen::Matrix <double,NUMAXIS,1> euler;
	Eigen::Quaternion <double> attitude; /**< Initial attitude in case no port in imu_orientation is connected **/
	
	meanacc[0] = init_acc->row(0).mean();
	meanacc[1] = init_acc->row(1).mean();
	meanacc[2] = init_acc->row(2).mean();
	
	std::cout<<"Mean acc values: "<<meanacc[0]<<" "<<meanacc[1]<<" "<<meanacc[2]<<"\n";	
	std::cout<<"Computed gravity: "<<meanacc.norm()<<"\n";
	
	euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
	euler[1] = (double)-atan(meanacc[0]/meanacc[2]); //Pitch
	euler[2] = 0.00;
	
	/** Set the initial attitude when no initial IMU orientation is provided **/
	attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
	Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
	
	/** Set the initial attitude quaternion of the IKF **/
	myikf->setAttitude (attitude);
	fogikf->setAttitude (attitude);
	init_attitude = true;
	
	RTT::log(RTT::Info) << "******** Init Attitude IKFEstimator *******"<< RTT::endlog();
	RTT::log(RTT::Info) << "Init Roll: "<<Angle::rad2Deg(euler[0])<<"Init Pitch: "<<Angle::rad2Deg(euler[1])<<"Init Yaw: "<<Angle::rad2Deg(euler[2])<< RTT::endlog();
      }


  }

  return;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See IKFEstimator.hpp for more detailed
// documentation about them.

bool IKFEstimator::configureHook()
{
    Eigen::Matrix< double, IKFSTATEVECTORSIZE , 1  > x_0; /** Initial vector state **/
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Ra; /**< Measurement noise convariance matrix for acc */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rg; /**< Measurement noise convariance matrix for gyros */
    Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rm; /**< Measurement noise convariance matrix for mag */
    Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> P_0; /**< Initial covariance matrix **/
    Eigen:: Matrix <double,NUMAXIS,NUMAXIS> Qbg;
    Eigen:: Matrix <double,NUMAXIS,NUMAXIS> Qba;
    double latitude = (double)_latitude.value();
    double altitude = (double)_altitude.value();
    double g;
    
    /************************/
    /** Read configuration **/
    /************************/
    adaptiveconfig = _adaptive_config.value();

    /** Fill the matrices **/
    Ra = Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    
    Ra(0,0) = pow(_accrw.get()[0]/sqrt(_delta_time.value()),2);
    Ra(1,1) = pow(_accrw.get()[1]/sqrt(_delta_time.value()),2);
    Ra(2,2) = pow(_accrw.get()[2]/sqrt(_delta_time.value()),2);

    Rg = Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rg(0,0) = pow(_gyrorw.get()[0]/sqrt(_delta_time.value()),2);
    Rg(1,1) = pow(_gyrorw.get()[1]/sqrt(_delta_time.value()),2);
    Rg(2,2) = pow(_gyrorw.get()[2]/sqrt(_delta_time.value()),2);

    Rm = Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    Rm(0,0) = pow(_magrw.get()[0]/sqrt(_delta_time.value()),2);
    Rm(1,1) = pow(_magrw.get()[1]/sqrt(_delta_time.value()),2);
    Rm(2,2) = pow(_magrw.get()[2]/sqrt(_delta_time.value()),2);
  
    /** Initial error covariance **/
    P_0 = Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Zero();
    P_0.block <NUMAXIS, NUMAXIS> (0,0) = 0.001 * Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> (3,3) = 0.00001 * Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    P_0.block <NUMAXIS, NUMAXIS> (6,6) = 0.00001 * Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    
    Qbg = 0.00000000001 * Matrix <double,NUMAXIS,NUMAXIS>::Identity();
    Qba = 0.00000000001 * Matrix <double,NUMAXIS,NUMAXIS>::Identity();

    std::cout<< "Ra\n"<<Ra<<"\n";
    std::cout<< "Rg\n"<<Rg<<"\n";
    std::cout<< "Rm\n"<<Rm<<"\n";
    std::cout<< "P_0\n"<<P_0<<"\n";

    
    /** Info and Warnings about the Task **/
    
    if (_imu_orientation.connected())
    {
	RTT::log(RTT::Info) << "IMU is connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "IMU Orientation NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Initial orientation is not provided."<< RTT::endlog();
	RTT::log(RTT::Warning) << "Zero Yaw angle pointing to North is then assumed." << RTT::endlog();
	RTT::log(RTT::Warning) << "Pitch and Roll are taken from accelerometers assuming static body at Init phase." << RTT::endlog();
	
    }
    
    if (_fog_samples.connected())
    {
	RTT::log(RTT::Info) << "FOG is connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "FOG NO connected" << RTT::endlog();
	RTT::log(RTT::Info) << "Heading will be calculated from the IMU samples." << RTT::endlog();
    }
    if (_imu_samples.connected())
    {
	RTT::log(RTT::Info) << "IMU samples is connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "IMU samples NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Potential malfunction on the task!!" << RTT::endlog();
    }
    
    /** Gravitational value according to the location **/
    g = BaseEstimator::GravityModel (latitude, altitude);

    /** Output port frames information */
    rbs_b_g->sourceFrame = "Body_Frame"; /** The body Frame in Source  */
    rbs_b_g->targetFrame = "Geographic_Frame (North-West-Up)"; /** The Geographic Frame in Target */

    /** Initial values for the IKF **/
    myikf->Init(P_0, Ra, Rg, Rm, Ra, Qbg, Qba, Qba, g, (double)_dip_angle.value(),
            adaptiveconfig.M1, adaptiveconfig.M2, adaptiveconfig.gamma,
            adaptiveconfig.M1, adaptiveconfig.M2, adaptiveconfig.gamma);
    fogikf->Init(P_0, Ra, Rg, Rm, Ra, Qbg, Qba, Qba, g, (double)_dip_angle.value(),
            adaptiveconfig.M1, adaptiveconfig.M2, adaptiveconfig.gamma,
            adaptiveconfig.M1, adaptiveconfig.M2, adaptiveconfig.gamma);

    /** init set the vector state to zero but it can be changed here **/
    x_0 = Matrix<double,IKFSTATEVECTORSIZE,1>::Zero();
    x_0.block<NUMAXIS, 1> (3,0) = _gbiasof.value();
    x_0.block<NUMAXIS, 1> (6,0) = _abiasof.value();
    myikf->setState(x_0);

    return IKFEstimatorBase::configureHook();
}
bool IKFEstimator::startHook()
{
    if (! IKFEstimatorBase::startHook())
        return false;
    return true;
}
void IKFEstimator::updateHook()
{
    IKFEstimatorBase::updateHook();
    
}
void IKFEstimator::errorHook()
{
    IKFEstimatorBase::errorHook();
}
void IKFEstimator::stopHook()
{
    IKFEstimatorBase::stopHook();
}
void IKFEstimator::cleanupHook()
{
    IKFEstimatorBase::cleanupHook();
}

