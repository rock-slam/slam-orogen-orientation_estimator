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
  
  xsens_gyros = new Eigen::Matrix <double,NUMAXIS,1>;
  xsens_acc = new Eigen::Matrix <double,NUMAXIS,1>;
  xsens_mag = new Eigen::Matrix <double,NUMAXIS,1>;
  fog_gyros = new Eigen::Matrix <double,NUMAXIS,1>;
  head_q = new Eigen::Quaternion <double>;
  rbs_b_g = new base::samples::RigidBodyState;
  oldeuler = new Eigen::Matrix <double, NUMAXIS, 1>;
  myikf = new ikf;
  
  backup = new base::samples::IMUSensors;
  
  flag_xsens_time  = false;
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
void IKFEstimator::fog_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &fog_samples_sample)
{
  
  Eigen::Matrix <double, NUMAXIS, 1> euler;
  (*fog_gyros) = fog_samples_sample.gyro;
  
  if (init_attitude == true)
  {
    if (flag_fog_time == false)
    {
      fog_time = (double)fog_samples_sample.time.toMilliseconds();      
      flag_fog_time = true;
    }
    else
    {     
      fog_dt = ((double)fog_samples_sample.time.toMilliseconds() - fog_time)/1000.00;
      fog_time = (double)fog_samples_sample.time.toMilliseconds();
      
//       /** Substract the Earth Rotation from the FOG output */
//       BaseEstimator::SubstractEarthRotation (fog_gyros, head_q, _latitude.value());
//       
//       /** Only in the Yaw (Z-axis) are the FOG angular velocity ) */
//       (*fog_gyros)[0] = 0.00;
//       (*fog_gyros)[1] = 0.00;
//       (*fog_gyros)[2] = (*fog_gyros)[2] - _gbiasof.get()[2];
//       
//       BaseEstimator::PropagateHeadingQuaternion (head_q, fog_gyros, fog_dt);
      
      //myikf->Quaternion2Euler(head_q, &euler);
      /*euler[2] = quat->toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
      euler[1] = quat->toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
      euler[0] = quat->toRotationMatrix().eulerAngles(2,1,0)[2];//ROL*/
     //std::cout << "Heading(FOG): "<< euler[2]*R2D <<"\n";

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
void IKFEstimator::xsens_orientationCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xsens_orientation_sample)
{
  
  Eigen::Quaternion <double> attitude (xsens_orientation_sample.orientation.w(), xsens_orientation_sample.orientation.x(),
    xsens_orientation_sample.orientation.y(), xsens_orientation_sample.orientation.z());
   Eigen::Matrix <double, NUMAXIS, 1> euler;
   
   if (init_attitude == false)
   {
     std::cout << "******** Init Attitude IKFEstimator *******\n";
     /** Eliminate the Magnetic declination from the initial attitude quaternion **/
     BaseEstimator::CorrectMagneticDeclination (&attitude, _magnetic_declination.value(), _magnetic_declination_mode.value());
     
     /** Set the initial attitude quaternion of the IKF **/
     myikf->setAttitude (&attitude);
     init_attitude = true;
     
     /** Fog quaternion initial value is also the IKF initial quaternion **/
     (*head_q) = myikf->getAttitude ();
     
     //myikf->Quaternion2Euler(&attitude, &euler);
     
     euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
     euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//PITCH
     euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//ROLL
     
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
void IKFEstimator::xsens_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &xsens_samples_sample)
{
  Eigen::Quaternion <double> qb_g;
  Eigen::Quaternion <double> auxq;
  Eigen::Matrix <double, NUMAXIS, 1> euler;
  Eigen::Matrix <double, NUMAXIS, 1> heading;
  
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
      (*xsens_gyros)[2] = (*fog_gyros)[2];
      (*xsens_acc) = xsens_samples_sample.acc;
      (*xsens_mag) = xsens_samples_sample.mag;

      /** Substract the Earth Rotation from the gyros output */
      qb_g = myikf->getAttitude(); /** Rotation with respect to the geographic frame (North-Up-West) */
      BaseEstimator::SubstractEarthRotation (xsens_gyros, &qb_g, _latitude.value());
      
      /** Perform the Indirect Kalman Filter */
      myikf->predict (xsens_gyros, xsens_dt);
      myikf->update (xsens_acc, xsens_mag, false);
    }
    
    /** Get Attitude en Euler **/
    euler = myikf->getEuler();
    
    /** Out in the Outports  */
    rbs_b_g->time = xsens_samples_sample.time; //base::Time::now(); /** Set the timestamp */
    
    /** Orientation (Pitch and Roll from IKF, Yaw from FOG) */
//     euler[2] = head_q->toRotationMatrix().eulerAngles(2,1,0)[0];//YAW
    
    std::cout << "(Roll, Pitch, Yaw)\n"<< euler[0]*R2D<<","<< euler[1]*R2D<<","<< euler[2]*R2D<<"\n";
     
    auxq = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
 			    Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
 			    Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())); //Roll, Pitch and Yaw in this order
    
    /** Copy to the rigid_body_state **/
    rbs_b_g->orientation = (base::Orientation) auxq;
    
    /** Also update the quaternion used by the fog callback function **/
    (*head_q) = auxq;
    
    /** Write the Angular velocity (as the different between two orientations in radians)*/
    rbs_b_g->angular_velocity = (euler - (*oldeuler))/xsens_dt;
    
    /** Store the euler angle for the next iteration **/
    (*oldeuler)= euler;
    
    /** Write in the OROCOS Ports */
    _attitude_b_g.write((*rbs_b_g));
    
    /** Write inputs into output for backup **/
    backup->time = xsens_samples_sample.time;
    backup->gyro[0] = (*xsens_gyros)[0];
    backup->gyro[1] = (*xsens_gyros)[1];
    backup->gyro[2] = (*fog_gyros)[2];
    backup->acc = (*xsens_acc);
    
    _inputs_backup.write ((*backup));
    
    
    
  }
  return;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See IKFEstimator.hpp for more detailed
// documentation about them.

bool IKFEstimator::configureHook()
{
  
  Eigen::Matrix <double,NUMAXIS,NUMAXIS> Ra; /**< Measurement noise convariance matrix for acc */
  Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rg; /**< Measurement noise convariance matrix for gyros */
  Eigen::Matrix <double,NUMAXIS,NUMAXIS> Rm; /**< Measurement noise convariance matrix for mag */
  double latitude = (double)_latitude.value();
  double altitude = (double)_altitude.value();
  double g;
  
  
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

/*  std::cout<< "Ra\n"<<Ra<<"\n";
  std::cout<< "RG\n"<<Rg<<"\n";
  std::cout<< "RM\n"<<Rm<<"\n";*/
  
  /** Gravitational value according to the location **/
  g = BaseEstimator::GravityModel (latitude, altitude);
  
  /** Output port frames information */
  rbs_b_g->sourceFrame = "Body_Frame"; /** The body Frame in Source  */
  rbs_b_g->targetFrame = "Geographic_Frame (North-West-Up)"; /** The Geographic Frame in Target */
  
  /** Initial values for the IKF **/
  myikf->Init(&Ra, &Rg, &Rm, g, (double)_dip_angle.value());
  
  return IKFEstimatorBase::configureHook();;  
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

