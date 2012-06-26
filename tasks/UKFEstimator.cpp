/* Generated from orogen/lib/orogen/templates/tasks/UKFEstimator.cpp */
/**\file UKFEstimator.cpp
 *
 * This class perform a OROCOS/ROCK component for Attitude (orientation) estimation
 * using the Unscented Kalman Filter (UKF) library.
 * IKF uses the measurement comming from the IMU sensor (Xsens MTi). It predict using
 * gyros read-out and correct using the Accelerometers and the Magnetometers (Yaw only).
 * The IKF is dynamics in the measurement step for accelerometers, therefore the attitude 
 * estimation for Pitcha and Roll is also valid when external acceleration are felt.
 * 
 * This UKFEstimator uses the StreamAlligner having three Agregators (Inputs)
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

#include "UKFEstimator.hpp"
#include "BaseEstimator.hpp"

using namespace orientation_estimator;
using namespace filter;

/**
 * @brief Constructor
 * 
 * Memory allocation for UKFEstimator component and
 * initial values of task flags. 
 * 
 * @return void
 *
 */
UKFEstimator::UKFEstimator(std::string const& name)
    : UKFEstimatorBase(name)
{
    gyros = new Eigen::Matrix <double,NUMAXIS,1>;
    acc = new Eigen::Matrix <double,NUMAXIS,1>;
    rbs_b_g = new base::samples::RigidBodyState();
    rbs_b_g->invalidate();
    oldeuler = new Eigen::Matrix <double, NUMAXIS, 1>;
    myukf = new ukf;

    flag_imu_time  = false;
    flag_fog_time  = false;
    init_attitude = false;  
}

/**
 * @brief Destructor
 * 
 * Free allocated memory by UKFEstimator class
 *
 *
 * @return void
 *
 */
UKFEstimator::~UKFEstimator()
{

    /** Free filter object **/
    delete myukf;
    myukf = NULL;

    delete oldeuler;
    oldeuler = NULL;

    delete rbs_b_g;
    rbs_b_g = NULL;

    delete gyros;
    gyros = NULL;

    delete acc;
    acc = NULL;
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
void UKFEstimator::fog_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &fog_samples_sample)
{
    Eigen::Quaternion <double> qb_g;
        
    if (init_attitude == true)
    {	
	if (flag_fog_time == false)
	{
	    /** First timing **/
	    fog_time = (double)fog_samples_sample.time.toMilliseconds();
	    
	    /** Copy the sensor information */
	    (*gyros)[2] = fog_samples_sample.gyro[2];
	    
	    flag_fog_time = true;
	    
	}
	else
	{   
	    /** Timing update **/
	    fog_dt = ((double)fog_samples_sample.time.toMilliseconds() - fog_time)/1000.00;
	    fog_time = (double)fog_samples_sample.time.toMilliseconds();
	    
	
	    /** Copy the sensor information */
	    (*gyros)[2] = fog_samples_sample.gyro[2];
	    (*gyros)[1] = 0.00;
	    (*gyros)[0] = 0.00;
	    
	    
	    qb_g = myukf->getAttitude();
	    
	    BaseEstimator::SubstractEarthRotation (gyros, &qb_g, _latitude.value());
	    
	    (*gyros)[1] = 0.00;
	    (*gyros)[0] = 0.00;
    
	    myukf->predict (gyros, fog_dt);
	    myukf->attitudeUpdate ();

	}
    }

  return;
}

/**
 * @brief IMU orientation callback function
 * 
 * This function performs the callback of the StreamAlligner for the 
 * orientation coming from the IMU quaternion
 * 
 * This function is used to initially set the quaternion.
 * Therefore, it initializes the original value for the orientation (init quaternion).
 *
 * @author Javier Hidalgo Carrio.
 *
 * @param[in] &ts timestamp
 * @param[in] &imu_samples_sample IMU sensor quaternion.
 *
 * @return void
 *
 */
void UKFEstimator::imu_orientationCallback(const base::Time &ts, const ::base::samples::RigidBodyState &imu_orientation_sample)
{
  
    Eigen::Quaternion <double> attitude (imu_orientation_sample.orientation.w(), imu_orientation_sample.orientation.x(),
    imu_orientation_sample.orientation.y(), imu_orientation_sample.orientation.z());
    Eigen::Matrix <double, NUMAXIS, 1> euler;

    if (init_attitude == false)
    {
	std::cout << "******** Init Attitude UKFEstimator *******\n";
	/** Eliminate the Magnetic declination from the initial attitude quaternion  (because the initial quaternion from Xsens come from Magn) **/
	BaseEstimator::CorrectMagneticDeclination (&attitude, _magnetic_declination.value(), _magnetic_declination_mode.value());
	
	/** Set the initial attitude quaternion of the UKF **/
	myukf->setAttitude (&attitude);
	init_attitude = true;
	
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
void UKFEstimator::imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{

    Eigen::Quaternion <double> qb_g;
    Eigen::Matrix <double, NUMAXIS, 1> euler;
    
    /** Copy the sensor information */
    (*gyros)[0] = imu_samples_sample.gyro[0];
    (*gyros)[1] = imu_samples_sample.gyro[1];
    (*gyros)[2] = imu_samples_sample.gyro[2];
    
    (*acc) = imu_samples_sample.acc;
    
        
    if (init_attitude == true)
    {
	if (flag_imu_time == false)
	{
	    imu_time = (double)imu_samples_sample.time.toMilliseconds();
	    flag_imu_time = true;
	}
	else
	{
	    imu_dt = ((double)imu_samples_sample.time.toMilliseconds() - imu_time)/1000.00;
	    imu_time = (double)imu_samples_sample.time.toMilliseconds();


	    /** Substract the Earth Rotation from the gyros output */
	    qb_g = myukf->getAttitude(); /** Rotation with respect to the geographic frame (North-Up-West) */
	    BaseEstimator::SubstractEarthRotation (gyros, &qb_g, _latitude.value());
	    
	    /** Orientation (Pitch and Roll from IMU, Yaw from FOG) */
	    if (_fog_samples.connected())
		(*gyros)[2] = 0.00;
	    
	    /** Perform the Unscented Kalman Filter */
 	    myukf->predict (gyros, imu_dt);
	    myukf->update (acc, acc);
	}


	/** Out in the Outports  */
	rbs_b_g->time = imu_samples_sample.time; //base::Time::now(); /** Set the timestamp */
	
	/** Get attitude from the filter **/
	qb_g = myukf->getAttitude();

	/** Copy to the rigid_body_state **/
	rbs_b_g->orientation = (base::Orientation) qb_g;
	
	/** Copy to covariance to the rigid_body_state **/
	rbs_b_g->cov_orientation = myukf->getCovariance().block<NUMAXIS, NUMAXIS>(0,0);
	
	/** Get Attitude en Euler **/
	euler = myukf->getEuler();

	/** Write the Angular velocity (as the different between two orientations in radians)*/
	rbs_b_g->angular_velocity = (euler - (*oldeuler))/imu_dt;

	/** Store the euler angle for the next iteration **/
	(*oldeuler)= euler;

	/** Write in the OROCOS Ports */
	_attitude_b_g.write((*rbs_b_g));
	
    }
      
  return;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See UKFEstimator.hpp for more detailed
// documentation about them.

bool UKFEstimator::configureHook()
{
    Eigen::Quaternion <double> attitude; /**< Initial attitude in case no port is connected **/
    Eigen::Matrix <double,UKFSTATEVECTORSIZE, UKFSTATEVECTORSIZE> P_0; /**< Initial State covariance matrix */
    Eigen::Quaternion <double> at_q;  /**< Attitude quaternion. Note the order of the arguments: the real w coefficient first, while internally the coefficients are stored in the following order: [x, y, z, w] */
    Eigen::Matrix <double,UKFSTATEVECTORSIZE, UKFSTATEVECTORSIZE> Q; /**< Process noise covariance matrix */
    Eigen::Matrix <double,NUMAXIS, NUMAXIS>  R; /**< Measurements noise variance and covar matrix */
    Eigen::Matrix <double,UKFSTATEVECTORSIZE,1> x_0; /**< Initial state vector */
    Eigen::Matrix <double,NUMAXIS,1> sigma_gyrosrw; 
    Eigen::Matrix <double,NUMAXIS,1> sigma_gyrosrrw;
    double latitude = (double)_latitude.value();
    double altitude = (double)_altitude.value();
    double g;


    /** Init times **/
    fog_time = 0.00;	imu_time = 0.00;
    fog_dt = 0.00;	imu_dt = 0.00;
    
    
    /** Init covariances **/
    sigma_gyrosrw[0] = _gyrorw.get()[0]/sqrt(_delta_time.value());
    sigma_gyrosrw[1] = _gyrorw.get()[1]/sqrt(_delta_time.value());
    sigma_gyrosrw[2] = _gyrorw.get()[2]/sqrt(_delta_time.value());
    
    sigma_gyrosrrw[0] = _gyrorrw.get()[0]*sqrt(_delta_time.value()/3);
    sigma_gyrosrrw[1] = _gyrorrw.get()[1]*sqrt(_delta_time.value()/3);
    sigma_gyrosrrw[2] = _gyrorrw.get()[2]*sqrt(_delta_time.value()/3);
    
    /** Fill the matrices **/
    R = Matrix<double,NUMAXIS,NUMAXIS>::Zero();
    R(0,0) = pow(_accrw.get()[0]/sqrt(_delta_time.value()),2);
    R(1,1) = pow(_accrw.get()[1]/sqrt(_delta_time.value()),2);
    R(2,2) = pow(_accrw.get()[2]/sqrt(_delta_time.value()),2);

    Q = Matrix<double,UKFSTATEVECTORSIZE,UKFSTATEVECTORSIZE>::Zero();
    Q(0,0) = pow(sigma_gyrosrw[0], 2)-(1.0/6.0)*pow(sigma_gyrosrrw[0], 2)*pow(_delta_time.value(), 2);
    Q(1,1) = pow(sigma_gyrosrw[1], 2)-(1.0/6.0)*pow(sigma_gyrosrrw[1], 2)*pow(_delta_time.value(), 2);
    Q(2,2) = pow(sigma_gyrosrw[2], 2)-(1.0/6.0)*pow(sigma_gyrosrrw[2], 2)*pow(_delta_time.value(), 2);
    Q(3,3) = (_delta_time.value()*0.5)*pow(sigma_gyrosrrw[0], 2);
    Q(4,4) = (_delta_time.value()*0.5)*pow(sigma_gyrosrrw[1], 2);
    Q(5,5) = (_delta_time.value()*0.5)*pow(sigma_gyrosrrw[2], 2);
    Q = Q * (_delta_time.value() / 2.0);
    
    P_0 = Matrix<double,UKFSTATEVECTORSIZE,UKFSTATEVECTORSIZE>::Zero();
    P_0(0,0) = 0.0025;
    P_0(1,1) = 0.0025;
    P_0(2,2) = 0.0025;
    P_0(3,3) = 0.0000000000001;
    P_0(4,4) = 0.0000000000001;
    P_0(5,5) = 0.0000000000001;

    x_0 << 0,0,0,0.0,0.0,0.0;
    
    at_q = Eigen::Quaternion<double>::Identity();
    
//     std::cout<< "R\n"<<R<<"\n";
//     std::cout<< "Q\n"<<Q<<"\n";
//     std::cout<< "P_0\n"<<P_0<<"\n";

    /** Gravitational value according to the location **/
    g = BaseEstimator::GravityModel (latitude, altitude);

    /** Output port frames information */
    rbs_b_g->sourceFrame = "Body_Frame"; /** The body Frame in Source  */
    rbs_b_g->targetFrame = "Geographic_Frame (North-West-Up)"; /** The Geographic Frame in Target */

    /** Initial values for the UKF **/
    myukf->Init(&x_0, &P_0, &Q, &R, &at_q, (double)1.00, (double)4.00, 1, g);
    
    /** Info and Warnings about the Task **/
    if (_imu_orientation.connected())
    {
	RTT::log(RTT::Info) << "IMU is connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "IMU NO connected." << RTT::endlog();
	RTT::log(RTT::Warning) << "Initial orientation is not provided."<< RTT::endlog();
	RTT::log(RTT::Warning) << "Zero angles attitude pointing North is then assumed." << RTT::endlog();
	
	/** Set the initial attitude when no initial IMU orientation is provided **/
	attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(0.00, Eigen::Vector3d::UnitZ())*
			Eigen::AngleAxisd(0.00, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(0.00, Eigen::Vector3d::UnitX()));
	
	/** Set the initial attitude quaternion of the UKF **/
	myukf->setAttitude (&attitude);
	init_attitude = true;
	
	RTT::log(RTT::Warning) << "Init: Pitch 0.00 Init: Roll 0.00 Init: Yaw 0.00" << RTT::endlog();
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
	RTT::log(RTT::Warning) << "Potential malfunction on the task!" << RTT::endlog();
    }

    return UKFEstimatorBase::configureHook();
}
bool UKFEstimator::startHook()
{
    if (! UKFEstimatorBase::startHook())
        return false;
    return true;
}
void UKFEstimator::updateHook()
{
    UKFEstimatorBase::updateHook();
    
}
void UKFEstimator::errorHook()
{
    UKFEstimatorBase::errorHook();
}
void UKFEstimator::stopHook()
{
    UKFEstimatorBase::stopHook();
}
void UKFEstimator::cleanupHook()
{
    UKFEstimatorBase::cleanupHook();
}
