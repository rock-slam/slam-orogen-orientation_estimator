/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "IKF.hpp"

#include "BaseEstimator.hpp"


using namespace orientation_estimator;

IKF::IKF(std::string const& name)
    : IKFBase(name)
{
}

IKF::IKF(std::string const& name, RTT::ExecutionEngine* engine)
    : IKFBase(name, engine)
{
}

IKF::~IKF()
{
}

void IKF::imu_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{
    /** Receive imu to body transformation **/
    Eigen::Affine3d imu2body;
    if (!_imu2body.get(ts, imu2body))
    {
        RTT::log(RTT::Error) << "skip, have no imu2body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    /** check for NaN values */
    if(!base::isnotnan(imu_samples_sample.acc))
    {
	RTT::log(RTT::Fatal) << "ERROR: Accelerometer readings contain NaN values!" << RTT::endlog();
	return exception(NAN_ERROR);
    }
    if(!base::isnotnan(imu_samples_sample.gyro))
    {
	RTT::log(RTT::Fatal) << "ERROR: Gyroscope readings contain NaN values!" << RTT::endlog();
	return exception(NAN_ERROR);
    }
    if(config.use_magnetometers && !base::isnotnan(imu_samples_sample.mag))
    {
	RTT::log(RTT::Fatal) << "ERROR: Magnetometer readings contain NaN values!" << RTT::endlog();
	return exception(NAN_ERROR);
    }
    
    /** Rotate measurements to body frame **/
    base::samples::IMUSensors transformed_imu_samples;
    transformed_imu_samples.time = imu_samples_sample.time;
    transformed_imu_samples.acc = imu2body.rotation() * imu_samples_sample.acc;
    transformed_imu_samples.gyro = imu2body.rotation() * imu_samples_sample.gyro;
    transformed_imu_samples.mag = imu2body.rotation() * imu_samples_sample.mag;
    
    /** Attitude filter **/
    if(!init_attitude)
    {
	//** Do initial alignment **/
	initialAlignment(ts, transformed_imu_samples);
    }
    else
    {
	if(!prev_ts.isNull())
	{
	    new_state = RUNNING;
	    double delta_t = (ts - prev_ts).toSeconds();
	    if(delta_t > max_time_delta)
	    {
		RTT::log(RTT::Warning) << "Time delta exceeds maximum allowed time delta." << RTT::endlog();
		RTT::log(RTT::Warning) << "Predition step size will be limited to max_time_delta." << RTT::endlog();
		delta_t = max_time_delta;
	    }
	    
	    /** Eliminate Earth rotation **/
	    gyro_reading = transformed_imu_samples.gyro;
	    if(config.substract_earth_rotation)
	    {
		Eigen::Quaterniond q_body2world = ikf_filter.getAttitude();
		BaseEstimator::SubstractEarthRotation(&gyro_reading, &q_body2world, location.latitude);
	    }
	    
	    /** Predict **/
	    ikf_filter.predict(gyro_reading, delta_t);
	    
	}
	prev_ts = ts;
	    
	
	/** Filter magnetometer samples **/
	if(config.use_magnetometers)
	    mag_imu_sum += transformed_imu_samples.mag;

	/** Filter accelerometer samples **/
	acc_imu_sum += transformed_imu_samples.acc;
	// TODO Use a better filter than a mean filter
	
	imu_samples++;
	if(imu_start.isNull())
	    imu_start = ts;
	else if(imu_samples >= 1 && (ts - imu_start).toSeconds() > (1.0/config.correction_frequency))
	{
	    Eigen::Matrix <double,3,1> aux;
	    aux.setZero();
	    
	    base::Vector3d prev_euler = base::getEuler(ikf_filter.getAttitude());
	    
	    /** Update/Correction IMU accelerometers **/
	    Eigen::Vector3d acc_imu_mean = acc_imu_sum / (double)imu_samples;
	    ikf_filter.update(acc_imu_mean, true, aux, false, aux, false);

	    /** Exclude yaw angle from accelerometers correction step **/
	    base::Vector3d corrected_euler = base::getEuler(ikf_filter.getAttitude());
	    Eigen::Quaterniond corrected_attitide = Eigen::AngleAxisd(prev_euler[0], Eigen::Vector3d::UnitZ()) * 
						    Eigen::AngleAxisd(corrected_euler[1], Eigen::Vector3d::UnitY()) *
						    Eigen::AngleAxisd(corrected_euler[2], Eigen::Vector3d::UnitX());
	    ikf_filter.setAttitude(corrected_attitide);
	    
	    /** Update/Correction IMU magnetometers **/
	    if(config.use_magnetometers)
	    {
		Eigen::Vector3d mag_imu_mean = mag_imu_sum / (double)imu_samples;
		ikf_filter.update(aux, false, aux, false, mag_imu_mean, true);
	    }

	    /** Reset counter and accumulated values **/
	    acc_imu_sum.setZero();
	    mag_imu_sum.setZero();
	    imu_start.microseconds = 0;
	    imu_samples = 0;
	}
	
	
    }
}

void IKF::initialAlignment(const base::Time &ts,  const base::samples::IMUSensors &imu_sample)
{
    #ifdef DEBUG_PRINTS
	std::cout<<"** [ORIENT_IKF] Initial Attitude[IMU: "<<initial_samples<<"]\n";
    #endif
    new_state = INITIAL_ALIGNMENT;
    
    if(initial_alignment_ts.isNull())
	initial_alignment_ts = ts;

    initial_alignment.acc += imu_sample.acc;
    initial_alignment.gyro += imu_sample.gyro;
    initial_alignment.mag += imu_sample.mag;
    initial_samples++;

    /** Calculate the initial alignment to the local geographic frame **/
    if ((ts - initial_alignment_ts).toSeconds() >= config.initial_alignment_duration)
    {
	/** Set attitude to inital heading **/
	Eigen::Quaterniond initial_attitude = Eigen::Quaterniond(Eigen::AngleAxisd(_initial_heading.value(), Eigen::Vector3d::UnitZ()));

	if (config.initial_alignment_duration > 0)
	{
	    if(initial_samples == 0)
	    {
		RTT::log(RTT::Fatal)<<"[orientation_estimator] No samples available to perform the inital alignment."<<RTT::endlog();
		return exception(CONFIGURATION_ERROR);
	    }
	    
	    /** Compute mean values **/
	    initial_alignment.acc /= (double)initial_samples;
	    initial_alignment.gyro /= (double)initial_samples;
	    initial_alignment.mag /= (double)initial_samples;

	    if ((base::isnotnan(initial_alignment.acc)) && (base::isnotnan(initial_alignment.gyro)))
	    {
		if (initial_alignment.acc.norm() < (GRAVITY+GRAVITY_MARGIN) && initial_alignment.acc.norm() > (GRAVITY-GRAVITY_MARGIN))
		{
		    /** Override the gravity model value with the sensed from the sensors **/
		    if (config.use_samples_as_theoretical_gravity)
			ikf_filter.setGravity(initial_alignment.acc.norm());

		    /** Compute the local horizontal plane **/
		    Eigen::Quaterniond rot = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), initial_alignment.acc);
		    base::Vector3d euler = base::getEuler(rot);
		    euler.z() = 0.0;
		    /* TODO: This is most likely not correct, it should be plane in UNIT_Z, this is UNIT_Z in plane:
		    Eigen::Vector3d euler;
		    euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
		    euler[1] = (double) -atan(meanacc[0]/meanacc[2]); //Pitch
		    euler[2] = 0.0; //Yaw
		    */
		    
		    /** Set the attitude  **/
		    initial_attitude = Eigen::Quaterniond(Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX()));

		    #ifdef DEBUG_PRINTS
		    std::cout<< "******** Local Horizontal *******"<<"\n";
		    std::cout<< "Roll: "<<base::Angle::rad2Deg(euler[2])<<" Pitch: "<<base::Angle::rad2Deg(euler[1])<<" Yaw: "<<base::Angle::rad2Deg(euler[0])<<"\n";
		    #endif

		    /** The angular velocity in the local horizontal plane **/
		    /** Gyro_ho = Tho_body * gyro_body **/
		    Eigen::Vector3d transformed_meangyro = initial_attitude * initial_alignment.gyro;

		    /** Determine the initial heading **/
		    if(config.initial_heading_source == MAGNETOMETERS)
		    {
			if(base::isnotnan(initial_alignment.mag) && !initial_alignment.mag.isZero())
			{
			    Eigen::Vector3d transformed_meanmag = initial_attitude * initial_alignment.mag;
			    euler[0] = base::Angle::fromRad(-atan2(transformed_meanmag.y(), transformed_meanmag.x())).getRad();
			}
			else
			{
			    RTT::log(RTT::Warning) << "Don't have any magnetometer samples." << RTT::endlog();
			    RTT::log(RTT::Warning) << "Falling back to initial heading from parameter." << RTT::endlog();
			    euler[0] = _initial_heading.value();
			}
			
		    }
		    else if(config.initial_heading_source == ESTIMATE_FROM_EARTH_ROTATION)
		    {
			if (transformed_meangyro.x() == 0.0 && transformed_meangyro.y() == 0.0)
			{
			    RTT::log(RTT::Warning) << "Couldn't estimate initial heading. Earth rotaion was estimated as zero." << RTT::endlog();
			    RTT::log(RTT::Warning) << "Falling back to initial heading from parameter." << RTT::endlog();
			    euler[0] = _initial_heading.value();
			}
			else
			    euler[0] = base::Angle::fromRad(-atan2(transformed_meangyro.y(), transformed_meangyro.x())).getRad();
		    }
		    else if(config.initial_heading_source == INITIAL_HEADING_PARAMETER)
		    {
			euler[0] = _initial_heading.value();
		    }
		    else
		    {
			RTT::log(RTT::Fatal)<<"[orientation_estimator] Selected initial heading source is unknown."<<RTT::endlog();
			return exception(CONFIGURATION_ERROR);
		    }
		    
		    /** Set the attitude  **/
		    initial_attitude = Eigen::Quaterniond(Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX()));

		    #ifdef DEBUG_PRINTS
		    std::cout<< " Mean Gyro:\n"<<transformed_meangyro<<"\n Mean Acc:\n"<<initial_alignment.acc<<"\n";
		    std::cout<< " Earth rot * cos(lat): "<<EARTHW*cos(location.latitude)<<"\n";
		    std::cout<< " Filter Gravity: "<<ikf_filter.getGravity()[2]<<"\n";
		    std::cout<< "******** Azimuthal Orientation *******"<<"\n";
		    std::cout<< " Yaw: "<<base::Angle::rad2Deg(euler[0])<<"\n";
		    #endif

		    /** Compute the Initial Bias **/
		    Eigen::Vector3d gyro_bias = initial_alignment.gyro;
		    if(config.substract_earth_rotation)
			BaseEstimator::SubstractEarthRotation(&gyro_bias, &initial_attitude, location.latitude);
		    
		    Eigen::Vector3d acc_bias = initial_alignment.acc - initial_attitude.inverse() * ikf_filter.getGravity();
		    
		    ikf_filter.setInitBias(gyro_bias, acc_bias, Eigen::Vector3d::Zero());

		    #ifdef DEBUG_PRINTS
		    std::cout<< "******** Initial Bias Offset *******"<<"\n";
		    std::cout<< " Gyroscopes Bias Offset:\n"<<ikf_filter.getGyroBias()<<"\n";
		    std::cout<< " Accelerometers Bias Offset:\n"<<ikf_filter.getAccBias()<<"\n";
		    #endif
		}
		else
		{
		    RTT::log(RTT::Fatal)<<"[orientation_estimator] ERROR in Initial Alignment. Unable to compute reliable attitude."<<RTT::endlog();
		    RTT::log(RTT::Fatal)<<"[orientation_estimator] Computed "<< initial_alignment.acc.norm() <<" [m/s^2] gravitational margin of "<<GRAVITY_MARGIN<<" [m/s^2] has been exceeded."<<RTT::endlog();
		    return exception(ALIGNMENT_ERROR);
		}
	    }
	    else
	    {
		RTT::log(RTT::Fatal)<<"[orientation_estimator] ERROR - NaN values in Initial Alignment."<<RTT::endlog();
		RTT::log(RTT::Fatal)<<"[orientation_estimator] This might be a configuration error or sensor fault."<<RTT::endlog();
		return exception(NAN_ERROR);
	    }
	}
	else
	{
	    RTT::log(RTT::Warning)<<"[orientation_estimator] Skipping inital alignment. Initial alignment duration was zero."<<RTT::endlog();
	}
	
	ikf_filter.setAttitude(initial_attitude);
	init_attitude = true;

	#ifdef DEBUG_PRINTS
	Eigen::Matrix <double,3,1> eulerprint;
	eulerprint[2] = initial_attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
	eulerprint[1] = initial_attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
	eulerprint[0] = initial_attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
	std::cout<< "******** Initial Attitude  *******"<<"\n";
	std::cout<< "Init Roll: "<<base::Angle::rad2Deg(eulerprint[0])<<" Init Pitch: "<<base::Angle::rad2Deg(eulerprint[1])<<" Init Yaw: "<<base::Angle::rad2Deg(eulerprint[2])<<"\n";
	#endif
    }
}

void IKF::writeOutput()
{
    if (init_attitude && !prev_ts.isNull())
    {
	orientation_out.time = prev_ts;
	orientation_out.orientation = ikf_filter.getAttitude();
	orientation_out.cov_orientation = ikf_filter.getCovariance().block<NUMAXIS, NUMAXIS>(0,0);
	Eigen::AngleAxisd angular_velocity_angle_axis = 
				    Eigen::AngleAxisd(Eigen::AngleAxisd(gyro_reading[2], Eigen::Vector3d::UnitZ()) * 
						    Eigen::AngleAxisd(gyro_reading[1], Eigen::Vector3d::UnitY()) * 
						    Eigen::AngleAxisd(gyro_reading[0], Eigen::Vector3d::UnitX()));
	orientation_out.angular_velocity = angular_velocity_angle_axis.angle() * angular_velocity_angle_axis.axis();
	_attitude_b_g.write(orientation_out);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See IKF.hpp for more detailed
// documentation about them.

bool IKF::configureHook()
{
    if (! IKFBase::configureHook())
        return false;
    
    /******************************************/
    /** Configuration of the attitude filter **/
    /******************************************/
    Eigen::Matrix< double, IKFSTATEVECTORSIZE , 1  > x_0; /** Initial vector state **/
    Eigen::Matrix3d Ra; /** Measurement noise covariance matrix for acc */
    Eigen::Matrix3d Rg; /** Measurement noise covariance matrix for gyros */
    Eigen::Matrix3d Rm; /** Measurement noise covariance matrix for mag */
    Eigen::Matrix <double, IKFSTATEVECTORSIZE, IKFSTATEVECTORSIZE> P_0; /** Initial covariance matrix **/
    Eigen::Matrix3d Qbg; /** Noise for the gyros bias instability **/
    Eigen::Matrix3d Qba; /** Noise for the acc bias instability **/
    double sqrtdelta_t = 0.0;

    /************************/
    /** Read configuration **/
    /************************/
    config = _filter_configuration.value();
    inertialnoise = _inertial_noise.value();
    adaptiveconfig = _adaptive_config.value();
    location = _location.value();

    /*************************/
    /** Noise configuration **/
    /*************************/
    if(config.correction_frequency == 0.0)
	config.correction_frequency = inertialnoise.bandwidth;
    
    if(config.correction_frequency > inertialnoise.bandwidth)
	sqrtdelta_t = sqrt(1.0/inertialnoise.bandwidth); /** Noise depends on frequency bandwidth **/
    else
	sqrtdelta_t = sqrt(1.0/config.correction_frequency); /** Noise depends on frequency bandwidth **/

    Ra = Eigen::Matrix3d::Zero();
    Ra(0,0) = inertialnoise.accresolut[0] + pow(inertialnoise.accrw[0]/sqrtdelta_t,2);
    Ra(1,1) = inertialnoise.accresolut[1] + pow(inertialnoise.accrw[1]/sqrtdelta_t,2);
    Ra(2,2) = inertialnoise.accresolut[2] + pow(inertialnoise.accrw[2]/sqrtdelta_t,2);

    Rg = Eigen::Matrix3d::Zero();
    Rg(0,0) = pow(inertialnoise.gyrorw[0]/sqrtdelta_t,2);
    Rg(1,1) = pow(inertialnoise.gyrorw[1]/sqrtdelta_t,2);
    Rg(2,2) = pow(inertialnoise.gyrorw[2]/sqrtdelta_t,2);

    Rm = Eigen::Matrix3d::Zero();
    Rm(0,0) = pow(inertialnoise.magrw[0]/sqrtdelta_t,2);
    Rm(1,1) = pow(inertialnoise.magrw[1]/sqrtdelta_t,2);
    Rm(2,2) = pow(inertialnoise.magrw[2]/sqrtdelta_t,2);

    /** Noise for error in gyros bias instability **/
    //TODO use asDiagonal
    Qbg.setZero();
    Qbg(0,0) = pow(inertialnoise.gbiasins[0],2);
    Qbg(1,1) = pow(inertialnoise.gbiasins[1],2);
    Qbg(2,2) = pow(inertialnoise.gbiasins[2],2);

    /** Noise for error in accelerometers bias instability **/
    Qba.setZero();
    Qba(0,0) = pow(inertialnoise.abiasins[0],2);
    Qba(1,1) = pow(inertialnoise.abiasins[1],2);
    Qba(2,2) = pow(inertialnoise.abiasins[2],2);

    /** Initial error covariance **/
    P_0 = Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Zero();
    P_0.block <3, 3> (0,0) = 1.0e-06 * Eigen::Matrix3d::Identity();//Error quaternion
    P_0.block <3, 3> (3,3) = 1.0e-06 * Eigen::Matrix3d::Identity();//Gyros bias
    P_0.block <3, 3> (6,6) = 1.0e-06 * Eigen::Matrix3d::Identity();//Accelerometers bias

    /** Theoretical Gravity **/
    double gravity = GRAVITY;
    if (location.latitude > 0.0 && location.latitude < 90.0)
    {
        gravity = BaseEstimator::GravityModel (location.latitude, location.altitude);
	#ifdef DEBUG_PRINTS
	    std::cout<< "GravityModel gives " << gravity << " instead of standard gravity " << GRAVITY << std::endl;
	#endif
    }

    /** Initialize the filter, including the adaptive part **/
    ikf_filter.Init(P_0, Ra, Rg, Rm, Eigen::Matrix3d::Zero(), Qbg, Qba, Eigen::Matrix3d::Zero(), 
		    gravity, location.dip_angle,
		    adaptiveconfig.M1, adaptiveconfig.M2, adaptiveconfig.gamma,
		    0, 0, 0);

    ikf_filter.setInitBias(inertialnoise.gbiasoff, inertialnoise.abiasoff, Eigen::Vector3d::Zero());

    /** Allignment configuration **/
    initial_alignment.acc.setZero();
    initial_alignment.gyro.setZero();
    initial_alignment.mag.setZero();

    /** Set the samples count to Zero **/
    initial_samples = 0;
    initial_alignment_ts.microseconds = 0;
    
    /** Initial attitude **/
    init_attitude = false;
    
    gyro_reading.setZero();
    
    max_time_delta = 0.1;
    
    prev_ts.microseconds = 0;
    
    acc_imu_sum.setZero();
    mag_imu_sum.setZero();
    imu_samples = 0;
    imu_start.microseconds = 0;
    
    /** Task states **/
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    /** Output variable **/
    orientation_out.invalidate();
    orientation_out.sourceFrame = config.source_frame_name;
    orientation_out.targetFrame = config.target_frame_name;
    orientation_out.orientation.setIdentity();
    orientation_out.cov_angular_velocity = Rg;

    #ifdef DEBUG_PRINTS
	std::cout<< "IKF:"<<"\n";
	std::cout<< "Rg\n"<<Rg<<"\n";
	std::cout<< "Ra\n"<<Ra<<"\n";
	std::cout<< "Rm\n"<<Rm<<"\n";
	std::cout<< "P_0\n"<<P_0<<"\n";
	std::cout<< "Qbg\n"<<Qbg<<"\n";
	std::cout<< "Qba\n"<<Qba<<"\n";
    #endif 

    return true;
}
bool IKF::startHook()
{
    if (! IKFBase::startHook())
        return false;
    return true;
}
void IKF::updateHook()
{    
    IKFBase::updateHook();
    
    /** Write estimated attitude to output port **/
    writeOutput();
    
    /** Write tast state if it has changed **/
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void IKF::errorHook()
{
    IKFBase::errorHook();
}
void IKF::stopHook()
{
    IKFBase::stopHook();
}
void IKF::cleanupHook()
{
    IKFBase::cleanupHook();
}
