/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "IKF.hpp"

#include "BaseEstimator.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

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

void IKF::fog_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &fog_samples_sample)
{
    /** Receive fog to body transformation **/
    Eigen::Affine3d fog2body;
    if (!_fog2body.get(ts, fog2body))
    {
        RTT::log(RTT::Error) << "skip, have no fog2body transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    /** Rotate measurements to body frame **/
    base::samples::IMUSensors transformed_fog_samples;
    transformed_fog_samples.time = fog_samples_sample.time;
    transformed_fog_samples.acc = fog2body.rotation() * fog_samples_sample.acc;
    transformed_fog_samples.gyro = fog2body.rotation() * fog_samples_sample.gyro;
    transformed_fog_samples.mag = fog2body.rotation() * fog_samples_sample.mag;
    
    /** Attitude filter **/
    if (!init_attitude && !_initial_orientation.connected())
    {
	/** Do initial alignment **/
	initialAlignment(ts, transformed_fog_samples, FOG);
    }
    
    if(init_attitude)
    {
	new_state = RUNNING;
	if(!prev_ts.isNull())
	{
	    double delta_t = (ts - prev_ts).toSeconds();
	    if(delta_t > max_time_delta)
		delta_t = max_time_delta;
	    
	    /** Eliminate Earth rotation **/
	    Eigen::Vector3d fog_gyro = transformed_fog_samples.gyro;
	    if(config.substract_earth_rotation)
	    {
		Eigen::Quaterniond q_body2world = ikf_filter.getAttitude();
		BaseEstimator::SubstractEarthRotation(&fog_gyro, &q_body2world, location.latitude);
	    }
	    
	    /** Augment gyro reading **/
	    if(config.fog_type == SINGLE_AXIS)
		gyro_reading.z() = fog_gyro.z();
	    else
		gyro_reading = fog_gyro;

	    /** Predict **/
	    ikf_filter.predict(gyro_reading, delta_t);
	    
	    if(config.fog_type == MULTI_AXIS)
	    {
		/** Filter accelerometer samples **/
		acc_fog_sum += transformed_fog_samples.acc;
		// TODO Use a better filter than a mean filter
		fog_samples++;
		if(fog_start.isNull())
		    fog_start = ts;
		else if(fog_samples >= 1 && (ts - fog_start).toSeconds() > (1.0/config.correction_frequency))
		{
		    base::Vector3d prev_euler = base::getEuler(ikf_filter.getAttitude());
		    
		    /** Update/Correction FOG accelerometers **/
		    Eigen::Vector3d acc_fog_mean = acc_fog_sum / (double)fog_samples;
		    Eigen::Matrix <double,3,1> aux; 
		    aux.setZero();
		    ikf_filter.update(aux, false, acc_fog_mean, true, aux, false);
		    
		    /** Exclude yaw angle from correction **/
		    base::Vector3d corrected_euler = base::getEuler(ikf_filter.getAttitude());
		    Eigen::Quaterniond corrected_attitide = Eigen::AngleAxisd(prev_euler[0], Eigen::Vector3d::UnitZ()) * 
							    Eigen::AngleAxisd(corrected_euler[1], Eigen::Vector3d::UnitY()) *
							    Eigen::AngleAxisd(corrected_euler[2], Eigen::Vector3d::UnitX());
		    ikf_filter.setAttitude(corrected_attitide);
		
		    acc_fog_sum.setZero();
		    fog_start.microseconds = 0;
		    fog_samples = 0;
		}
	    }
	}
	
	prev_ts = ts;
    }
}

void IKF::initial_orientationTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &initial_orientation_sample)
{
    if (!init_attitude)
    {
	Eigen::Quaterniond attitude = initial_orientation_sample.orientation;
	Eigen::Vector3d euler = base::getEuler(attitude);
	
	std::cout << "******** Init Attitude IKFEstimator *******\n";
	/** Eliminate the Magnetic declination from the initial attitude quaternion **/
	BaseEstimator::CorrectMagneticDeclination (&attitude, location.magnetic_declination, (int)location.magnetic_declination_mode);
	
	/** Set the initial attitude quaternion of the IKF **/
	ikf_filter.setAttitude (attitude);
	init_attitude = true;
	
	std::cout << "Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
	std::cout << "(Roll, Pitch, Yaw)\n"<< base::Angle::rad2Deg(euler.z()) << "," << base::Angle::rad2Deg(euler.y()) << "," << base::Angle::rad2Deg(euler.x()) << "\n";
	std::cout << "**********************\n";
    }
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
    
    /** Rotate measurements to body frame **/
    base::samples::IMUSensors transformed_imu_samples;
    transformed_imu_samples.time = imu_samples_sample.time;
    transformed_imu_samples.acc = imu2body.rotation() * imu_samples_sample.acc;
    transformed_imu_samples.gyro = imu2body.rotation() * imu_samples_sample.gyro;
    transformed_imu_samples.mag = imu2body.rotation() * imu_samples_sample.mag;
    
    /** Attitude filter **/
    if(!init_attitude && !_initial_orientation.connected())
    {
	//** Do initial alignment **/
	initialAlignment(ts, transformed_imu_samples, IMU);
    }
    
    if (init_attitude)
    {
	if(!prev_ts.isNull())
	{
	    new_state = RUNNING;
	    double delta_t = (ts - prev_ts).toSeconds();
	    if(delta_t > max_time_delta)
		delta_t = max_time_delta;
	    
	    /** Augment gyro reading **/
	    if(config.fog_type == SINGLE_AXIS || !_fog_samples.connected())
	    {
		/** Eliminate Earth rotation **/
		Eigen::Vector3d imu_gyro = transformed_imu_samples.gyro;
		if(config.substract_earth_rotation)
		{
		    Eigen::Quaterniond q_body2world = ikf_filter.getAttitude();
		    BaseEstimator::SubstractEarthRotation(&imu_gyro, &q_body2world, location.latitude);
		}
	    
		if(config.fog_type == SINGLE_AXIS)
		{
		    gyro_reading.x() = imu_gyro.x();
		    gyro_reading.y() = imu_gyro.y();
		}
		else
		{
		    gyro_reading = imu_gyro;
		}
	    }
	    
	    /** Predict **/
	    ikf_filter.predict(gyro_reading, delta_t);
	    
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
	
	prev_ts = ts;
    }
}

void IKF::initialAlignment(const base::Time &ts,  const base::samples::IMUSensors &imu_sample, SensorType type)
{
    #ifdef DEBUG_PRINTS
	std::cout<<"** [ORIENT_IKF] Initial Attitude[IMU: "<<initial_imu_samples<<", FOG: "<<initial_fog_samples<<"]\n";
    #endif
    new_state = INITIAL_ALIGNMENT;
    
    if(initial_alignment_ts.isNull())
	initial_alignment_ts = ts;

    if(type == IMU)
    {
	initial_alignment_imu.acc += imu_sample.acc;
	initial_alignment_imu.gyro += imu_sample.gyro;
	initial_alignment_imu.mag += imu_sample.mag;
	initial_imu_samples++;
    }
    else if(type == FOG)
    {
	initial_alignment_fog.acc += imu_sample.acc;
	initial_alignment_fog.gyro += imu_sample.gyro;
	initial_alignment_fog.mag += imu_sample.mag;
	initial_fog_samples++;
    }
    else
    {
	RTT::log(RTT::Fatal)<<"[orientation_estimator] Selected sensor type is unknown."<<RTT::endlog();
	return exception(CONFIGURATION_ERROR);
    }

    /** Calculate the initial alignment to the local geographic frame **/
    if ((ts - initial_alignment_ts).toSeconds() >= config.initial_alignment_duration)
    {
	/** Set attitude to identity **/
	Eigen::Quaterniond initial_attitude = Eigen::Quaterniond(Eigen::AngleAxisd(_initial_heading.value(), Eigen::Vector3d::UnitZ()));

	if (config.initial_alignment_duration > 0)
	{
	    if(initial_imu_samples == 0 && config.fog_type == SINGLE_AXIS)
	    {
		RTT::log(RTT::Fatal)<<"[orientation_estimator] Can't do the inital alignment with a single axis FOG and no IMU."<<RTT::endlog();
		return exception(CONFIGURATION_ERROR);
	    }
	    
	    /** Compute mean values **/
	    if(initial_imu_samples > 0)
	    {
		initial_alignment_imu.acc /= (double)initial_imu_samples;
		initial_alignment_imu.gyro /= (double)initial_imu_samples;
		initial_alignment_imu.mag /= (double)initial_imu_samples;
	    }
	    if(initial_fog_samples > 0)
	    {
		initial_alignment_fog.acc /= (double)initial_fog_samples;
		initial_alignment_fog.gyro /= (double)initial_fog_samples;
		initial_alignment_fog.mag /= (double)initial_fog_samples;
	    }

	    if ((base::isnotnan(initial_alignment_imu.acc)) && (base::isnotnan(initial_alignment_imu.gyro)))
	    {
		if ((initial_alignment_imu.acc.norm() < (GRAVITY+GRAVITY_MARGIN) && initial_alignment_imu.acc.norm() > (GRAVITY-GRAVITY_MARGIN)) || 
		    (config.fog_type == MULTI_AXIS && initial_alignment_fog.acc.norm() < (GRAVITY+GRAVITY_MARGIN) && initial_alignment_fog.acc.norm() > (GRAVITY-GRAVITY_MARGIN)))
		{
		    /** use imu accelerometers as reference, if imu is connected **/
		    Eigen::Vector3d meanacc = initial_alignment_imu.acc;
		    if(initial_imu_samples == 0)
			meanacc = initial_alignment_fog.acc;

		    /** Override the gravity model value with the sensed from the sensors **/
		    if (config.use_samples_as_theoretical_gravity)
			ikf_filter.setGravity(meanacc.norm());

		    /** Compute the local horizontal plane **/
		    Eigen::Vector3d euler;
		    euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
		    euler[1] = (double) -atan(meanacc[0]/meanacc[2]); //Pitch
		    euler[2] = 0.0; //Yaw

		    /** Set the attitude  **/
		    initial_attitude = Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

		    #ifdef DEBUG_PRINTS
		    std::cout<< "******** Local Horizontal *******"<<"\n";
		    std::cout<< "Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
		    #endif

		    /** The angular velocity in the local horizontal plane **/
		    /** Gyro_ho = Tho_body * gyro_body **/
		    Eigen::Vector3d meangyro = initial_alignment_imu.gyro;
		    if(initial_fog_samples > 0 && config.fog_type == SINGLE_AXIS)
			meangyro[2] = initial_alignment_fog.gyro[2];
		    if(initial_fog_samples > 0 && config.fog_type == MULTI_AXIS)
			meangyro = initial_alignment_fog.gyro;
		    
		    Eigen::Vector3d transformed_meangyro = initial_attitude * meangyro;

		    /** Determine the initial heading **/
		    if (transformed_meangyro.x() == 0.0 && transformed_meangyro.y() == 0.0)
		    {
			RTT::log(RTT::Warning) << "Couldn't estimate initial heading. Earth rotaion was estimated as zero." << RTT::endlog();
			euler[2] = _initial_heading.value(); //Yaw
		    }
		    else
			euler[2] = base::Angle::fromRad(-atan2(transformed_meangyro.y(), transformed_meangyro.x())).getRad();
		    /** Set the attitude  **/
		    initial_attitude = Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

		    #ifdef DEBUG_PRINTS
		    std::cout<< " Mean Gyro:\n"<<transformed_meangyro<<"\n Mean Acc:\n"<<meanacc<<"\n";
		    std::cout<< " Earth rot * cos(lat): "<<EARTHW*cos(location.latitude)<<"\n";
		    std::cout<< " Filter Gravity: "<<ikf_filter.getGravity()[2]<<"\n";
		    std::cout<< "******** Azimuthal Orientation *******"<<"\n";
		    std::cout<< " Yaw: "<<euler[2]*R2D<<"\n";
		    #endif

		    /** Compute the Initial Bias **/
		    Eigen::Vector3d gyro_bias = meangyro;
		    if(config.substract_earth_rotation)
			BaseEstimator::SubstractEarthRotation(&gyro_bias, &initial_attitude, location.latitude);
		    
		    Eigen::Vector3d imu_acc_bias = ikf_filter.getAccBias();
		    Eigen::Vector3d fog_acc_bias = ikf_filter.getInclBias();
		    if(initial_imu_samples > 0)
			imu_acc_bias = initial_alignment_imu.acc - initial_attitude.inverse() * ikf_filter.getGravity();
		    if(initial_fog_samples > 0 && config.fog_type == MULTI_AXIS)
			fog_acc_bias = initial_alignment_fog.acc - initial_attitude.inverse() * ikf_filter.getGravity();
		    
		    ikf_filter.setInitBias (gyro_bias, imu_acc_bias, fog_acc_bias);

		    #ifdef DEBUG_PRINTS
		    std::cout<< "******** Initial Bias Offset *******"<<"\n";
		    std::cout<< " Gyroscopes Bias Offset:\n"<<ikf_filter.getGyroBias()<<"\n";
		    std::cout<< " Accelerometers Bias Offset:\n"<<ikf_filter.getAccBias()<<"\n";
		    std::cout<< " Inclinometers Bias Offset:\n"<<ikf_filter.getInclBias()<<"\n";
		    #endif
		}
		else
		{
		    RTT::log(RTT::Fatal)<<"[orientation_estimator] ERROR in Initial Alignment. Unable to compute reliable attitude."<<RTT::endlog();
		    RTT::log(RTT::Fatal)<<"[orientation_estimator] Computed "<< initial_alignment_imu.acc.norm() <<" [m/s^2] gravitational margin of "<<GRAVITY_MARGIN<<" [m/s^2] has been exceeded."<<RTT::endlog();
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
	
	ikf_filter.setAttitude(initial_attitude);
	init_attitude = true;

	#ifdef DEBUG_PRINTS
	Eigen::Matrix <double,3,1> eulerprint;
	eulerprint[2] = initial_attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
	eulerprint[1] = initial_attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
	eulerprint[0] = initial_attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
	std::cout<< "******** Initial Attitude  *******"<<"\n";
	std::cout<< "Init Roll: "<<eulerprint[0]*R2D<<" Init Pitch: "<<eulerprint[1]*R2D<<" Init Yaw: "<<eulerprint[2]*R2D<<"\n";
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
	orientation_out.cov_angular_velocity = gyro_measurement_noise;
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
    Eigen::Matrix3d Ra; /** Measurement noise covariance matrix for IMU acc */
    Eigen::Matrix3d Rg; /** Measurement noise covariance matrix for gyros */
    Eigen::Matrix3d Rm; /** Measurement noise covariance matrix for mag */
    Eigen::Matrix3d Ra2; /** Measurement noise covariance matrix for FOG acc */
    Eigen::Matrix <double, IKFSTATEVECTORSIZE, IKFSTATEVECTORSIZE> P_0; /** Initial covariance matrix **/
    Eigen::Matrix3d Qbg; /** Noise for the gyros bias instability **/
    Eigen::Matrix3d Qba; /** Noise for the IMU acc bias instability **/
    Eigen::Matrix3d Qba2; /** Noise for the FOG acc bias instability **/
    double sqrtdelta_t_imu, sqrtdelta_t_fog;

    /************************/
    /** Read configuration **/
    /************************/
    config = _filter_configuration.value();
    inertialnoise_imu = _inertial_noise_imu.value();
    inertialnoise_fog = _inertial_noise_fog.value();
    adaptiveconfig_acc_imu = _adaptive_config_acc_imu.value();
    adaptiveconfig_acc_fog = _adaptive_config_acc_fog.value();
    location = _location.value();

    /*************************/
    /** Noise configuration **/
    /*************************/
    sqrtdelta_t_imu = sqrt(1.0/inertialnoise_imu.bandwidth);
    sqrtdelta_t_fog = sqrt(1.0/inertialnoise_fog.bandwidth);
    if(config.correction_frequency == 0.0)
	config.correction_frequency = inertialnoise_imu.bandwidth;
    double sqrtdelta_t_acc = 1.0;
    if(config.correction_frequency > inertialnoise_imu.bandwidth)
	sqrtdelta_t_acc = sqrt(1.0/inertialnoise_imu.bandwidth); /** Noise depends on frequency bandwidth **/
    else
	sqrtdelta_t_acc = sqrt(1.0/config.correction_frequency); /** Noise depends on frequency bandwidth **/

    Ra = Eigen::Matrix3d::Zero();
    Ra(0,0) = inertialnoise_imu.accresolut[0] + pow(inertialnoise_imu.accrw[0]/sqrtdelta_t_acc,2);
    Ra(1,1) = inertialnoise_imu.accresolut[1] + pow(inertialnoise_imu.accrw[1]/sqrtdelta_t_acc,2);
    Ra(2,2) = inertialnoise_imu.accresolut[2] + pow(inertialnoise_imu.accrw[2]/sqrtdelta_t_acc,2);

    Rg = Eigen::Matrix3d::Zero();
    if(config.fog_type == SINGLE_AXIS)
    {
	Rg(0,0) = pow(inertialnoise_imu.gyrorw[0]/sqrtdelta_t_imu,2);
	Rg(1,1) = pow(inertialnoise_imu.gyrorw[1]/sqrtdelta_t_imu,2);
    }
    else
    {
	Rg(0,0) = pow(inertialnoise_fog.gyrorw[0]/sqrtdelta_t_fog,2);
	Rg(1,1) = pow(inertialnoise_fog.gyrorw[1]/sqrtdelta_t_fog,2);
    }
    Rg(2,2) = pow(inertialnoise_fog.gyrorw[2]/sqrtdelta_t_fog,2);
    gyro_measurement_noise = Rg;

    Rm = Eigen::Matrix3d::Zero();
    Rm(0,0) = pow(inertialnoise_imu.magrw[0]/sqrtdelta_t_imu,2);
    Rm(1,1) = pow(inertialnoise_imu.magrw[1]/sqrtdelta_t_imu,2);
    Rm(2,2) = pow(inertialnoise_imu.magrw[2]/sqrtdelta_t_imu,2);

    Ra2 = Eigen::Matrix3d::Zero();
    Ra2(0,0) = inertialnoise_fog.accresolut[0] + pow(inertialnoise_fog.accrw[0]/sqrtdelta_t_acc,2);
    Ra2(1,1) = inertialnoise_fog.accresolut[1] + pow(inertialnoise_fog.accrw[1]/sqrtdelta_t_acc,2);
    Ra2(2,2) = inertialnoise_fog.accresolut[2] + pow(inertialnoise_fog.accrw[2]/sqrtdelta_t_acc,2);

    /** Noise for error in gyros bias instability **/
    //TODO use asDiagonal
    Qbg.setZero();
    if(config.fog_type == SINGLE_AXIS)
    {
	Qbg(0,0) = pow(inertialnoise_imu.gbiasins[0],2);
	Qbg(1,1) = pow(inertialnoise_imu.gbiasins[1],2);
    }
    else
    {
	Qbg(0,0) = pow(inertialnoise_fog.gbiasins[0],2);
	Qbg(1,1) = pow(inertialnoise_fog.gbiasins[1],2);
    }
    Qbg(2,2) = pow(inertialnoise_fog.gbiasins[2],2);

    /** Noise for error in IMU accelerometers bias instability **/
    Qba.setZero();
    Qba(0,0) = pow(inertialnoise_imu.abiasins[0],2);
    Qba(1,1) = pow(inertialnoise_imu.abiasins[1],2);
    Qba(2,2) = pow(inertialnoise_imu.abiasins[2],2);

    /** Noise for error in FOG accelerometers bias instability **/
    Qba2.setZero();
    Qba2(0,0) = pow(inertialnoise_fog.abiasins[0],2);
    Qba2(1,1) = pow(inertialnoise_fog.abiasins[1],2);
    Qba2(2,2) = pow(inertialnoise_fog.abiasins[2],2);


    /** Initial error covariance **/
    P_0 = Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Zero();
    P_0.block <3, 3> (0,0) = 1.0e-06 * Eigen::Matrix3d::Identity();//Error quaternion
    P_0.block <3, 3> (3,3) = 1.0e-06 * Eigen::Matrix3d::Identity();//Gyros bias
    P_0.block <3, 3> (6,6) = 1.0e-06 * Eigen::Matrix3d::Identity();//IMU Accelerometers bias
    P_0.block <3, 3> (9,9) = 1.0e-06 * Eigen::Matrix3d::Identity();//FOG Accelerometers bias

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
    ikf_filter.Init(P_0, Ra, Rg, Rm, Ra2, Qbg, Qba, Qba2, gravity, location.dip_angle,
            adaptiveconfig_acc_imu.M1, adaptiveconfig_acc_imu.M2, adaptiveconfig_acc_imu.gamma,
            adaptiveconfig_acc_fog.M1, adaptiveconfig_acc_fog.M2, adaptiveconfig_acc_fog.gamma);

    base::Vector3d gbiasoff = inertialnoise_fog.gbiasoff;
    if(config.fog_type == SINGLE_AXIS)
    {
	gbiasoff.x() = inertialnoise_imu.gbiasoff.x();
	gbiasoff.y() = inertialnoise_imu.gbiasoff.y();
    }
    if(!_fog_samples.connected())
    {
	gbiasoff = inertialnoise_imu.gbiasoff;
    }
    ikf_filter.setInitBias(gbiasoff, inertialnoise_imu.abiasoff, inertialnoise_fog.abiasoff);

    /** Allignment configuration **/
    initial_alignment_imu.acc.setZero();
    initial_alignment_imu.gyro.setZero();
    initial_alignment_imu.mag.setZero();
    initial_alignment_fog.acc.setZero();
    initial_alignment_fog.gyro.setZero();
    initial_alignment_fog.mag.setZero();

    /** Set the samples count to Zero **/
    initial_imu_samples = 0;
    initial_fog_samples = 0;
    initial_alignment_ts.microseconds = 0;
    
    /** Initial attitude **/
    init_attitude = false;
    
    gyro_reading.setZero();
    
    max_time_delta = 0.1;
    
    prev_ts.microseconds = 0;
    
    acc_imu_sum.setZero();
    acc_fog_sum.setZero();
    mag_imu_sum.setZero();
    imu_samples = 0;
    fog_samples = 0;
    imu_start.microseconds = 0;
    fog_start.microseconds = 0;
    
    /** Task states **/
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    /** Output variable **/
    orientation_out.invalidate();
    orientation_out.sourceFrame = config.source_frame_name;
    orientation_out.targetFrame = config.target_frame_name;
    orientation_out.orientation.setIdentity();

    #ifdef DEBUG_PRINTS
	std::cout<< "IKF:"<<"\n";
	std::cout<< "Rg\n"<<Rg<<"\n";
	std::cout<< "Ra\n"<<Ra<<"\n";
	std::cout<< "Rm\n"<<Rm<<"\n";
	std::cout<< "Ra2\n"<<Ra2<<"\n";
	std::cout<< "P_0\n"<<P_0<<"\n";
	std::cout<< "Qbg\n"<<Qbg<<"\n";
	std::cout<< "Qba\n"<<Qba<<"\n";
	std::cout<< "Qba2\n"<<Qba2<<"\n";
    #endif
    
    /** Info and Warnings about the Task **/
    if (_initial_orientation.connected())
    {
	RTT::log(RTT::Info) << "Initial orientation is connected." << RTT::endlog();
	RTT::log(RTT::Info) << "Initial orientation used from the first sample."<< RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Info) << "Initial orientation is not connected." << RTT::endlog();

	if(config.initial_alignment_duration > 0.0)
	    RTT::log(RTT::Info) << "Seeking initial yaw by measuring the earth rotation."<< RTT::endlog();
	else
	    RTT::log(RTT::Info) << "Using initial_heading paramter as initial yaw."<< RTT::endlog();

	if(config.fog_type == MULTI_AXIS)
	    RTT::log(RTT::Info) << "Find initial roll and pitch by filtering FOG's accelerometers."<< RTT::endlog();
    }

    if (_fog_samples.connected())
    {
	RTT::log(RTT::Info) << "FOG is connected" << RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "FOG NOT connected" << RTT::endlog();
	RTT::log(RTT::Info) << "Orientation will be calculated from the IMU samples." << RTT::endlog();
    }

    if (_imu_samples.connected())
    {
	RTT::log(RTT::Info) << "IMU is connected" << RTT::endlog();
	
	if(!_fog_samples.connected() || config.fog_type == SINGLE_AXIS)
	    RTT::log(RTT::Info) << "Find initial roll and pitch by filtering IMU's accelerometers."<< RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "IMU NOT connected." << RTT::endlog();
	if(config.fog_type == SINGLE_AXIS)
	    RTT::log(RTT::Warning) << "Potential malfunction on the task!!" << RTT::endlog();
    }    

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
