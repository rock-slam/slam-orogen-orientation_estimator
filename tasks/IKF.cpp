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

void IKF::fog_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &fog_samples_sample)
{
    /** Attitude filter **/
    if (!init_attitude && !_imu_orientation.connected() && config.fog_type == MULTI_AXIS)
    {
	/** Do initial north seeking **/
	if(config.do_initial_north_seeking)
	{
	    //TODO throw this out
	    new_state = INITIAL_NORTH_SEEKING;
	    acc_gyro.x() += fog_samples_sample.gyro[0];
	    acc_gyro.y() += fog_samples_sample.gyro[1];
	    acc_gyro.z() += fog_samples_sample.gyro[2];
	    
	    if((base::Time::now() - start_seeking).toSeconds() >= config.initial_alignment_duration)
	    {
		initial_heading = base::Angle::fromRad(- atan2(acc_gyro.y(), acc_gyro.x()));
		acc_gyro.setZero();
		config.do_initial_north_seeking = false;
	    }
	}
	
	/** Set initial attitude **/
	if(!config.do_initial_north_seeking)
	{
	    if (config.use_inclinometers)
		initialAlignment(ts ,fog_samples_sample.mag, fog_samples_sample.gyro);
	    else
		initialAlignment(ts, fog_samples_sample.acc, fog_samples_sample.gyro);
	}
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
	    Eigen::Vector3d fog_gyro = fog_samples_sample.gyro;
	    Eigen::Quaterniond q_body2world = ikf_filter.getAttitude();
	    BaseEstimator::SubstractEarthRotation(&fog_gyro, &q_body2world, location.latitude);
	    
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
		// TODO Use a better filter than a mean filter
		acca_fog += fog_samples_sample.acc;
		acca_fog_samples++;
		if(acca_fog_start.isNull())
		    acca_fog_start = ts;
		else if(acca_fog_samples >= 1 && (ts - acca_fog_start).toSeconds() > (1.0/config.correction_frequency))
		{
		    base::Vector3d prev_euler = base::getEuler(ikf_filter.getAttitude());
		    
		    /** Update/Correction FOG accelerometers **/
		    acca_fog /= (double)acca_fog_samples;
		    Eigen::Matrix <double,3,1> aux; 
		    aux.setZero();
		    ikf_filter.update(aux, false, acca_fog, true, aux, false);
		    
		    /** Exclude yaw angle from correction **/
		    base::Vector3d corrected_euler = base::getEuler(ikf_filter.getAttitude());
		    Eigen::Quaterniond corrected_attitide = Eigen::AngleAxisd(prev_euler[0], Eigen::Vector3d::UnitZ()) * 
							    Eigen::AngleAxisd(corrected_euler[1], Eigen::Vector3d::UnitY()) *
							    Eigen::AngleAxisd(corrected_euler[2], Eigen::Vector3d::UnitX());
		    ikf_filter.setAttitude(corrected_attitide);
		
		    acca_fog.setZero();
		    acca_fog_start.microseconds = 0;
		    acca_fog_samples = 0;
		}
	    }
	}
	
	prev_ts = ts;
    }
}

void IKF::imu_orientationCallback(const base::Time &ts, const ::base::samples::RigidBodyState &imu_orientation_sample)
{
    if (!init_attitude)
    {
	Eigen::Quaternion <double> attitude (imu_orientation_sample.orientation.w(), imu_orientation_sample.orientation.x(),
	imu_orientation_sample.orientation.y(), imu_orientation_sample.orientation.z());
	Eigen::Matrix <double, NUMAXIS, 1> euler;
	
	std::cout << "******** Init Attitude IKFEstimator *******\n";
	/** Eliminate the Magnetic declination from the initial attitude quaternion **/
	BaseEstimator::CorrectMagneticDeclination (&attitude, location.magnetic_declination, (int)location.magnetic_declination_mode);
	
	/** Set the initial attitude quaternion of the IKF **/
	ikf_filter.setAttitude (attitude);
	init_attitude = true;    

	euler[2] = base::getEuler(attitude)[0];//YAW
	euler[1] = base::getEuler(attitude)[1];//PITCH
	euler[0] = base::getEuler(attitude)[2];//ROLL
	
	std::cout << "Orientation (Quaternion): "<< attitude.w()<<","<<attitude.x()<<","<<attitude.y()<<","<<attitude.z()<<"\n";
	std::cout << "(Roll, Pitch, Yaw)\n"<< base::Angle::rad2Deg(euler.x()) << "," << base::Angle::rad2Deg(euler.y()) << "," << base::Angle::rad2Deg(euler.z()) << "\n";
	std::cout << "**********************\n";
    }
}

void IKF::imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{
    /** Attitude filter **/
    if(!init_attitude && !_imu_orientation.connected())
    {
	/** Set initial attitude **/
	if (config.fog_type == SINGLE_AXIS || !_fog_samples.connected())
	    initialAlignment(ts, imu_samples_sample.acc, imu_samples_sample.gyro);
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
		Eigen::Vector3d imu_gyro = imu_samples_sample.gyro;
		Eigen::Quaterniond q_body2world = ikf_filter.getAttitude();
		BaseEstimator::SubstractEarthRotation(&imu_gyro, &q_body2world, location.latitude);
	    
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
	    
	    /** Update/Correction magnetometers **/
	    Eigen::Matrix <double,3,1> aux; 
	    aux.setZero();
	    //TODO maybe exclude rotation around x and y from magnetometer correction
	    if(config.use_magnetometers)
		ikf_filter.update(aux, false, aux, false, imu_samples_sample.mag, true);

	    /** Filter accelerometer samples **/
	    // TODO Use a better filter than a mean filter
	    acca_imu += imu_samples_sample.acc;
	    acca_imu_samples++;
	    if(acca_imu_start.isNull())
		acca_imu_start = ts;
	    else if(acca_imu_samples >= 1 && (ts - acca_imu_start).toSeconds() > (1.0/config.correction_frequency))
	    {
		base::Vector3d prev_euler = base::getEuler(ikf_filter.getAttitude());
		
		/** Update/Correction IMU accelerometers **/
		acca_imu /= (double)acca_imu_samples;
		ikf_filter.update(acca_imu, true, aux, false, aux, false);

		/** Exclude yaw angle from correction **/
		base::Vector3d corrected_euler = base::getEuler(ikf_filter.getAttitude());
		Eigen::Quaterniond corrected_attitide = Eigen::AngleAxisd(prev_euler[0], Eigen::Vector3d::UnitZ()) * 
							Eigen::AngleAxisd(corrected_euler[1], Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(corrected_euler[2], Eigen::Vector3d::UnitX());
		ikf_filter.setAttitude(corrected_attitide);

		acca_imu.setZero();
		acca_imu_start.microseconds = 0;
		acca_imu_samples = 0;
	    }
	}
	
	prev_ts = ts;
    }
}

void IKF::initialAlignment(const base::Time &ts, const Eigen::Vector3d& acc_sample, const Eigen::Vector3d& gyro_sample)
{
    #ifdef DEBUG_PRINTS
	std::cout<<"** [ORIENT_IKF] Initial Attitude["<<initial_alignment_idx<<"]\n";
    #endif
    new_state = INITIAL_ALIGNMENT;
    
    if(initial_alignment_ts.isNull())
	initial_alignment_ts = ts;

    initial_alignment_acc += acc_sample;
    initial_alignment_gyro += gyro_sample;
    initial_alignment_idx++;

    /** Calculate the initial alignment to the local geographic frame **/
    if ((ts - initial_alignment_ts).toSeconds() >= config.initial_alignment_duration)
    {
	/** Set attitude to identity **/
	Eigen::Quaterniond initial_attitude = Eigen::Quaterniond(Eigen::AngleAxisd(initial_heading.getRad(), Eigen::Vector3d::UnitZ()));

	if (config.initial_alignment_duration > 0)
	{
	    /** Acceleration **/
	    Eigen::Vector3d meanacc = initial_alignment_acc / (double)initial_alignment_idx;

	    /** Angular velocity **/
	    Eigen::Vector3d meangyro = initial_alignment_gyro / (double)initial_alignment_idx;

	    if ((base::isnotnan(meanacc)) && (base::isnotnan(meangyro)))
	    {
		if (meanacc.norm() < (GRAVITY+GRAVITY_MARGIN))
		{
		    Eigen::Matrix <double,3,1> euler;

		    /** Override the gravity model value with the sensed from the sensors **/
		    if (config.use_samples_as_theoretical_gravity)
			ikf_filter.setGravity(meanacc.norm());

		    /** Compute the local horizontal plane **/
		    euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
		    euler[1] = (double) -atan(meanacc[0]/meanacc[2]); //Pitch
		    euler[2] = initial_heading.getRad(); //Yaw

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
		    Eigen::Vector3d transformed_meangyro = initial_attitude * meangyro;

		    /** Determine the heading or azimuthal orientation **/
		    // TODO check north computation
		    if (transformed_meangyro[0] == 0.00)
			euler[2] = 90.0*D2R - atan(transformed_meangyro[0]/transformed_meangyro[1]);
			//initial_heading = base::Angle::fromRad(- atan2(acc_gyro.y(), acc_gyro.x()));
		    else
			euler[2] = atan(transformed_meangyro[1]/transformed_meangyro[0]);

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
		    BaseEstimator::SubstractEarthRotation(&meangyro, &initial_attitude, location.latitude);
		    meanacc = meanacc - initial_attitude.inverse() * ikf_filter.getGravity();

		    if (config.use_inclinometers)
			ikf_filter.setInitBias (meangyro, Eigen::Matrix<double, 3, 1>::Zero(), meanacc);
		    else
			ikf_filter.setInitBias (meangyro, meanacc, Eigen::Matrix<double, 3, 1>::Zero());

		    #ifdef DEBUG_PRINTS
		    std::cout<< "******** Initial Bias Offset *******"<<"\n";
		    std::cout<< " Gyroscopes Bias Offset:\n"<<ikf_filter.getGyroBias()<<"\n";
		    std::cout<< " Accelerometers Bias Offset:\n"<<ikf_filter.getAccBias()<<"\n";
		    std::cout<< " Inclinometers Bias Offset:\n"<<ikf_filter.getInclBias()<<"\n";
		    #endif
		}
		else
		{
		    RTT::log(RTT::Fatal)<<"[STIM300] ERROR in Initial Alignment. Unable to compute reliable attitude."<<RTT::endlog();
		    RTT::log(RTT::Fatal)<<"[STIM300] Computed "<< meanacc.norm() <<" [m/s^2] gravitational margin of "<<GRAVITY_MARGIN<<" [m/s^2] has been exceeded."<<RTT::endlog();
		    return exception(ALIGNMENT_ERROR);
		}
	    }
	    else
	    {
		RTT::log(RTT::Fatal)<<"[STIM300] ERROR - NaN values in Initial Alignment."<<RTT::endlog();
		RTT::log(RTT::Fatal)<<"[STIM300] This might be a configuration error or sensor fault."<<RTT::endlog();
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
    //TODO adapt to using only acc, gyr
    ikf_filter.Init(P_0, Ra, Rg, Rm, Ra2, Qbg, Qba, Qba2, gravity, location.dip_angle,
            adaptiveconfig_acc_imu.M1, adaptiveconfig_acc_imu.M2, adaptiveconfig_acc_imu.gamma,
            adaptiveconfig_acc_fog.M1, adaptiveconfig_acc_fog.M2, adaptiveconfig_acc_fog.gamma);

    //TODO check if inital bias settings help
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
    initial_alignment_gyro.setZero();
    initial_alignment_acc.setZero();

    /** Set the index to Zero **/
    initial_alignment_idx = 0;
    initial_alignment_ts.microseconds = 0;
    
    /** Initial attitude **/
    init_attitude = false;
    
    gyro_reading.setZero();
    
    max_time_delta = 0.1;
    
    prev_ts.microseconds = 0;
    
    acca_imu.setZero();
    acca_fog.setZero();
    acca_imu_samples = 0;
    acca_fog_samples = 0;
    acca_imu_start.microseconds = 0;
    acca_fog_start.microseconds = 0;
    
    /** North seeking **/
    start_seeking = base::Time::now();
    acc_gyro.setZero();
    initial_heading = base::Angle::fromRad(_initial_heading.get());
    
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
    // TODO rewrite configuration warnings
    if (_imu_orientation.connected())
    {
	RTT::log(RTT::Info) << "IMU orientation is connected" << RTT::endlog();
	RTT::log(RTT::Info) << "Initial orientation used from the first sample."<< RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Info) << "IMU orientation is not connected." << RTT::endlog();

	if(config.do_initial_north_seeking)
	    RTT::log(RTT::Info) << "Seeking initial yaw by measuring the earth rotation."<< RTT::endlog();
	else
	    RTT::log(RTT::Info) << "Using initial_heading paramter as initial yaw."<< RTT::endlog();

	if(config.use_inclinometers)
	    RTT::log(RTT::Info) << "Find initial roll and pitch by filtering FOG's inclinometers."<< RTT::endlog();
	else if(config.fog_type == MULTI_AXIS)
	    RTT::log(RTT::Info) << "Find initial roll and pitch by filtering FOG's accelerometers."<< RTT::endlog();
    }

    if (_fog_samples.connected())
    {
	RTT::log(RTT::Info) << "FOG is connected" << RTT::endlog();
	
	if(config.fog_type == SINGLE_AXIS && config.do_initial_north_seeking) 
	{
	    RTT::log(RTT::Warning) << "Can't do initial north seeking on a single axis FOG." << RTT::endlog();
	    RTT::log(RTT::Warning) << "Potential malfunction on the task!!" << RTT::endlog();
	}
	if(config.fog_type == SINGLE_AXIS && config.use_inclinometers)
	{
	    RTT::log(RTT::Warning) << "Can't find initial roll and pitch orientation using a single axis FOG." << RTT::endlog();
	    RTT::log(RTT::Warning) << "Potential malfunction on the task!!" << RTT::endlog();
	}
    }
    else
    {
	RTT::log(RTT::Warning) << "FOG NOT connected" << RTT::endlog();
	if(config.do_initial_north_seeking || config.use_inclinometers)
	    RTT::log(RTT::Warning) << "Potential malfunction on the task!!" << RTT::endlog();
	else
	    RTT::log(RTT::Info) << "Orientation will be calculated from the IMU samples." << RTT::endlog();
    }

    if (_imu_samples.connected())
    {
	RTT::log(RTT::Info) << "IMU samples is connected" << RTT::endlog();
	
	if(!config.use_inclinometers && config.fog_type == SINGLE_AXIS)
	    RTT::log(RTT::Info) << "Find initial roll and pitch by filtering IMU's accelerometers."<< RTT::endlog();
    }
    else
    {
	RTT::log(RTT::Warning) << "IMU samples NOT connected." << RTT::endlog();
	if(!config.use_inclinometers && config.fog_type == SINGLE_AXIS)
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