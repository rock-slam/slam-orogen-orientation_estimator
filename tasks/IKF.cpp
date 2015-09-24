/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "IKF.hpp"

#define DEBUG_PRINTS 1

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
    if((config.use_magnetometers || config.use_inclinometers) && !base::isnotnan(imu_samples_sample.mag))
    {
        RTT::log(RTT::Fatal) << "ERROR: Magnetometer or inclinometer readings contain NaN values!" << RTT::endlog();
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
        /** Do initial alignment **/
        this->initialAlignment(ts, transformed_imu_samples);
    }
    else
    {
	if(!prev_ts.isNull())
	{
            new_state = RUNNING;
            delta_t = (ts - prev_ts).toSeconds();

            if(delta_t > max_time_delta)
            {
                RTT::log(RTT::Warning) << "Time delta exceeds maximum allowed time delta." << RTT::endlog();
                RTT::log(RTT::Warning) << "Predition step size will be limited to max_time_delta." << RTT::endlog();
                delta_t = max_time_delta;
            }

            Eigen::Vector3d acc, gyro, inc;
            acc = transformed_imu_samples.acc;
            gyro = transformed_imu_samples.gyro;
            inc = transformed_imu_samples.mag;

            acc_body = acc; // save this to be output later after being corrected

            /** Eliminate Earth rotation **/
            if(config.substract_earth_rotation)
            {
                Eigen::Quaterniond q_body2world = ikf_filter.getAttitude();
                BaseEstimator::SubtractEarthRotation(gyro, q_body2world.inverse(), location.latitude);
            }

            /** Predict **/
            ikf_filter.predict(gyro, delta_t);

            /** Accumulate correction measurements **/
            correctionAcc += acc; correctionInc += inc;
            correction_idx++;

            if (correction_idx == correction_numbers)
            {
                acc = correctionAcc / (double)correction_numbers;

                inc = correctionInc / (double)correction_numbers;

		base::Vector3d prev_euler = base::getEuler(ikf_filter.getAttitude());

                /** Update/Correction **/
                ikf_filter.update(acc, true, inc, config.use_inclinometers);

		/** Exclude yaw angle from accelerometers correction step **/
		base::Vector3d corrected_euler = base::getEuler(ikf_filter.getAttitude());
		Eigen::Quaterniond corrected_attitide = Eigen::AngleAxisd(prev_euler[0], Eigen::Vector3d::UnitZ()) *
							Eigen::AngleAxisd(corrected_euler[1], Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(corrected_euler[2], Eigen::Vector3d::UnitX());
		ikf_filter.setAttitude(corrected_attitide);

                correctionAcc.setZero();
                correctionInc.setZero();
                correction_idx = 0;
            }

    	}
        prev_ts = ts;
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
    Eigen::Matrix3d Ra; /** Measurement noise covariance matrix for accelerometers */
    Eigen::Matrix3d Rg; /** Measurement noise covariance matrix for gyros */
    Eigen::Matrix3d Rm; /** Measurement noise covariance matrix for mag */
    Eigen::Matrix3d Ri; /** Measurement noise covariance matrix for inclinometers */
    Eigen::Matrix <double, IKFSTATEVECTORSIZE, IKFSTATEVECTORSIZE> P_0; /** Initial covariance matrix **/
    Eigen::Matrix3d Qbg; /** Noise for the gyros bias instability **/
    Eigen::Matrix3d Qba; /** Noise for the accelerometers bias instability **/
    Eigen::Matrix3d Qbi; /** Noise for the inclinometers bias instability **/
    double sqrtdelta_t = 0.0;

    /************************/
    /** Read configuration **/
    /************************/
    config = _filter_configuration.value();
    accnoise = _accelerometer_noise.value();
    gyronoise = _gyroscope_noise.value();
    incnoise = _inclinometer_noise.value();
    adaptiveconfigAcc = _adaptive_config_acc.value();
    adaptiveconfigInc = _adaptive_config_inc.value();
    location = _location.value();
    
    /****************************/
    /** Check misconfiguration **/
    /****************************/
    if(_imu_samples_period.value() <= 0.0)
    {
	RTT::log(RTT::Error)<<"[ORIENT_IKF] imu_samples_period has to be a positive value!"<<RTT::endlog();
	return false;
    }
    if(config.correction_frequency <= 0.0)
    {
	RTT::log(RTT::Error)<<"[ORIENT_IKF] correction_frequency has to be a positive value!"<<RTT::endlog();
	return false;
    }
    if(accnoise.bandwidth <= 0.0)
    {
	RTT::log(RTT::Error)<<"[ORIENT_IKF] accnoise.bandwidth has to be a positive value!"<<RTT::endlog();
	return false;
    }
    if(gyronoise.bandwidth <= 0.0)
    {
	RTT::log(RTT::Error)<<"[ORIENT_IKF] gyronoise.bandwidth has to be a positive value!"<<RTT::endlog();
	return false;
    }
    if(config.use_inclinometers && incnoise.bandwidth <= 0.0)
    {
	RTT::log(RTT::Error)<<"[ORIENT_IKF] incnoise.bandwidth has to be a positive value!"<<RTT::endlog();
	return false;
    }

    /** Calculate the sampling frequency **/
    sampling_frequency = 1.0/_imu_samples_period.value();
    
    /********************************/
    /** Configuration frequencies  **/
    /********************************/
    if (config.correction_frequency > sampling_frequency)
    {
        config.correction_frequency = sampling_frequency;
        RTT::log(RTT::Warning)<<"[ORIENT_IKF] Set  correction frequency to sampling frequency. It cannot be higher that it!!"<<RTT::endlog();
    }

    /******************************/
    /** Correction configuration **/
    /******************************/
    correction_numbers = ceil(sampling_frequency/config.correction_frequency);
    correctionAcc.setZero(); correctionInc.setZero();

    /*************************/
    /** Noise configuration **/
    /*************************/
    if (config.correction_frequency > accnoise.bandwidth)
        sqrtdelta_t = sqrt(1.0/accnoise.bandwidth); /** Noise depends on frequency bandwidth **/
    else
        sqrtdelta_t = sqrt(1.0/config.correction_frequency); /** Noise depends on frequency bandwidth **/

    Ra = Eigen::Matrix3d::Zero();
    Ra(0,0) = accnoise.resolution[0] + pow(accnoise.randomwalk[0]/sqrtdelta_t,2);
    Ra(1,1) = accnoise.resolution[1] + pow(accnoise.randomwalk[1]/sqrtdelta_t,2);
    Ra(2,2) = accnoise.resolution[2] + pow(accnoise.randomwalk[2]/sqrtdelta_t,2);

    sqrtdelta_t = sqrt(1.0/accnoise.bandwidth); /** Noise depends on frequency bandwidth **/
    acceleration_out.cov_acceleration = Eigen::Matrix3d::Zero(); // this is the noise on the acceleration output which occurs at every sample
    acceleration_out.cov_acceleration(0,0) = accnoise.resolution[0] + pow(accnoise.randomwalk[0]/sqrtdelta_t,2);
    acceleration_out.cov_acceleration(1,1) = accnoise.resolution[1] + pow(accnoise.randomwalk[1]/sqrtdelta_t,2);
    acceleration_out.cov_acceleration(2,2) = accnoise.resolution[2] + pow(accnoise.randomwalk[2]/sqrtdelta_t,2);

    sqrtdelta_t = sqrt(1.0/gyronoise.bandwidth); /** Noise depends on frequency bandwidth **/

    Rg = Eigen::Matrix3d::Zero();
    Rg(0,0) = pow(gyronoise.randomwalk[0]/sqrtdelta_t,2);
    Rg(1,1) = pow(gyronoise.randomwalk[1]/sqrtdelta_t,2);
    Rg(2,2) = pow(gyronoise.randomwalk[2]/sqrtdelta_t,2);

    Ri = Eigen::Matrix3d::Zero();
    if(config.use_inclinometers)
    {
	if (config.correction_frequency > incnoise.bandwidth)
	    sqrtdelta_t = sqrt(1.0/incnoise.bandwidth); /** Noise depends on frequency bandwidth **/
	else
	    sqrtdelta_t = sqrt(1.0/config.correction_frequency); /** Noise depends on frequency bandwidth **/
	    
	Ri(0,0) = incnoise.resolution[0] + pow(incnoise.randomwalk[0]/sqrtdelta_t,2);
	Ri(1,1) = incnoise.resolution[1] + pow(incnoise.randomwalk[1]/sqrtdelta_t,2);
	Ri(2,2) = incnoise.resolution[2] + pow(incnoise.randomwalk[2]/sqrtdelta_t,2);
    }

    /** It does not have magnetometers **/
    Rm = Eigen::Matrix3d::Zero();

    /** Noise for error in gyros bias instability **/
    Qbg.setZero();
    Qbg(0,0) = pow(gyronoise.biasinstability[0],2);
    Qbg(1,1) = pow(gyronoise.biasinstability[1],2);
    Qbg(2,2) = pow(gyronoise.biasinstability[2],2);

    /** Noise for error in accelerometers bias instability **/
    Qba.setZero();
    Qba(0,0) = pow(accnoise.biasinstability[0],2);
    Qba(1,1) = pow(accnoise.biasinstability[1],2);
    Qba(2,2) = pow(accnoise.biasinstability[2],2);

    /** Noise for error in inclinometers bias instability **/
    Qbi.setZero();
    if(config.use_inclinometers)
    {
	Qbi(0,0) = pow(incnoise.biasinstability[0],2);
	Qbi(1,1) = pow(incnoise.biasinstability[1],2);
	Qbi(2,2) = pow(incnoise.biasinstability[2],2);
    }


    /** Initial error covariance **/
    P_0 = Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Zero();
    P_0.block <3, 3> (0,0) = 1.0e-06 * Eigen::Matrix3d::Identity();//Error quaternion
    P_0.block <3, 3> (3,3) = 1.0e-06 * Eigen::Matrix3d::Identity();//Gyros bias
    P_0.block <3, 3> (6,6) = 1.0e-06 * Eigen::Matrix3d::Identity();//Accelerometers bias
    P_0.block <3, 3> (9,9) = 1.0e-06 * Eigen::Matrix3d::Identity();//Inclinometers bias

    /** Theoretical Gravity **/
    double gravity = GRAVITY;
    if (location.latitude > 0.0 && location.latitude < base::Angle::deg2Rad(90.0))
        gravity = BaseEstimator::GravityModel (location.latitude, location.altitude);

    /** Initialize the filter, including the adaptive part **/
    ikf_filter.Init(P_0, Ra, Rg, Rm, Ri, Qbg, Qba, Qbi, gravity, location.dip_angle,
            adaptiveconfigAcc.M1, adaptiveconfigAcc.M2, adaptiveconfigAcc.gamma,
            adaptiveconfigInc.M1, adaptiveconfigInc.M2, adaptiveconfigInc.gamma);
    
    ikf_filter.setInitBias(gyronoise.biasoffset, accnoise.biasoffset, incnoise.biasoffset);

    /** Alignment configuration **/
    initial_alignment.acc.setZero();
    initial_alignment.gyro.setZero();
    initial_alignment.mag.setZero();
    initial_samples = 0;

    /** Set the samples count to Zero **/
    initial_samples = 0;
    initial_alignment_ts.fromMicroseconds(0);

    /** Set the correction index **/
    correction_idx = 0;

    /** Initial attitude **/
    init_attitude = false;

    max_time_delta = 0.1;

    prev_ts.microseconds = 0;

    /** Task states **/
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;

    /** Output variable **/
    orientation_out.invalidate();
    orientation_out.sourceFrame = config.source_frame_name;
    orientation_out.targetFrame = config.target_frame_name;
    orientation_out.orientation.setIdentity();
    acc_body = Eigen::Vector3d::Ones() * base::NaN<double>();

    prev_orientation_out = orientation_out;

    #ifdef DEBUG_PRINTS
    std::cout<< "Sampling frequency: "<<sampling_frequency<<"\n";
    std::cout<< "Correction frequency: "<<config.correction_frequency<<"\n";
    std::cout<< "Correction numbers: "<<correction_numbers<<"\n";
    std::cout<< "Rg\n"<<Rg<<"\n";
    std::cout<< "Ra\n"<<Ra<<"\n";
    std::cout<< "Rm\n"<<Rm<<"\n";
    std::cout<< "Ri\n"<<Ri<<"\n";
    std::cout<< "P_0\n"<<P_0<<"\n";
    std::cout<< "Qbg\n"<<Qbg<<"\n";
    std::cout<< "Qba\n"<<Qba<<"\n";
    std::cout<< "Qbi\n"<<Qbi<<"\n";
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
    this->writeOutput(ikf_filter);

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
	/** Set attitude to initial heading **/
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
                    euler.x() = 0.0; // remove rotation around the z-axis

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

                    /** Determine the initial heading  **/
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
                        {
                            if (transformed_meangyro.x() == 0.0)
                                euler[0] = (base::Angle::fromDeg(90.0) - base::Angle::fromRad(atan2(transformed_meangyro.x(), transformed_meangyro.y()))).getRad();
                            else
                                euler[0] = base::Angle::fromRad(atan2(transformed_meangyro.y(), transformed_meangyro.x())).getRad();
                        }
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

		    if(config.initial_estimate_bias_offset)
		    {
			/** Compute the Initial Bias **/
			Eigen::Vector3d gyro_bias = initial_alignment.gyro;
			if(config.substract_earth_rotation)
			    BaseEstimator::SubtractEarthRotation(gyro_bias, initial_attitude.inverse(), location.latitude);

			Eigen::Vector3d acc_bias = initial_alignment.acc - initial_attitude.inverse() * ikf_filter.getGravity();

			ikf_filter.setInitBias(gyro_bias, acc_bias, Eigen::Vector3d::Zero());
		    }

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
        orientation_out.orientation = initial_attitude;
        prev_orientation_out.orientation = initial_attitude;
        init_attitude = true;

        #ifdef DEBUG_PRINTS
	base::Vector3d euler = base::getEuler(initial_attitude);
        std::cout<< "******** Initial Attitude  *******"<<"\n";
        std::cout<< "Init Roll: "<<base::Angle::rad2Deg(euler[2])<<" Init Pitch: "<<base::Angle::rad2Deg(euler[1])<<" Init Yaw: "<<base::Angle::rad2Deg(euler[0])<<"\n";
        #endif
    }
}


void IKF::writeOutput(IKFFilter & filter)
{
    if (init_attitude && !prev_ts.isNull())
    {
        orientation_out.time = prev_ts;
        orientation_out.orientation = filter.getAttitude();
        orientation_out.cov_orientation = filter.getCovariance().block<3, 3>(0,0);
        Eigen::AngleAxisd deltaAngleaxis(prev_orientation_out.orientation.inverse() * orientation_out.orientation);
	double delta_time = (orientation_out.time - prev_orientation_out.time).toSeconds();
	if(delta_time > 0.0)
	    orientation_out.angular_velocity = (deltaAngleaxis.angle() * deltaAngleaxis.axis()) / delta_time;
	else
	    orientation_out.angular_velocity = base::Vector3d::Zero();
        _orientation_samples_out.write(orientation_out);

        if(base::isnotnan(acc_body))
        {
            acceleration_out.time = prev_ts;
    //        acceleration_out.acceleration = filter.getAttitude().toRotationMatrix()*(acc_body - filter.getAccBias() - filter.getGravityinBody()); // world frame, since RigidBodyAcceleration is defined this way
            acceleration_out.acceleration = acc_body - filter.getAccBias() - filter.getGravityinBody(); // body frame
            _acceleration_samples_out.write(acceleration_out);
        }
    }

    prev_orientation_out = orientation_out;
}

bool IKF::resetHeading(double heading)
{
    base::Vector3d euler = base::getEuler(ikf_filter.getAttitude());
    Eigen::Quaterniond corrected_attitide = Eigen::AngleAxisd(base::Angle::normalizeRad(heading), Eigen::Vector3d::UnitZ()) *
                                            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX());
    return ikf_filter.setAttitude(corrected_attitide);
}
