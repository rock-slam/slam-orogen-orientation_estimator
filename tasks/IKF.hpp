/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ORIENTATION_ESTIMATOR_IKF_TASK_HPP
#define ORIENTATION_ESTIMATOR_IKF_TASK_HPP

#include "orientation_estimator/IKFBase.hpp"
#include "orientation_estimator/BaseEstimator.hpp"
#include <quater_ikf/Ikf.hpp>

#define DEBUG_PRINTS 1

namespace orientation_estimator {

    typedef filter::Ikf<double, true, true> IKFFilter;

    /** WGS-84 ellipsoid constants (Nominal Gravity Model and Earth angular velocity) **/
    static const int Re = 6378137; /** Equatorial radius in meters **/
    static const int Rp = 6378137; /** Polar radius in meters **/
    static const double ECC = 0.0818191908426; /** First eccentricity **/
    static const double GRAVITY = 9.79766542; /** Mean value of gravity value in m/s^2 according to WGS-84 **/
    static const double GRAVITY_SI = 9.80665; /** Mean value of gravity value in m/s^2 according to SI standard **/
    static const double GWGS0 = 9.7803267714; /** Gravity value at the equator in m/s^2 **/
    static const double GWGS1 = 0.00193185138639; /** Gravity formula constant **/
    static const double EARTHW = 7.292115e-05; /** Earth angular velocity in rad/s **/

    enum CONST
    {
        IKFSTATEVECTORSIZE = IKFFilter::IKFSTATEVECTORSIZE,
    };

    static const double GRAVITY_MARGIN = 0.3; /** Accepted error for the gravity value in [m/s^2] **/

    /*! \class IKF 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * timestamped state vector holding orientation and angular velocity with respect to the Earth fixed frame (geographic frame).
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','orientation_estimator::IKF')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class IKF : public IKFBase
    {
	friend class IKFBase;
    protected:
	
        /******************************/
        /*** Control Flow Variables ***/
        /******************************/

        /** Initial Attitude **/
        bool init_attitude;

        /** Index for initializing attitude **/
    	unsigned int initial_samples;
        base::Time initial_alignment_ts;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /** Filter configuration values **/
        FilterConfiguration config;

        /** Accelerometers noise parameters **/
        InertialNoiseParameters accnoise;

        /** Gyroscopes noise parameters **/
        InertialNoiseParameters gyronoise;

        /** Inclinometers noise parameters **/
        InertialNoiseParameters incnoise;

        /** Adaptive Measurement Configuration **/
        AdaptiveAttitudeConfig adaptiveconfigAcc;
        AdaptiveAttitudeConfig adaptiveconfigInc;

        /** Location configuration variables **/
        LocationConfiguration location;

        /**************************/
        /*** Internal Variables ***/
        /**************************/

        base::Time prev_ts;

        unsigned int correction_numbers, correction_idx;

        double sampling_frequency;

        /** Correction  Variables**/
        Eigen::Vector3d correctionAcc, correctionInc;

        Eigen::Vector3d acc_body; // saved acceleration for output

        /** Accumulated measurement for attitude calculation */
        base::samples::IMUSensors initial_alignment;

        IKFFilter ikf_filter; /** The adaptive Indirect Kalman filter */
	
        double delta_t, max_time_delta;

        /** Task states **/
        States last_state;
        States new_state;

        /***************************/
        /** Output port variables **/
        /***************************/

        base::samples::RigidBodyState orientation_out;
        base::samples::RigidBodyState prev_orientation_out;

        base::samples::RigidBodyAcceleration acceleration_out;

	
    protected:

        virtual void imu_samplesTransformerCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample);
	
        void writeOutput(IKFFilter & filter);
	
        void initialAlignment(const base::Time &ts, const base::samples::IMUSensors &imu_sample);

        virtual bool resetHeading(double heading);

    public:
        /** TaskContext constructor for IKF
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        IKF(std::string const& name = "orientation_estimator::IKF");

        /** TaskContext constructor for IKF 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        IKF(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of IKF
         */
        ~IKF();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

