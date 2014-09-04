/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ORIENTATION_ESTIMATOR_BASEESTIMATOR_TASK_HPP
#define ORIENTATION_ESTIMATOR_BASEESTIMATOR_TASK_HPP

#include <math.h>
#include <Eigen/Geometry> /**< Eigen data type for Matrix, Quaternion, etc... */
#include "orientation_estimator/BaseEstimatorBase.hpp"

namespace orientation_estimator {

    /*! \class BaseEstimator 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 1 is EAST, which means positive declination. 2 is WEST, which means negative declination.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','orientation_estimator::BaseEstimator')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class BaseEstimator : public BaseEstimatorBase
    {
	friend class BaseEstimatorBase;

        /** Sensors constant parameters **/
        enum CONSTS {
          NUMAXIS = 3,
          QUATERSIZE = 4
        };

        enum DECLINATION_CONSTS {
          EAST = 1, /** EAST is 1 and means positive magnetic declination **/
          WEST = 2 /** WEST is 2 and means negative magnetic declination **/
        };

    protected:

	double fog_time, fog_dt; /**< Delta time coming for FOG values */
	bool flag_fog_time, init_attitude; /**< Control flags */
	Eigen::Quaternion <double> *head_q; /**< Quaternion for the yaw (heading) */
	Eigen::Matrix <double, NUMAXIS, 1> *euler;
	base::samples::RigidBodyState *rbs_b_g; /**< Output RigidBody State containin the orientation and angular velocity of the body */
	Eigen::Matrix <double, NUMAXIS, 1> *oldeuler; /**< Euler angles for the velocity stimation */
	Eigen::Matrix <double,NUMAXIS,1> fog_gyros; /**< Angular velocity for the FOG */ 
	
	
        virtual void fog_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &fog_samples_sample);
        virtual void imu_orientationCallback(const base::Time &ts, const ::base::samples::RigidBodyState &imu_orientation_sample);

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
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
	static void PropagateHeadingQuaternion(Eigen::Quaternion <double> *quat, Eigen::Matrix<double, NUMAXIS , 1>* angvelo, double dt);
	
	/**
	* @brief Substract the Earth rotation from the gyroscopes readout
	*
	* This function computes the substraction of the rotation of the Earth (EARTHW)
	* from the gyroscope values. This function uses quaternion of transformation from
	* the body to the geographic frame and the latitude in radians.
	*
	* @author Javier Hidalgo Carrio.
	*
	* @param[in, out] *u pointer to angular velocity
	* @param[in] *qb_g quaternion from body frame to geographic frame
	* @param[in] latitude location latitude angle in radians
	*
	* @return void
	*
	*/
	static void SubstractEarthRotation(Eigen::Matrix <double, NUMAXIS, 1> *u, Eigen::Quaternion <double> *qb_g, double latitude);
	
	
	/**
	* @brief This computes the theoretical gravity value according to the WGS-84 ellipsoid earth model.
	*
	* @author Javier Hidalgo Carrio.
	*
	* @param[in] latitude double the latitude value in radian
	* @param[in] altitude double with the altitude value in meters
	*
	* @return double. the theoretical value of the local gravity
	*
	*/
	static double GravityModel (double latitude, double altitude);
	
	
	/**
	* @brief Correct the magnetic declination of the North
	*
	* Magnetic North and geographic North (Ertah rotation axis)
	* are different depending on geograohic location according
	* to a Declination Map. The function correct this bias.
	* See: http://www.magnetic-declination.com for futher information
	* about the declination angle of your location.
	*
	* @author Javier Hidalgo Carrio.
	*
	* @param[in, out] *quat pointer to quaternion with the orientation 
	* @param[in] double magnetic declination angle in radians
	* @param[in] mode. EAST or WEST depending on the magnetic declination
	*
	* @return OK is everything all right. ERROR on other cases.
	*
	*/
	static bool CorrectMagneticDeclination (Eigen::Quaternion <double> *quat, double magnetic_declination,  int mode);
	
        /** TaskContext constructor for BaseEstimator
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        BaseEstimator(std::string const& name = "orientation_estimator::BaseEstimator");

        /** TaskContext constructor for BaseEstimator 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        BaseEstimator(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of BaseEstimator
         */
	~BaseEstimator();
	
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

