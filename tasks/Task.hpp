/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */
/**\file Task.hpp
 * Header function file and defines of the Task component.
 */

#ifndef ORIENTATION_ESTIMATOR_TASK_TASK_HPP
#define ORIENTATION_ESTIMATOR_TASK_TASK_HPP

#include <quater_ikf/ikf.h> /**< IKF header file*/
#include "orientation_estimator/TaskBase.hpp"

namespace orientation_estimator {
  
    /** General defines **/
    #ifndef OK
    #define OK	0  /**< Integer value in order to return when everything is all right. */
    #endif
    #ifndef ERROR
    #define ERROR -1  /**< Integer value in order to return when an error occured. */
    #endif
    
    #ifndef QUATERSIZE
    #define QUATERSIZE 4 /**< Number of parameters of a quaternion **/
    #endif
    
    /** Sensors constant parameters **/
    #ifndef NUMAXIS
    #define NUMAXIS 3 /**< Number of axis sensed by the sensors **/
    #endif
    
    /** FOG DSP 3000 defines **/
    #define FOGBIAS  0.00 /** FOG Initial bias offset **/
    #define FOGRW 1.32e-05 /** FOG Ramdom walk (white noise) **/
    #define FOGRRW 1.90e-08 /** FOG Rate Ramdom walk (white noise in angular acc) **/
    
    /** Xsens MTi defines **/
    #define XSENSBIASGYROX 6.911825e-03 /** Xsens gyro X Initial bias offset **/
    #define XSENSBIASGYROY 1.202073e-01 /** Xsens gyro Y Initial bias offset **/
    #define XSENSBIASGYROZ -1.881326e-06 /** Xsens gyro Z Initial bias offset **/
    
    #define XSENSRWGYROX 0.0006898864 /** Xsens random walk gyro X **/
    #define XSENSRWGYROY 0.0007219069 /** Xsens random walk gyro Y **/
    #define XSENSRWGYROZ 0.0005708627 /** Xsens random walk gyro Z **/
    #define XSENSRWGYROS (XSENSRWGYROX+XSENSRWGYROY+XSENSRWGYROZ)/3/**<Mean random walk for the gyros in rad/sqrt(s) **/
    
    #define XSENSRWACCX 0.0010310305 /** Xsens random walk acc X **/
    #define XSENSRWACCY 0.001318912 /** Xsens random walk acc Y **/
    #define XSENSRWACCZ 0.0011245297 /** Xsens random walk acc Z **/
    #define XSENSRWACCS (XSENSRWACCX+XSENSRWACCY+XSENSRWACCZ)/3
    
    #define XSENSRWMAGX 0.001590766 /** Xsens random walk mag X **/
    #define XSENSRWMAGY 0.00250713 /** Xsens random walk mag Y **/
    #define XSENSRWMAGZ 0.001061307 /** Xsens random walk mag Z **/
    #define XSENSRWMAGS (XSENSRWACCX+XSENSRWACCY+XSENSRWACCZ)/3
    
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

	double xsens_time; /**< Delta time coming for Xsens values */
	double fog_time; /**< Delta time coming for FOG values */
	bool flag_xsens_time, flag_fog_time, init_attitude; /** Control flags */
	Eigen::Matrix <double,NUMAXIS,1> *xsens_gyros; /**< Gyroscopes from Xsens */
	Eigen::Matrix <double,NUMAXIS,1> *xsens_acc; /**< Acceleremeters from Xsens */
	Eigen::Matrix <double,NUMAXIS,1> *xsens_mag; /**< Magnetometers from Xsens */
	Eigen::Matrix <double,NUMAXIS,1> *fog_gyros; /**< Angular velocity for the FOG */
	Eigen::Quaternion <double> *head_q; /**< Quaternion for the yaw (heading) */
	filter::ikf *myikf; /**< The adaptive extended kalman filter */
	base::samples::RigidBodyState *rbs_b_g; /**< Output RigidBody State containin the orientation and angular velocity of the body */
	Eigen::Matrix <double, NUMAXIS, 1> *oldeuler; /**< Euler angles for the velocity stimation */
	
        virtual void fog_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &fog_samples_sample);
        virtual void xsens_orientationCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xsens_orientation_sample);
        virtual void xsens_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &xsens_samples_sample);
	
	void PropagateHeadingQuaternion(Eigen::Quaternion <double> *quat, Eigen::Matrix<double, NUMAXIS , 1>* angvelo, double dt);
	
    public:
        Task(std::string const& name = "orientation_estimator::Task");

	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
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
         * Call recovered() to go back in the Runtime state.
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

