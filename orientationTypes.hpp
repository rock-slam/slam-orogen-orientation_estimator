#ifndef orientation_TYPES_HPP
#define orientation_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <base/Time.hpp>
#include <base/Eigen.hpp>

namespace orientation_estimator
{

    /** Processing Configuration **/
    struct Configuration
    {
        std::string source_frame_name; //Output Frame name. Transformation: source -> target

        std::string target_frame_name; //Output Frame name. Transformation: source -> target

        bool use_samples_as_theoretical_gravity;//Inclinometers are more stable than accelerometers at initial time.
                                                    //They cloud be use as theoretical local gravity value instead of using
                                                    //some models as WGS-84 ellipsoid Earth.
                                                    //It will use inclinometers in case use_inclinometers_leveling is true
                                                    //and accelerometers otherwise.
        bool use_inclinometers_leveling;//Some IMU provide inclinometers as fast and more accurate solution for initial leveling.
                                //Set True or False to use inclinometers values or not.
                                //Note: Check if the IMU has inclinometers information.

        bool use_magnetometers_heading;// Some IMUS provides Magnetic information.
                                    // Set to true or false in case you want to correct heading with magnetometers

        unsigned int init_leveling_samples;//Samples to compute the initial leveling of the imu in order to find the gravity vector.
                                            //Set to zero in case zero attitude is desired as initial orientation from an arbitrary frame.
    };

    //Data type for the Inertial measurement characteristic
    struct InertialNoiseParameters
    {
        /********************************/
        /** Inertial Sensor Properties **/
        /********************************/

        double bandwidth; //Inertial sensors bandwidth in Hertz.
        //This is characteristic of the sensor and should be equal
        //or smaller than the sampling rate.

        /** Gyroscope Noise **/
        base::Vector3d gbiasoff;//bias offset in static regimen for the Gyroscopes
        base::Vector3d gyrorw;//angle random walk for gyroscopes (rad/sqrt(s))
        base::Vector3d gyrorrw;//rate random walk for gyroscopes (rad/s/sqrt(s))
        base::Vector3d gbiasins; //gyros bias instability (rad/s)

        /** Accelerometers Noise **/
        base::Vector3d abiasoff;//bias offset in static regimen for the Accelerometers
        base::Vector3d accrw;//velocity random walk for accelerometers (m/s/sqrt(s))
        base::Vector3d accrrw;//acceleration random walk for accelerometers (m/s^2/sqrt(s))
        base::Vector3d abiasins;//accelerometers bias instability (m/s^2)
        base::Vector3d accresolut;//minimum accelerometers resolution (m/s^2)

        /** Magnetometers Noise **/
        base::Vector3d magrw; //random walk for magnetometers

        /** Inclinometers Noise **/
        base::Vector3d incrw; //random walk for inclinometers (m/s/sqrt(s))
        base::Vector3d incresolut;//minimum accelerometers resolution (m/s^2)

    };

    //Data type to know the location
    struct LocationConfiguration
    {
        double latitude;//Latitude in radians
        double longitude;//Longitude in radians
        double altitude;//Altitude in meters
        double magnetic_declination;//Declination in radians
        int magnetic_declination_mode;//The declination is positive when the magnetic north is east of true north
                                    //1 is EAST, which means positive declination. 2 is WEST, which means negative declination.
        double dip_angle;//Dip angle in radians
    };

    /** Adaptive Measurement Configuration. Variables for the attitude estimation inside the algorithm **/
    struct AdaptiveAttitudeConfig
    {
        unsigned int M1; /** Parameter for adaptive algorithm (to estimate Uk with is not directly observable) */
        unsigned int M2; /** Parameter for adaptive algorithm (to prevent false entering in no-external acc mode) */
        double gamma; /** Parameter for adaptive algorithm. Only entering when Qstart (adaptive cov. matrix) is greater than RHR'+Ra */

        void reset()
        {
            M1 = 0;
            M2 = 0;
            gamma = 0.0;
            return;
        }

    };

}

#endif

