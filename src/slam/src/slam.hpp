#ifndef SLAM_HPP
#define SLAM_HPP
#include "includes.ihh"

#define M_PIf 3.14159265358979f
/**
 * @brief read the laser data
 * @class slam
 */
class slam
{
public:
    /// @brief constructor
    slam(noos::cloud::platform platf);

    /// @brief read lasers
    void read_laser(const sensor_msgs::LaserScan::ConstPtr & scan);

    /// @brief process data
    void process_data(noos::object::laser & obs);

private:
    //callback
    void callback(noos::object::pose<float> pose3d);

    //callable object
    noos::cloud::callable<
        noos::cloud::icp_slam, true> callab_;
    //
    std::mutex mutex__;

};

#endif
