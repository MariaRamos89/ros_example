#include "slam.hpp"

slam::slam() 
: callab_( std::bind(&slam::callback, this, std::placeholders::_1),
           {"10.130.3.21", "9001", "test_token", "test"},
           "raspberry_map", "icp.ini", noos::object::laser())
{}

void slam::read_laser(const sensor_msgs::LaserScan::ConstPtr & scan)
{
    assert(scan);
    noos::object::laser obs;
    if (scan) { 
        int count = scan->scan_time / scan->time_increment;
        //auto now = std::chrono::system_clock::now();
        obs.timestamp = 0; //now.time_since_epoch().count();
        obs.ranges.resize(count);
        obs.intensities.resize(count);
        obs.right_to_left = false;
        obs.aperture = 2 * M_PIf;
        obs.max_range = 6.0;
        obs.std_error = 0.010f;
        obs.pose3d = noos::object::pose<float>();

        for (int i = 0; i < count; i++) {
            obs.ranges[i] = scan->ranges[i];
            obs.intensities[i] = (int)scan->intensities[i];
            std::cout << obs.ranges[i] << std::endl;
        }
        process_data(obs);
    }
    else {
        std::cout << "No laser data" << std::endl;
    }
}

void slam::process_data(noos::object::laser & obs)
{
    std::lock_guard<std::mutex> lock(mutex__);
    callab_.object = noos::cloud::icp_slam("raspberry_map", "icp.ini", obs); //map, config, laser
    callab_.send();
}

void slam::callback(noos::object::pose<float> pose3d)
{
    std::cout << pose3d;
}
