#include <ros/ros.h>
#include <thread>
#include "camera_libuvc.hpp"
#include "imu_node.hpp"
using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vr_helmet_driver_node");
    ros::NodeHandle nh("~");

    ImuNode imu_node(nh);
    CameraNode cam_node(nh,&imu_node);
    std::thread cam_thread(&CameraNode::image_retrieve,cam_node);

    std::thread imu_thread(&ImuNode::imu_read,imu_node);

    imu_thread.join();
    cam_thread.join();
    return 0;
}
