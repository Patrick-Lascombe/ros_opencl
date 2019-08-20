#include <ros/ros.h>
#include <ros/package.h>
#include "ros_opencl/ros_opencl.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;

ros::Publisher pub;
ros_opencl::ROS_OpenCL roscl;
std::vector<float> minmax;
void cloudCallback (const sensor_msgs::PointCloud2& msg){
    std::vector<float> vx, vy, vz;
    std::vector<float> res_x, res_y, res_z;

    sensor_msgs::PointCloud2 msg_2(msg);

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg_2, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg_2, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg_2, "z");

    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        vx.push_back(*iter_x);
        vy.push_back(*iter_y);
        vz.push_back(*iter_z);
    }



//    roscl.process(x, y, z, minmax, res_x, res_y, res_z);


}

int main (int argc, char** argv){
    ros::init (argc, argv, "ros_opencl_pointcloud_test1");
    ros::NodeHandle nh;
    string kernel_filename;
    string cloud_topic;
    string result_topic;

    nh.param("ros_opencl_pointcloud_test1/kernel_filename", kernel_filename, string("test_kernels.cl"));
    nh.param("ros_opencl_pointcloud_test1/cloud_topic", cloud_topic, string("/zed/point_cloud/cloud_registered"));
    nh.param("ros_opencl_pointcloud_test1/result_topic", result_topic, string("ros_opencl_pointcloud_test1/result"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = new ros_opencl::ROS_OpenCL(full_kernel_path, "cpCropBox");

    pub = nh.advertise<sensor_msgs::PointCloud2>(result_topic, 1);
    ros::Subscriber s = nh.subscribe (cloud_topic, 1, cloudCallback);

    while(ros::ok()){
        ros::spin();
    }
}
