#include <ros/ros.h>
#include <ros/package.h>
#include "ros_opencl/ros_opencl.hpp"
#include <sensor_msgs/PointCloud2.h>

using namespace std;

ros::Publisher pub_normal;
ros_opencl::ROS_OpenCL roscl;

void cloudNormalCallback(const sensor_msgs::PointCloud2& msg) {
    ros::Time t1 = ros::Time::now();
    sensor_msgs::PointCloud2 msg_out;
    msg_out.header = msg.header;
    for (auto it = msg.data.begin() ; it != msg.data.end() ; it++) {
        msg_out.data.push_back(-*it);
    }
    pub_normal.publish(msg_out);
    ros::Time t2 = ros::Time::now();
    ROS_INFO_STREAM("Normal callback : " << (t2-t1).toSec());
}


int main (int argc, char** argv){
    ros::init (argc, argv, "ros_pointcloud_revert_cpu");
    ros::NodeHandle nh;
    string kernel_filename;
    string cloud_topic;
    string result_topic_opencl, result_topic_normal;

    nh.param("ros_opencl_pointcloud_test1/kernel_filename", kernel_filename, string("test_kernels.cl"));
    nh.param("ros_opencl_pointcloud_test1/cloud_topic", cloud_topic, string("/camera/depth_registered/points"));
    nh.param("ros_opencl_pointcloud_test1/result_topic", result_topic_opencl, string("/ros_opencl/output_cloud"));
    nh.param("ros_opencl_pointcloud_test1/result_normal", result_topic_normal, string("/ros_opencl/output_cloud_normal"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = new ros_opencl::ROS_OpenCL(full_kernel_path, "invertPointcloud");

    pub_normal = nh.advertise<sensor_msgs::PointCloud2>(result_topic_normal, 1);
    ros::Subscriber s2 = nh.subscribe (cloud_topic, 1, cloudNormalCallback);

    while(ros::ok()){
        ros::spin();
    }
}
