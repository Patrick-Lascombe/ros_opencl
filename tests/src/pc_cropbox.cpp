#include <ros/ros.h>
#include <ros/package.h>
#include "ros_opencl/ros_opencl.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace std;

ros::Publisher pub;
ros_opencl::ROS_OpenCL roscl;
std::vector<float> minmax_;

void cloudCallback (const sensor_msgs::PointCloud2& msg){
    std::vector<float> vx, vy, vz;

    sensor_msgs::PointCloud2 msg_2(msg);
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg_2, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg_2, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg_2, "z");

    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        vx.push_back(*iter_x);
        vy.push_back(*iter_y);
        vz.push_back(*iter_z);
    }

    minmax_.push_back(vx[0]);
    minmax_.push_back(vx[1]);
    minmax_.push_back(vx[2]);
    minmax_.push_back(vx[0]);
    minmax_.push_back(vx[1]);
    minmax_.push_back(vx[2]);
    for(int i=0; i < vx.size() ; i++) {
        if(vx[i] < minmax_[0]){
            minmax_[0] = vx[i];
        } else if (vx[i] > minmax_[3]) {
            minmax_[3] = vx[i];
        }

        if(vy[i] < minmax_[1]){
            minmax_[1] = vy[i];
        } else if (vy[i] > minmax_[4]) {
            minmax_[4] = vy[i];
        }

        if(vz[i] < minmax_[2]){
            minmax_[2] = vz[i];
        } else if (vz[i] > minmax_[5]) {
            minmax_[5] = vz[i];
        }
    }

    for(int i = 0; i < minmax_.size(); i++) {
        minmax_[i] = minmax_[i] / 2;
    }

    std::vector<int> isIn;
    roscl.process(vx, vy, vz, minmax_, &isIn);
    std::vector<float> res_x, res_y, res_z;

    sensor_msgs::PointCloud2Iterator<float> iter_x_output(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_output(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_output(output, "z");

    for (int i = 0; i < res_x.size(); i++) {
        if(isIn[i] == 1) {
            *iter_x_output = res_x[i];
            *iter_y_output = res_y[i];
            *iter_z_output = res_z[i];
        }
    }

    pub.publish(output);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "ros_opencl_pointcloud_test1");
    ros::NodeHandle nh;
    string kernel_filename;
    string cloud_topic;
    string result_topic;

    nh.param("ros_opencl_pointcloud_test1/kernel_filename", kernel_filename, string("test_kernels.cl"));
    nh.param("ros_opencl_pointcloud_test1/cloud_topic", cloud_topic, string("/camera/depth/color/pointsFiltered"));
    nh.param("ros_opencl_pointcloud_test1/result_topic", result_topic, string("ros_opencl_pointcloud_test1/result"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = new ros_opencl::ROS_OpenCL(full_kernel_path, "cropPointCloud");

    pub = nh.advertise<sensor_msgs::PointCloud2>(result_topic, 1);
    ros::Subscriber s = nh.subscribe (cloud_topic, 1, cloudCallback);

    while(ros::ok()){
        ros::spin();
    }
}
