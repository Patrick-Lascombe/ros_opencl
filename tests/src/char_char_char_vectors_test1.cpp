#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include "ros_opencl/ros_opencl.hpp"

using namespace std;

ros::Publisher pub;
ros_opencl::ROS_OpenCL roscl;
sensor_msgs::Image prev_msg;
sensor_msgs::Image prev_msg2;

void imageCallback (const sensor_msgs::Image& msg){
    if (prev_msg2.data.size() > 0){
        sensor_msgs::Image im = sensor_msgs::Image(msg);
        std::vector<char> v;
        std::vector<char> v2;
        std::vector<char> v3;
        for (unsigned i = 0; i<msg.data.size(); i++){
            v.push_back(msg.data[i]);
            v2.push_back(prev_msg.data[i]);
            v3.push_back(prev_msg.data[i]);
        }
        v = roscl.process(v, v2, v3);
        im.data.clear();
        for (unsigned i = 0; i<msg.data.size(); i++){
            im.data.push_back(v[i]);
        }
        pub.publish(im);
    }
    prev_msg2 = prev_msg;
    prev_msg = msg;
}

int main (int argc, char** argv){
    ros::init (argc, argv, "ros_opencl_char_char_char_vectors_test1");
    ros::NodeHandle nh;
    string kernel_filename;
    string image_topic;
    string result_topic;

    nh.param("ros_opencl_char_char_char_vectors_test1/kernel_filename", kernel_filename, string("test_kernels.cl"));
    nh.param("ros_opencl_char_char_char_vectors_test1/image_topic", image_topic, string("/usb_cam/image_raw"));
    nh.param("ros_opencl_char_char_char_vectors_test1/result_topic", result_topic, string("ros_opencl_char_char_vectors_test1/result"));

    string full_kernel_path = ros::package::getPath("ros_opencl") + "/tests/kernels/" + kernel_filename;

    roscl = new ros_opencl::ROS_OpenCL(full_kernel_path, "frameDiff3");

    pub = nh.advertise<sensor_msgs::Image>(result_topic, 1);
    ros::Subscriber s = nh.subscribe (image_topic, 1, imageCallback);

    while(ros::ok()){
        ros::spin();
    }
}