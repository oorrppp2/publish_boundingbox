
// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_boundingbox_node");

    //! ROS node handle.
    ros::NodeHandle nodeHandle_;
//    ros::NodeHandle nh;
//    nodeHandle_(nh);

    ros::Publisher boundingBoxesPublisher_;
    darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;

    std::string boundingBoxesTopicName;
    int boundingBoxesQueueSize;
    bool boundingBoxesLatch;

    nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName,
                      std::string("/darknet_ros/bounding_boxes"));
    nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
    nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);

    boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(
        boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
    while(ros::ok()) {
//        printf("sending bounding box\n");

        darknet_ros_msgs::BoundingBox boundingBox;
        int xmin = 0;
        int ymin = 0;
        int xmax = 100;
        int ymax = 100;

        boundingBox.Class = "apple";
        boundingBox.probability = 0.9;
        boundingBox.xmin = xmin;
        boundingBox.ymin = ymin;
        boundingBox.xmax = xmax;
        boundingBox.ymax = ymax;
        boundingBoxesResults_.bounding_boxes.push_back(boundingBox);

        xmin = 200;
        ymin = 100;
        xmax = 350;
        ymax = 300;

        boundingBox.Class = "cup";
        boundingBox.probability = 0.9;
        boundingBox.xmin = xmin;
        boundingBox.ymin = ymin;
        boundingBox.xmax = xmax;
        boundingBox.ymax = ymax;
        boundingBoxesResults_.bounding_boxes.push_back(boundingBox);

        boundingBoxesResults_.header.stamp = ros::Time::now();
        boundingBoxesResults_.header.frame_id = "detection";
//        boundingBoxesResults_.image_header = headerBuff_[(buffIndex_ + 1) % 3];
        boundingBoxesPublisher_.publish(boundingBoxesResults_);

        boundingBoxesResults_.bounding_boxes.clear();
    }
    return 0;
}
