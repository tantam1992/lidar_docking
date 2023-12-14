#include <ros/ros.h>
#include <laser_line_extraction/LineSegmentList.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <boost/array.hpp>
#include <vector>
#include <cmath>
#include "lidar_docking/pattern.h"

bool calAngle(double a, double b, double angle_ab, double detect_angle_tolerance) {
    double angle = fabs(a - b);
    if (angle < M_PI && angle > 1.7) {
        angle = angle - M_PI_2;
    } else if (angle > (M_PI_2 + M_PI)) {
        angle = angle - M_PI - M_PI_2;
    } else if (angle > M_PI && angle < (M_PI_2 + M_PI)) {
        angle = angle - M_PI;
    }
    if (fabs(angle_ab - angle) <= detect_angle_tolerance) {
        return true;
    } else {
        return false;
    }
}

void populateTF(double x, double y, double theta, std::string name) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0));
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), laser_frame_id, name));
}

void patternCallback(const laser_line_extraction::LineSegmentList::ConstPtr& msg) {
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> vectors = msg->line_segments;

    int lineNum = vectors.size();
    bool check_vec_size = true;

    if (lineNum < 2) {
        ROS_WARN("There aren't enough lines in the laser field!");
        check_vec_size = false;
    }

    if (check_vec_size) {
        for (int i = 0; i < lineNum; i++) {
            for (int j = i + 1; j < lineNum; j++) {
                if (calAngle(vectors[i].angle, vectors[j].angle, M_PI_2, detect_angle_tolerance)) {
                    // Calculate the position of the corner
                    double x = vectors[i].end[0];
                    double y = vectors[i].end[1];
                    double theta = vectors[i].angle + M_PI_2;

                    // Publish the TF transform for the corner
                    populateTF(x, y, theta, "corner");

                    return;  // Exit the callback after detecting the corner
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "corner_detection_node");
    ros::NodeHandle nh_;

    nh_.param<double>("detect_angle_tolerance", detect_angle_tolerance, 0.25);
    nh_.param<std::string>("laser_frame_id", laser_frame_id, "scan_front");

    ros::Subscriber line_sub_ = nh_.subscribe("line_segments", 10, patternCallback);
    ros::spin();

    return 0;
}