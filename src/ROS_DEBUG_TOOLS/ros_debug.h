#ifndef ROS_DEBUG_DEBUG_H
#define ROS_DEBUG_DEBUG_H

#include <iostream>
#include <vector>

#include "ros/package.h"
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/PointCloud2.h"
#include <ros/node_handle.h>

#define INVALID_STRUCT "invalid struct"

using visualization_msgs::PointCloud2;
using std::vector;
using Eigen::Vector2f;

/*
 * This is a single header that contains some of the debug code I use for my
 * lab work. It is designed to be as easy to use as possible. Speed is not really
 * a consideration when debugging so I don't worry about that when writing this.
 */

/*
 * PubPoints(topic, points)
 * topic - topic to publish on.
 * points - points to publish.
 */
struct ROSPub {
    std::string topic;
    ros::Publisher pub;
    PointCloud2 point_marker;

    ROSPub() {
      topic = INVALID_STRUCT;
    }

    ROSPub(std::string topic,
           ros::Publisher pub,
           PointCloud2 point_marker) :
           topic(topic),
           pub(pub),
           point_marker(point_marker) {}
};

static ros::NodeHandle n;
static vector<ROSPub> publishers;

void pointcloud_helpers::InitPointcloud(PointCloud2* point) {
  std::string arr[3] = {"x", "y", "z"};
  point->header.seq = 1;
  point->header.stamp = ros::Time::now();
  point->header.frame_id = "map";
  sensor_msgs::PointField field;
  int offset = 0;
  field.datatype = 7;
  field.count = 1;
  for (std::string type : arr) {
    field.offset = offset;
    field.name = type;
    point->fields.push_back(field);
    offset += 4;
  }
  point->height = 1;
  point->width = 0;
  point->is_bigendian = false;
  point->point_step = 12;
  point->row_step = 0;
  point->is_dense = true;
}


void pointcloud_helpers::PushBackBytes(float val,
                                       sensor_msgs::PointCloud2& ptr) {
  uint8_t *data_ptr = reinterpret_cast<uint8_t*>(&val);
  for (int i = 0; i < 4; i++) {
    ptr.data.push_back(data_ptr[i]);
  }
}

void
pointcloud_helpers::PublishPointcloud(const vector<Vector2f>& points,
                                      PointCloud2& point_cloud,
                                      Publisher& pub) {
  for (uint64_t i = 0; i < points.size(); i++) {
    Vector2f vec = points[i];
    PushBackBytes(vec[0], point_cloud);
    PushBackBytes(vec[1], point_cloud);
    PushBackBytes(0.0f, point_cloud);
  }
  point_cloud.width = points.size();
  pub.publish(point_cloud);
  point_cloud.width = 0;
  point_cloud.data.clear();
}

static void PubPoints(std::string topic, const vector<Vector2f>& pointcloud) {
  // Find ROS Publishing struct.
  ROSPub ros_struct;
  for (const ROSPub& pub_struct : publishers) {
    if (pub_struct.topic.compare(topic) == 0) {
      ros_struct = pub_struct;
      break;
    }
  }
  // If we didn't find publishing info.
  if (ros_struct.compare(INVALID_STRUCT) == 0) {
    ros::Publisher pub = n.advertise<PointCloud2>(topic, 10);
    PointCloud2 point_marker;
    InitPointcloud(&point_marker);
    ROSPub rp(topic, pub, point_marker);
    publishers.push_back(rp);
    ros_struct = rp;
  }
  // Now just publish the points.
  PublishPointcloud(pointcloud, ros_struct.point_marker, ros_struct.pub);
}

static void WaitForUserInput() {
  char waiting_char;
  std::cin >>  waiting_char;
}

#endif
