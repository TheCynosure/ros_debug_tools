#ifndef ROS_DEBUG_DEBUG_H
#define ROS_DEBUG_DEBUG_H

#include <iostream>
#include <vector>

#include "ros/package.h"
#include "eigen3/Eigen/Dense"
#include "sensor_msgs/PointCloud2.h"
#include <ros/node_handle.h>
#include "visualization_msgs/Marker.h"

#include "gui_helpers.h"

#define INVALID_STRUCT "invalid struct"

using sensor_msgs::PointCloud2;
using std::vector;
using Eigen::Vector2f;
using std::pair;

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
    visualization_msgs::Marker vis_marker;

    ROSPub() {
      topic = INVALID_STRUCT;
    }

    ROSPub(std::string topic,
           ros::Publisher pub,
           PointCloud2 point_marker) :
           topic(topic),
           pub(pub),
           point_marker(point_marker) {}

    ROSPub(std::string topic,
           ros::Publisher pub,
           visualization_msgs::Marker marker) :
            topic(topic),
            pub(pub),
            vis_marker(marker) {}
};

static ros::NodeHandle* n = nullptr;
static vector<ROSPub> publishers;

static RosDebugInit() {
  n = new ros::NodeHandle;
}

void InitPointcloud(PointCloud2* point) {
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


void PushBackBytes(float val,
                   sensor_msgs::PointCloud2& ptr) {
  uint8_t *data_ptr = reinterpret_cast<uint8_t*>(&val);
  for (int i = 0; i < 4; i++) {
    ptr.data.push_back(data_ptr[i]);
  }
}

void
PublishPointcloud(const vector<Vector2f>& points,
                                      PointCloud2& point_cloud,
                                      ros::Publisher& pub) {
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
  if (ros_struct.topic.compare(INVALID_STRUCT) == 0) {
    ros::Publisher pub = n->advertise<PointCloud2>(topic, 10);
    PointCloud2 point_marker;
    InitPointcloud(&point_marker);
    ROSPub rp(topic, pub, point_marker);
    publishers.push_back(rp);
    ros_struct = rp;
  }
  // Now just publish the points.
  PublishPointcloud(pointcloud, ros_struct.point_marker, ros_struct.pub);
}

static void PubLines(std::string topic, const vector<pair<Vector2f, Vector2f>>& lines) {
  ROSPub ros_struct;
  for (const ROSPub& pub_struct : publishers) {
    if (pub_struct.topic.compare(topic) == 0) {
      ros_struct = pub_struct;
      break;
    }
  }
  // If we didn't find publishing info.
  if (ros_struct.topic.compare(INVALID_STRUCT) == 0) {
    ros::Publisher pub = n->advertise<visualization_msgs::Marker>(topic, 10);
    visualization_msgs::Marker marker;
    gui_helpers::InitializeMarker(visualization_msgs::Marker::LINE_LIST, gui_helpers::Color4f::kWhite, 0.1, 0.1, 0.1, &marker);
    ROSPub rp(topic, pub, marker);
    publishers.push_back(rp);
    ros_struct = rp;
  }
  // Add all the lines to the msg
  gui_helpers::ClearMarker(&ros_struct.vis_marker);
  for (const pair<Vector2f, Vector2f> line : lines) {
    Eigen::Vector3f vec1(line.first.x(), line.second.y(), 0.0);
    Eigen::Vector3f vec2(line.second.x(), line.second.y(), 0.0);
    gui_helpers::AddLine(vec1, vec2, gui_helpers::Color4f::kWhite, &ros_struct.vis_marker);
  }
  // Now just publish the points.
  ros_struct.pub.publish(ros_struct.vis_marker);
}

static void WaitForUserInput() {
  char waiting_char;
  std::cin >>  waiting_char;
}

#endif
