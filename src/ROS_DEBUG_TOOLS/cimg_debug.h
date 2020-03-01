//
// Created by jack on 2/28/20.
//

#ifndef ROS_DEBUG_TOOLS_CIMG_DEBUG_H
#define ROS_DEBUG_TOOLS_CIMG_DEBUG_H

#include "CImg.h"

using cimg_library::CImg;
using cimg_library::CImgDisplay;

struct Table {
    uint64_t width;
    uint64_t height;
    double resolution;
    CImg<double> values;
    Table(const double range, const double resolution) :
            width(floor((range * 2.0) / resolution)),
            height(floor((range * 2.0) / resolution)),
            resolution(resolution) {
      // Construct a width x height image, with only 1 z level.
      // And, only one double per color with default value 0.0.
      values = CImg<double>(width, height, 1, 1, 0.0);
    }

    Table() : width(0), height(0), resolution(1) {}

    inline uint64_t convertX(float x) const {
      return width / 2 + floor(x / resolution);
    }

    inline uint64_t convertY(float y) const {
      return height / 2 + floor(y / resolution);
    }

    inline double GetPointValue(Vector2f point) const {
      uint64_t x = convertX(point.x());
      uint64_t y = convertY(point.y());
      return values(x, y);
    }

    void SetPointValue(Vector2f point, double value) {
      uint64_t x = convertX(point.x());
      uint64_t y = convertY(point.y());
      if (x >= width || y >= height) {
        return;
      }
      values(x, y) = value;
    }

    CImg<double> GetDebugImage() const {
      return values;
    }
};

Table GetTable(const vector<Vector2f>& pointcloud,
               double range,
               double resolution) {
  Table table(range, resolution);
  for (const Vector2f& point : pointcloud) {
    table.SetPointValue(point, 1);
  }
  return table;
}

double largest_side(const vector<Vector2f>& points) {
  double min_x = INFINITY;
  double max_x = -INFINITY;
  double min_y = INFINITY;
  double max_y = -INFINITY;
  for (const Vector2f& p : points) {
    min_x = std::min(static_cast<double>(p.x()), min_x);
    max_x = std::max(static_cast<double>(p.x()), max_x);
    min_y = std::min(static_cast<double>(p.y()), min_y);
    max_y = std::max(static_cast<double>(p.y()), max_y);
  }
  return std::max(max_y - min_y, max_x - min_x);
}

static CImgDisplay PubPoints(std::string display_name,
                              const vector<Vector2f>& points) {
  double width = largest_side(points);
  // Plot the points in a display.
  Table table = GetTable(points, width / 2, 0.03);
  CImg<double> image = table.GetDebugImage();
  if (image.width() > 400) {
    image = image.resize_halfXY();
  }
  CImgDisplay display(image, display_name.c_str());
  return display;
}

static void WaitForClose(CImgDisplay display) {
  while (!display.is_closed()) {
    display.wait();
  }
}

#endif //ROS_DEBUG_TOOLS_CIMG_DEBUG_H
