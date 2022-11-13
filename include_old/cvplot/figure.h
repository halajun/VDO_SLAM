#ifndef CVPLOT_FIGURE_H
#define CVPLOT_FIGURE_H

#include "color.h"
#include "window.h"

#include <map>
#include <string>
#include <vector>

namespace cvplot {

struct Point2 {
  float x, y;
  Point2() : Point2(0, 0) {}
  Point2(float x, float y) : x(x), y(y) {}
};

struct Point3 {
  float x, y, z;
  Point3() : Point3(0, 0, 0) {}
  Point3(float x, float y, float z) : x(x), y(y), z(z) {}
};

enum Type {
  Line,
  DotLine,
  Dots,
  FillLine,
  RangeLine,
  Histogram,
  Vistogram,
  Horizontal,
  Vertical,
  Range,
  Circle,
};

class Series {
 public:
  Series(const std::string &label, enum Type type, Color color)
      : label_(label),
        type_(type),
        color_(color),
        dims_(0),
        depth_(0),
        legend_(true),
        dynamic_color_(false) {}

  Series &type(enum Type type);
  Series &color(Color color);
  Series &dynamicColor(bool dynamic_color);
  Series &legend(bool legend);
  Series &add(const std::vector<std::pair<float, float>> &data);
  Series &add(const std::vector<std::pair<float, Point2>> &data);
  Series &add(const std::vector<std::pair<float, Point3>> &data);
  Series &addValue(const std::vector<float> &values);
  Series &addValue(const std::vector<Point2> &values);
  Series &addValue(const std::vector<Point3> &values);
  Series &add(float key, float value);
  Series &add(float key, Point2 value);
  Series &add(float key, Point3 value);
  Series &addValue(float value);
  Series &addValue(float value_a, float value_b);
  Series &addValue(float value_a, float value_b, float value_c);
  Series &set(const std::vector<std::pair<float, float>> &data);
  Series &set(const std::vector<std::pair<float, Point2>> &data);
  Series &set(const std::vector<std::pair<float, Point3>> &data);
  Series &setValue(const std::vector<float> &values);
  Series &setValue(const std::vector<Point2> &values);
  Series &setValue(const std::vector<Point3> &values);
  Series &set(float key, float value);
  Series &set(float key, float value_a, float value_b);
  Series &set(float key, float value_a, float value_b, float value_c);
  Series &setValue(float value);
  Series &setValue(float value_a, float value_b);
  Series &setValue(float value_a, float value_b, float value_c);
  Series &clear();

  const std::string &label() const;
  bool legend() const;
  Color color() const;
  void draw(void *buffer, float x_min, float x_max, float y_min, float y_max,
            float x_axis, float xs, float xd, float ys, float yd, float y_axis,
            int unit, float offset) const;
  bool collides() const;
  void dot(void *b, int x, int y, int r) const;
  void bounds(float &x_min, float &x_max, float &y_min, float &y_max,
              int &n_max, int &p_max) const;
  void verifyParams() const;

 protected:
  void ensureDimsDepth(int dims, int depth);
  bool flipAxis() const;

 protected:
  std::vector<int> entries_;
  std::vector<float> data_;
  enum Type type_;
  Color color_;
  std::string label_;
  int dims_;
  int depth_;
  bool legend_;
  bool dynamic_color_;
};

class Figure {
 public:
  Figure(View &view)
      : view_(view),
        border_size_(50),
        background_color_(White),
        axis_color_(Black),
        sub_axis_color_(Light),
        text_color_(Black),
        include_zero_x_(true),
        include_zero_y_(true),
        aspect_square_(false),
        grid_size_(60),
        grid_padding_(20) {}

  Figure &clear();
  Figure &origin(bool x, bool y);
  Figure &square(bool square);
  Figure &border(int size);
  Figure &alpha(int alpha);
  Figure &gridSize(int size);
  Figure &backgroundColor(Color color);
  Figure &axisColor(Color color);
  Figure &subaxisColor(Color color);
  Figure &textColor(Color color);
  Color backgroundColor();
  Color axisColor();
  Color subaxisColor();
  Color textColor();

  void draw(void *b, float x_min, float x_max, float y_min, float y_max,
            int n_max, int p_max) const;
  void show(bool flush = true) const;
  Series &series(const std::string &label);

 protected:
  View &view_;
  std::vector<Series> series_;
  int border_size_;
  Color background_color_;
  Color axis_color_;
  Color sub_axis_color_;
  Color text_color_;
  bool include_zero_x_;
  bool include_zero_y_;
  bool aspect_square_;
  int grid_size_;
  int grid_padding_;
};

Figure &figure(const std::string &view);

}  // namespace cvplot

#endif  // CVPLOT_FIGURE_H
