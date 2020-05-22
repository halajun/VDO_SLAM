#include "cvplot/window.h"
#include "internal.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ctime>

namespace cvplot {

namespace {
Window *shared_window = NULL;
int shared_index = 0;
clock_t shared_time = clock();
}  // namespace

float runtime() { return (float)(clock() - shared_time) / CLOCKS_PER_SEC; }

void mouse_callback(int event, int x, int y, int flags, void *window) {
  ((Window *)window)->onmouse(event, x, y, flags);
}

// View

View &View::resize(Rect rect) {
  rect_ = rect;
  window_.dirty();
  return *this;
}

View &View::size(Size size) {
  rect_.width = size.width;
  rect_.height = size.height;
  window_.dirty();
  return *this;
}

View &View::offset(Offset offset) {
  rect_.x = offset.x;
  rect_.y = offset.y;
  window_.dirty();
  return *this;
}

View &View::autosize() {
  size({0, 0});
  return *this;
}

View &View::title(const std::string &title) {
  title_ = title;
  window_.dirty();
  return *this;
}

View &View::alpha(int alpha) {
  background_color_ = background_color_.alpha(alpha);
  frame_color_ = frame_color_.alpha(alpha);
  text_color_ = text_color_.alpha(alpha);
  window_.dirty();
  return *this;
}

View &View::backgroundColor(Color color) {
  background_color_ = color;
  window_.dirty();
  return *this;
}

View &View::frameColor(Color color) {
  frame_color_ = color;
  window_.dirty();
  return *this;
}

View &View::textColor(Color color) {
  text_color_ = color;
  window_.dirty();
  return *this;
}

View &View::mouse(MouseCallback callback, void *param) {
  mouse_callback_ = callback;
  mouse_param_ = (param == NULL ? this : param);
  return *this;
}

Color View::backgroundColor() { return background_color_; }

Color View::frameColor() { return frame_color_; }

Color View::textColor() { return text_color_; }
std::string &View::title() { return title_; }

void View::drawRect(Rect rect, Color color) {
  Trans trans(window_.buffer());
  cv::rectangle(trans.with(color), {rect_.x + rect.x, rect_.y + rect.y},
                {rect_.x + rect.x + rect.width, rect_.y + rect.y + rect.height},
                color2scalar(color), -1);
  window_.dirty();
}

void View::drawText(const std::string &text, Offset offset, Color color) const {
  auto face = cv::FONT_HERSHEY_SIMPLEX;
  auto scale = 0.4f;
  auto thickness = 1.f;
  int baseline;
  cv::Size size = getTextSize(text, face, scale, thickness, &baseline);
  cv::Point org(rect_.x + offset.x, rect_.y + size.height + offset.y);
  Trans trans(window_.buffer());
  cv::putText(trans.with(color), text.c_str(), org, face, scale,
              color2scalar(color), thickness);
  window_.dirty();
}

void View::drawFrame(const std::string &title) const {
  Trans trans(window_.buffer());
  cv::rectangle(trans.with(background_color_), {rect_.x, rect_.y},
                {rect_.x + rect_.width - 1, rect_.y + rect_.height - 1},
                color2scalar(background_color_), 1);
  cv::rectangle(trans.with(frame_color_), {rect_.x + 1, rect_.y + 1},
                {rect_.x + rect_.width - 2, rect_.y + rect_.height - 2},
                color2scalar(frame_color_), 1);
  cv::rectangle(trans.with(frame_color_), {rect_.x + 2, rect_.y + 2},
                {rect_.x + rect_.width - 3, rect_.y + 16},
                color2scalar(frame_color_), -1);
  int baseline;
  cv::Size size =
      getTextSize(title.c_str(), cv::FONT_HERSHEY_PLAIN, 1.f, 1.f, &baseline);
  cv::putText(trans.with(text_color_), title.c_str(),
              {rect_.x + 2 + (rect_.width - size.width) / 2, rect_.y + 14},
              cv::FONT_HERSHEY_PLAIN, 1.f, color2scalar(text_color_), 1.f);
  window_.dirty();
}

void View::drawImage(const void *image, int alpha) {
  auto &img = *(cv::Mat *)image;
  if (rect_.width == 0 && rect_.height == 0) {
    rect_.width = img.cols;
    rect_.height = img.rows;
  }
  window_.ensure(rect_);
  Trans trans(window_.buffer());
  if (img.cols != rect_.width || img.rows != rect_.height) {
    cv::Mat resized;
    cv::resize(img, resized, {rect_.width, rect_.height});
    resized.copyTo(
        trans.with(alpha)({rect_.x, rect_.y, rect_.width, rect_.height}));
  } else {
    img.copyTo(
        trans.with(alpha)({rect_.x, rect_.y, rect_.width, rect_.height}));
  }
  window_.dirty();
}

void View::drawFill(Color color) {
  Trans trans(window_.buffer());
  cv::rectangle(trans.with(color), {rect_.x, rect_.y},
                {rect_.x + rect_.width - 1, rect_.y + rect_.height - 1},
                color2scalar(color), -1);
  window_.dirty();
}

void *View::buffer(Rect &rect) {
  window_.ensure(rect_);
  rect = rect_;
  return window_.buffer();
}

void View::finish() {
  if (!frameless_) {
    drawFrame(title_);
  }
  window_.dirty();
}

void View::flush() { window_.flush(); }

bool View::has(Offset offset) {
  return offset.x >= rect_.x && offset.y >= rect_.y &&
         offset.x < rect_.x + rect_.width && offset.y < rect_.y + rect_.height;
}

void View::onmouse(int event, int x, int y, int flags) {
  if (mouse_callback_ != NULL) {
    mouse_callback_(event, x, y, flags, mouse_param_);
  }
}

void View::hide(bool hidden) {
  if (hidden_ != hidden) {
    hidden_ = hidden;
    drawFill();
  }
}

// Window

Window::Window(const std::string &title)
    : offset_(0, 0),
      buffer_(NULL),
      title_(title),
      dirty_(false),
      flush_time_(0),
      fps_(1),
      hidden_(false),
      show_cursor_(false),
      cursor_(-10, -10),
      name_("cvplot_" + std::to_string(shared_index++)) {}

void *Window::buffer() { return buffer_; }

Window &Window::resize(Rect rect) {
  offset({rect.x, rect.y});
  size({rect.width, rect.height});
  return *this;
}

Window &Window::size(Size size) {
  auto &buffer = *(new cv::Mat(cv::Size(size.width, size.height), CV_8UC3,
                               color2scalar(Gray)));
  if (buffer_ != NULL) {
    auto &current = *(cv::Mat *)buffer_;
    if (current.cols > 0 && current.rows > 0 && size.width > 0 &&
        size.height > 0) {
      cv::Rect inter(0, 0, std::min(current.cols, size.width),
                     std::min(current.rows, size.height));
      current(inter).copyTo(buffer(inter));
    }
    delete &current;
  }
  buffer_ = &buffer;
  dirty();
  return *this;
}

Window &Window::offset(Offset offset) {
  offset_ = offset;
  cv::namedWindow(name_, cv::WINDOW_AUTOSIZE);
  cv::moveWindow(name_, offset.x, offset.y);
  return *this;
}

Window &Window::title(const std::string &title) {
  title_ = title;
  return *this;
}

Window &Window::fps(float fps) {
  fps_ = fps;
  return *this;
}

Window &Window::cursor(bool cursor) {
  show_cursor_ = cursor;
  return *this;
}

Window &Window::ensure(Rect rect) {
  if (buffer_ == NULL) {
    size({rect.x + rect.width, rect.y + rect.height});
  } else {
    auto &b = *(cv::Mat *)buffer_;
    if (rect.x + rect.width > b.cols || rect.y + rect.height > b.rows) {
      size({std::max(b.cols, rect.x + rect.width),
            std::max(b.rows, rect.y + rect.height)});
    }
  }
  return *this;
}

void Window::onmouse(int event, int x, int y, int flags) {
  for (auto &pair : views_) {
    auto &view = pair.second;
    if (view.has({x, y})) {
      view.onmouse(event, x, y, flags);
      break;
    }
  }
  cursor_ = {x, y};
  if (show_cursor_) {
    dirty();
    flush();
  }
}

void Window::flush() {
  if (dirty_ && buffer_ != NULL) {
    auto b = (cv::Mat *)buffer_;
    if (b->cols > 0 && b->rows > 0) {
      cv::Mat mat;
      if (show_cursor_) {
        b->copyTo(mat);
        cv::line(mat, {cursor_.x - 4, cursor_.y + 1},
                 {cursor_.x + 6, cursor_.y + 1}, color2scalar(White), 1);
        cv::line(mat, {cursor_.x + 1, cursor_.y - 4},
                 {cursor_.x + 1, cursor_.y + 6}, color2scalar(White), 1);
        cv::line(mat, {cursor_.x - 5, cursor_.y}, {cursor_.x + 5, cursor_.y},
                 color2scalar(Black), 1);
        cv::line(mat, {cursor_.x, cursor_.y - 5}, {cursor_.x, cursor_.y + 5},
                 color2scalar(Black), 1);
        b = &mat;
      }
      cv::namedWindow(name_, cv::WINDOW_AUTOSIZE);
#if CV_MAJOR_VERSION > 2
      cv::setWindowTitle(name_, title_);
#endif
      cv::imshow(name_.c_str(), *b);
      cv::setMouseCallback(name_.c_str(), mouse_callback, this);
      Util::sleep();
    }
  }
  dirty_ = false;
  flush_time_ = runtime();
}

View &Window::view(const std::string &name, Size size) {
  if (views_.count(name) == 0) {
    views_.insert(
        std::map<std::string, View>::value_type(name, View(*this, name, size)));
  }
  return views_.at(name);
}

void Window::tick() {
  if (fps_ > 0 && (runtime() - flush_time_) > 1.f / fps_) {
    flush();
  }
}

void Window::dirty() { dirty_ = true; }

void Window::hide(bool hidden) {
  if (hidden_ != hidden) {
    hidden_ = hidden;
    if (hidden) {
      cv::destroyWindow(name_.c_str());
    } else {
      dirty();
      flush();
    }
  }
}

Window &Window::current() {
  if (shared_window == NULL) {
    shared_window = new Window("");
  }
  return *(Window *)shared_window;
}

Window &Window::current(const std::string &title) {
  shared_window = new Window(title);
  return *(Window *)shared_window;
}

void Window::current(Window &window) { shared_window = &window; }

// Util

void Util::sleep(float seconds) {
  cv::waitKey(std::max(1, (int)(seconds * 1000)));
}

char Util::key(float timeout) {
  return cv::waitKey(std::max(0, (int)(timeout * 1000)));
}

std::string Util::line(float timeout) {
  std::stringstream stream;
  auto ms = (timeout > 0 ? std::max(1, (int)(timeout * 1000)) : -1);
  while (ms != 0) {
    auto key = cv::waitKey(1);
    if (key == -1) {
      ms--;
      continue;
    }
    if (key == '\r' || key <= '\n') {
      break;
    } else if (key == '\b' || key == 127) {
      auto s = stream.str();
      stream = std::stringstream();
      if (s.length() > 0) {
        stream << s.substr(0, s.length() - 1);
      }
    } else {
      stream << (char)key;
    }
  }
  return stream.str();
}

}  // namespace cvplot
