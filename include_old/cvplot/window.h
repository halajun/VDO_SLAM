#ifndef CVPLOT_WINDOW_H
#define CVPLOT_WINDOW_H

#include "color.h"

#include <map>
#include <string>

namespace cvplot {

struct Rect {
  int x, y, width, height;
  Rect(int x, int y, int width, int height)
      : x(x), y(y), width(width), height(height) {}
};

struct Size {
  int width, height;
  Size(int width, int height) : width(width), height(height) {}
};

struct Offset {
  int x, y;
  Offset(int x, int y) : x(x), y(y) {}
};

typedef void (*MouseCallback)(int event, int x, int y, int flags, void *param);
typedef void (*TrackbarCallback)(int pos, void *param);

class Window;

class View {
 public:
  View(Window &window, const std::string &title = "", Size size = {300, 300})
      : window_(window),
        title_(title),
        rect_(0, 0, size.width, size.height),
        frameless_(false),
        background_color_(Black),
        frame_color_(Green),
        text_color_(Black),
        mouse_callback_(NULL),
        mouse_param_(NULL) {}
  View &resize(Rect rect);
  View &size(Size size);
  View &offset(Offset offset);
  View &autosize();
  View &title(const std::string &title);
  View &alpha(int alpha);
  View &backgroundColor(Color color);
  View &frameColor(Color color);
  View &textColor(Color color);
  View &mouse(MouseCallback callback, void *param = NULL);
  void onmouse(int event, int x, int y, int flags);

  Color backgroundColor();
  Color frameColor();
  Color textColor();
  std::string &title();
  bool has(Offset offset);

  void drawRect(Rect rect, Color color);
  void drawFill(Color background = White);
  void drawImage(const void *image, int alpha = 255);
  void drawText(const std::string &text, Offset offset, Color color) const;
  void drawFrame(const std::string &title) const;
  void *buffer(Rect &rect);
  void finish();
  void flush();
  void hide(bool hidden = true);

  View &operator=(const View &) = delete;

 protected:
  Rect rect_;
  std::string title_;
  bool frameless_;
  Window &window_;
  Color background_color_;
  Color frame_color_;
  Color text_color_;
  MouseCallback mouse_callback_;
  void *mouse_param_;
  bool hidden_;
};

class Window {
 public:
  Window(const std::string &title = "");
  Window &resize(Rect rect);
  Window &size(Size size);
  Window &offset(Offset offset);
  Window &title(const std::string &title);
  Window &fps(float fps);
  Window &ensure(Rect rect);
  Window &cursor(bool cursor);
  void *buffer();
  void flush();
  View &view(const std::string &name, Size size = {300, 300});
  void dirty();
  void tick();
  void hide(bool hidden = true);
  void onmouse(int event, int x, int y, int flags);

  Window &operator=(const Window &) = delete;

  static Window &current();
  static void current(Window &window);
  static Window &current(const std::string &title);

 protected:
  Offset offset_;
  void *buffer_;
  std::string title_;
  std::string name_;
  std::map<std::string, View> views_;
  bool dirty_;
  float flush_time_;
  float fps_;
  bool hidden_;
  bool show_cursor_;
  Offset cursor_;
};

class Util {
 public:
  static void sleep(float seconds = 0);
  static char key(float timeout = 0);
  static std::string line(float timeout = 0);
};

}  // namespace cvplot

#endif  // CVPLOT_WINDOW_H
