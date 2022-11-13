#include "cvplot/color.h"

#include <cmath>
#include <map>

namespace cvplot {

namespace {
std::map<std::string, int> color_counter;
}

Color Color::alpha(uint8_t alpha) const { return Color(r, g, b, alpha); }

Color Color::gamma(float gamma) const {
  return Color(pow(r / 255.f, 1 / gamma) * 255, pow(g / 255.f, 1 / gamma) * 255,
               pow(b / 255.f, 1 / gamma) * 255, a);
}

Color Color::gray(uint8_t v) { return Color(v, v, v); }

Color Color::index(uint8_t index, uint8_t density, float avoid,
                   float range) {  // avoid greens by default
  if (avoid > 0) {
    auto step = density / (6 - range);
    auto offset = (avoid + range / 2) * step;
    index = offset + index % density;
    density += step * range;
  }
  auto hue = index % density * 6.f / density;
  return Color::cos(hue);
}

Color Color::hash(const std::string &seed) {
  return Color::index(std::hash<std::string>{}(seed));
}

Color Color::uniq(const std::string &name) {
  if (color_counter.count(name) == 0) {
    color_counter[name] = color_counter.size();
  }
  return Color::index(color_counter[name]);
}

Color Color::hue(float hue) {
  Color color;
  auto i = (int)hue;
  auto f = (hue - i) * 255;
  switch (i % 6) {
    case 0:
      return Color(255, f, 0);
    case 1:
      return Color(255 - f, 255, 0);
    case 2:
      return Color(0, 255, f);
    case 3:
      return Color(0, 255 - f, 255);
    case 4:
      return Color(f, 0, 255);
    case 5:
      return Color(255, 0, 255 - f);
  }
  return Color();
}

Color Color::cos(float hue) {
  return Color((std::cos(hue * 1.047f) + 1) * 127.9f,
               (std::cos((hue - 2) * 1.047f) + 1) * 127.9f,
               (std::cos((hue - 4) * 1.047f) + 1) * 127.9f);
}

float Color::hue() const {
  auto min = std::min(std::min(r, g), b);
  auto max = std::max(std::max(r, g), b);
  if (min == max) {
    return 0;
  }
  auto hue = 0.f;
  if (r == max) {
    hue = (g - b) / (float)(max - min);
  } else if (g == max) {
    hue = 2.f + (b - r) / (float)(max - min);
  } else {
    hue = 4.f + (r - g) / (float)(max - min);
  }
  if (hue < 0) {
    hue += 6;
  }
  return hue;
}

}  // namespace cvplot
