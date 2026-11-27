#ifndef GRAVITY_SAND_FLUID_SHAPES_H
#define GRAVITY_SAND_FLUID_SHAPES_H

#include <stdint.h>

namespace sand {
namespace render {

enum class Shape : uint8_t { Circle, Triangle, Square, Star, Count };

inline bool validShape(Shape shape) {
  return static_cast<uint8_t>(shape) < static_cast<uint8_t>(Shape::Count);
}

inline const char *shapeName(Shape shape) {
  switch (shape) {
    case Shape::Triangle: return "triangle";
    case Shape::Square: return "square";
    case Shape::Star: return "star";
    default: return "circle";
  }
}

struct ShapeVertex { float x, y; };
struct ShapePolygon { const ShapeVertex *vertices; int count; };

// Upright outlines inscribed in the render radius's unit circle. Constants keep
// all trigonometry out of the frame loop. A star is a simple concave polygon.
inline ShapePolygon shapePolygon(Shape shape) {
  static const ShapeVertex triangle[] = {
    {0, -1}, {0.866025404f, 0.5f}, {-0.866025404f, 0.5f}
  };
  static const ShapeVertex square[] = {
    {-0.707106781f, -0.707106781f}, {0.707106781f, -0.707106781f},
    {0.707106781f, 0.707106781f}, {-0.707106781f, 0.707106781f}
  };
  static const ShapeVertex star[] = {
    {0, -1}, {0.293892626f, -0.404508497f},
    {0.951056516f, -0.309016994f}, {0.475528258f, 0.154508497f},
    {0.587785252f, 0.809016994f}, {0, 0.5f},
    {-0.587785252f, 0.809016994f}, {-0.475528258f, 0.154508497f},
    {-0.951056516f, -0.309016994f}, {-0.293892626f, -0.404508497f}
  };
  switch (shape) {
    case Shape::Triangle: return {triangle, 3};
    case Shape::Square: return {square, 4};
    case Shape::Star: return {star, 10};
    default: return {nullptr, 0};
  }
}

} // namespace render
} // namespace sand
#endif
