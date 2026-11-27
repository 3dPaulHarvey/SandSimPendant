#include <assert.h>
#include <float.h>
#include <initializer_list>
#include <stdio.h>
#include "../Gravity_Sand/FluidCluster.h"

using sand::cluster::Point;

static Point origin(const Point *points, int count, float radius = 2.5f) {
  return sand::cluster::explosionOrigin(count, radius,
      [points](int i) -> const Point & { return points[i]; });
}

static float distanceSquared(Point a, Point b) {
  const float x = a.x - b.x, y = a.y - b.y;
  return x * x + y * y;
}

static void emptySingleAndInvalidInput() {
  int calls = 0;
  const Point empty = sand::cluster::explosionOrigin(0, 2.5f, [&calls](int) {
    ++calls; return Point{NAN, NAN};
  });
  assert(calls == 0 && empty.x == 119.5f && empty.y == 119.5f);
  const Point negative = origin(nullptr, -1);
  assert(negative.x == empty.x && negative.y == empty.y);
  const Point singleton[] = {{37, 94}};
  for (float radius : {2.5f, 12.5f, 0.0f, -1.0f, NAN, INFINITY, FLT_MAX}) {
    assert(distanceSquared(origin(singleton, 1, radius), singleton[0]) == 0);
  }
  const Point invalid[] = {{NAN, 50}, {INFINITY, 20}, {15, NAN}};
  assert(distanceSquared(origin(invalid, 3), empty) == 0);
  const Point mixed[] = {{NAN, 50}, {78, 90}, {15, INFINITY}};
  assert(distanceSquared(origin(mixed, 3), mixed[1]) == 0);
  Point coincident[120];
  for (Point &point : coincident) point = {46, 64};
  assert(distanceSquared(origin(coincident, 120), coincident[0]) == 0);
  calls = 0;
  sand::cluster::explosionOrigin(1000, 2.5f, [&calls](int i) {
    ++calls; assert(i < 120); return Point{46, 64};
  });
  assert(calls == 120);
}

static void denseClumpBeatsWiderSparseCloud() {
  // Four closely packed dots beat a much larger population spread over the
  // other side of the display; a global centroid would land in the empty gap.
  Point points[24] = {{40, 68}, {40, 72}, {44, 68}, {44, 72}};
  for (int i = 0; i < 20; ++i) points[i + 4] = {120.0f + (i % 4) * 25,
                                                           40.0f + (i / 4) * 25};
  const Point result = origin(points, 24);
  assert(distanceSquared(result, {42, 70}) < 0.1f);
  assert(result.x < 45 && result.y >= 68 && result.y <= 72);
  assert(distanceSquared(result, points[0]) > 1); // A local center, not just a dot.
}

static void disconnectedEqualClumpsDoNotAverageTogether() {
  const Point points[] = {{38, 78}, {42, 78}, {38, 82}, {42, 82},
                          {178, 158}, {182, 158}, {178, 162}, {182, 162}};
  const Point result = origin(points, 8);
  assert(distanceSquared(result, {40, 80}) < 0.1f);
  assert(distanceSquared(origin(points, 8), result) == 0);
  Point reordered[8];
  for (int i = 0; i < 8; ++i) reordered[i] = points[(i + 4) % 8];
  assert(distanceSquared(origin(reordered, 8), {180, 160}) < 0.1f);
}

static void sizeScalingAndSmallPopulations() {
  const Point base[] = {{-2, -2}, {2, -2}, {-2, 2}, {2, 2}, {20, 0}};
  Point reference = {0, 0};
  for (float scale : {1.0f, 2.0f, 3.0f, 5.0f}) {
    Point scaled[5];
    for (int i = 0; i < 5; ++i) scaled[i] = {100 + base[i].x * scale,
                                                           100 + base[i].y * scale};
    const Point result = origin(scaled, 5, 2.5f * scale);
    const Point normalized = {(result.x - 100) / scale, (result.y - 100) / scale};
    if (scale == 1) reference = normalized;
    else assert(distanceSquared(normalized, reference) < 0.00001f);
    assert(fabsf(normalized.x) < 0.3f && fabsf(normalized.y) < 0.3f);
  }
  const Point threeLarge[] = {{30, 70}, {36, 94}, {210, 180}};
  const Point result = origin(threeLarge, 3, 12.5f);
  assert(result.x > 30 && result.x < 36 && result.y > 70 && result.y < 94);
}

static void wallAndHollowGroupsStayPopulated() {
  Point wall[12];
  const Point center = {119.5f, 119.5f};
  for (int i = 0; i < 12; ++i) {
    const float angle = (i - 5.5f) * 0.04f;
    wall[i] = {center.x + 114.5f * cosf(angle), center.y + 114.5f * sinf(angle)};
  }
  const Point wallOrigin = origin(wall, 12);
  assert(distanceSquared(wallOrigin, center) <= 114.5f * 114.5f + 0.01f);
  assert(wallOrigin.x > 230);

  Point ring[12];
  for (int i = 0; i < 12; ++i) {
    const float angle = i * 6.28318530718f / 12;
    ring[i] = {100 + 8 * cosf(angle), 100 + 8 * sinf(angle)};
  }
  const Point ringOrigin = origin(ring, 12);
  float nearest = FLT_MAX;
  for (Point dot : ring) {
    const float distance = distanceSquared(dot, ringOrigin);
    if (distance < nearest) nearest = distance;
  }
  assert(nearest <= 25); // Never the empty center of this ring.
}

int main() {
  emptySingleAndInvalidInput();
  denseClumpBeatsWiderSparseCloud();
  disconnectedEqualClumpsDoNotAverageTogether();
  sizeScalingAndSmallPopulations();
  wallAndHollowGroupsStayPopulated();
  puts("Explosion origins select dense local fluid, avoid disconnected gaps, scale with dots, and remain finite and populated.");
}
