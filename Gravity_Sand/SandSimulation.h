#ifndef GRAVITY_SAND_SIMULATION_H
#define GRAVITY_SAND_SIMULATION_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "SandConfig.h"
#include "FluidVariation.h"
#include "FluidImpact.h"

// Arduino's default -Os leaves substantial per-neighbor call/loop overhead.
// Optimize only this solver for speed; retain IEEE finite checks and rounding.
#if defined(ARDUINO_ARCH_ESP32) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3")
#endif

namespace sand {
struct Particle {
  float x, y;   // Continuous display-pixel coordinates, +Y down.
  float vx, vy; // Pixels / second.
  uint8_t color;
};

// Allocation-free double-density relaxation fluid (Clavet et al., SCA 2005).
// No lattice, grain occupancy, rolling rules, or plastic springs. Symmetric
// pressure displacements and local viscosity let neighbors exchange momentum.
// A small spatial index finds nearby dots; it never quantizes their positions.
class Simulation {
 public:
  static constexpr int kDisplaySize = 240;
  static constexpr int kParticleCount = config::kParticleCount;
  static constexpr int kMaxParticleCount = config::kMaxParticleCount;
  static constexpr int kCircleRadius = config::kCircleRadius;
  static constexpr float kCenter = 119.5f;
  static constexpr float kBoundaryRadius = kCircleRadius - config::kDotRadius;

  Simulation() { reset(); }

  // Match the inverse-area population rule, including its +20% variation.
  // Reject overfilled configurations before changing any existing fluid state.
  static int maxCountForScale(float scale) {
    return variation::maxCountForScale(scale);
  }

  // A separate capacity keeps the old compact-fluid API stable while allowing
  // crowded giant particles. At 10x, 24 round bodies already exceed the
  // display's available physical area; no extra particles are needed to jam.
  static int maxPackedCountForScale(float scale) {
    return variation::maxPackedCountForScale(scale);
  }

  bool configure(float scale, int count, uint32_t seed = 1) {
    if (!variation::validScale(scale) || count < config::kMinParticleCount ||
        count > maxCountForScale(scale)) return false;
    packed_ = false;
    configureState(scale, count);
    reset(seed);
    return true;
  }

  bool configurePacked(float scale, int count, uint32_t seed = 1) {
    const int capacity = maxPackedCountForScale(scale);
    if (!capacity || count < config::kMinParticleCount || count > capacity) return false;
    packed_ = true;
    configureState(scale, count);
    reset(seed);
    return true;
  }

  int particleCount() const { return activeCount_; }
  float dotRadius() const { return radius_; }
  float sizeScale() const { return scale_; }
  float boundaryRadius() const { return boundaryRadius_; }
  bool packed() const { return packed_; }

  void reset(uint32_t seed = 1) {
    rng_ = seed ? seed : 1;
    for (int i = 0; i < activeCount_; ++i) impacts_[i].reset();
    if (packed_) {
      // Equal-area sunflower spacing fills the disk without projecting a
      // rectangular lattice onto the rim. Trig runs only on reset/randomize.
      const float rotation = static_cast<float>(random() >> 8) *
                             (6.283185307f / 16777216.0f);
      const float goldenAngle = 2.399963230f;
      for (int i = 0; i < activeCount_; ++i) {
        Particle &p = particles_[i];
        const float distance = boundaryRadius_ *
            sqrtf((static_cast<float>(i) + 0.5f) / activeCount_);
        const float angle = rotation + goldenAngle * i;
        p.x = kCenter + distance * cosf(angle);
        p.y = kCenter + distance * sinf(angle);
        p.vx = p.vy = 0;
        p.color = static_cast<uint8_t>(random() & 7);
        projectWall(p);
      }
      return;
    }
    // A loose central patch produces a visible first splash instead of waiting
    // for isolated dots to trickle in from every corner of the display. The
    // inverse-area count limit keeps this patch compact even at 5x dot size.
    const int columns = static_cast<int>(ceilf(sqrtf(static_cast<float>(activeCount_))));
    const int rows = (activeCount_ + columns - 1) / columns;
    const float spacing = 8.2f * scale_;
    for (int i = 0; i < activeCount_; ++i) {
      Particle &p = particles_[i];
      p.x = kCenter + ((i % columns) - (columns - 1) * 0.5f) * spacing + jitter() * scale_;
      p.y = kCenter - 14 + ((i / columns) - (rows - 1) * 0.5f) * spacing + jitter() * scale_;
      p.vx = p.vy = 0;
      p.color = static_cast<uint8_t>(random() & 7);
      projectWall(p);
    }
  }

  // One instantaneous impulse. Balanced agitation adds no net momentum until
  // speed limiting/contact. Finite extremes are scaled before squaring.
  void kick(float dx, float dy, float agitation) {
    if (!isfinite(dx) || !isfinite(dy) || !isfinite(agitation)) return;
    if (agitation < 0) agitation = 0;
    if (agitation > config::kMaxKick) agitation = config::kMaxKick;
    if (dx == 0 && dy == 0 && agitation == 0) return;
    clampVector(dx, dy, config::kMaxKick);
    static const float nx[8] = {1.41421356f,1,0,-1,-1.41421356f,-1,0,1};
    static const float ny[8] = {0,1,1.41421356f,1,0,-1,-1.41421356f,-1};
    for (int i = 0; i < activeCount_; i += 2) {
      float jx = 0, jy = 0;
      if (agitation > 0 && i + 1 < activeCount_) {
        const int direction = random() & 7;
        jx = nx[direction] * agitation;
        jy = ny[direction] * agitation;
      }
      addVelocity(particles_[i], dx + jx, dy + jy);
      if (i + 1 < activeCount_) addVelocity(particles_[i + 1], dx - jx, dy - jy);
    }
  }

  // A point blast changes velocity once; it never teleports or resets dots.
  // Inputs are screen coordinates and a peak impulse in pixels/second.
  void explode(float originX, float originY, float strength) {
    if (!isfinite(originX) || !isfinite(originY) || !isfinite(strength) ||
        originX < 0 || originY < 0 || originX >= kDisplaySize || originY >= kDisplaySize ||
        strength <= 0) return;
    if (strength > config::kExplosionMaxImpulse) strength = config::kExplosionMaxImpulse;
    static const float nx[8] = {1,0.70710678f,0,-0.70710678f,-1,-0.70710678f,0,0.70710678f};
    static const float ny[8] = {0,0.70710678f,1,0.70710678f,0,-0.70710678f,-1,-0.70710678f};
    for (int i = 0; i < activeCount_; ++i) {
      Particle &p = particles_[i];
      float x = p.x - originX, y = p.y - originY;
      const float distance = sqrtf(x * x + y * y);
      if (distance > 0.0001f) {
        const float inverseDistance = 1.0f / distance;
        x *= inverseDistance; y *= inverseDistance;
      } else {
        const int direction = random() & 7;
        x = nx[direction]; y = ny[direction];
      }
      const float fraction = distance < 2 * kCircleRadius ? distance / (2 * kCircleRadius) : 1.0f;
      const float impulse = strength * (config::kExplosionEdgeFraction +
        (1.0f - config::kExplosionEdgeFraction) * (1.0f - fraction));
      addVelocity(p, x * impulse, y * impulse);
    }
  }

  // Fixed 240Hz in firmware; other callers are safely split into <=1/240s.
  // Only fast bursts in the smallest particles need two half-steps. Larger
  // spacing and ordinary motion keep exactly the original integration path.
  // Zero in-plane tilt has no artificial downward force. A long stall cannot
  // trigger an unbounded catch-up or a single unstable integration step.
  void step(float gx, float gy, float seconds) {
    if (!isfinite(gx) || !isfinite(gy) || !isfinite(seconds) || seconds < 0.000001f) return;
    if (seconds > config::kMaxFrameSeconds) seconds = config::kMaxFrameSeconds;
    clampVector(gx, gy, config::kMaxGravity);
    if (gx * gx + gy * gy < config::kGravityDeadzone * config::kGravityDeadzone) gx = gy = 0;
    int count = static_cast<int>(ceilf(seconds / config::kMaxSubstepSeconds - 0.00001f));
    if (count < 1) count = 1;
    const float dt = seconds / count;
    for (int i = 0; i < count; ++i) {
      if (needsFastSubsteps(gx, gy, dt)) {
        advance(gx, gy, dt * 0.5f);
        advance(gx, gy, dt * 0.5f);
      } else advance(gx, gy, dt);
    }
  }

  const Particle &particle(size_t index) const { return particles_[index]; }
  const impact::Pulse &impact(size_t index) const { return impacts_[index]; }
  int pixelX(size_t index) const { return static_cast<int>(particles_[index].x + 0.5f); }
  int pixelY(size_t index) const { return static_cast<int>(particles_[index].y + 0.5f); }

 private:
  struct Vector { float x, y; };
  struct Pair { uint8_t a, b; };
  static constexpr int kMaxBins = static_cast<int>(kDisplaySize / config::kInteractionRadius) + 1;
  static constexpr int kMaxPairs = kMaxParticleCount * (kMaxParticleCount - 1) / 2;
  Particle particles_[kMaxParticleCount];
  impact::Pulse impacts_[kMaxParticleCount]; // Render-only; never feeds forces or positions.
  Vector predicted_[kMaxParticleCount], delta_[kMaxParticleCount];
  float density_[kMaxParticleCount], nearDensity_[kMaxParticleCount];
  int16_t heads_[kMaxBins * kMaxBins], next_[kMaxParticleCount];
  Pair pairs_[kMaxPairs]; // Every possible pair fits; no neighbor truncation.
  int pairCount_;
  uint32_t rng_;
  int activeCount_ = kParticleCount, bins_ = kMaxBins;
  bool packed_ = false;
  float scale_ = 1.0f, radius_ = config::kDotRadius;
  float support_ = config::kInteractionRadius, inverseSupport_ = 1.0f / config::kInteractionRadius;
  float minSeparation_ = config::kMinSeparation, maxRelaxation_ = config::kMaxRelaxation;
  float pressure_ = config::kPressure, nearPressure_ = config::kNearPressure;
  float maxAttraction_ = config::kMaxAttraction, boundaryRadius_ = kBoundaryRadius;
  // Positions do not change between density measurement and pair correction.
  // Reuse those exact distances instead of taking each square root twice.
  float pairDistances_[kMaxPairs];

  void configureState(float scale, int count) {
    activeCount_ = count;
    scale_ = scale;
    radius_ = config::kDotRadius * scale;
    support_ = config::kInteractionRadius * scale;
    inverseSupport_ = 1.0f / support_;
    minSeparation_ = config::kMinSeparation * scale;
    maxRelaxation_ = config::kMaxRelaxation * scale;
    pressure_ = config::kPressure * scale;
    nearPressure_ = config::kNearPressure * scale;
    maxAttraction_ = config::kMaxAttraction * scale;
    boundaryRadius_ = kCircleRadius - radius_;
    bins_ = static_cast<int>(kDisplaySize / support_) + 1;
  }

  uint32_t random() {
    rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
    return rng_;
  }
  float jitter() { return (static_cast<int>(random() & 1023) - 511.5f) / 1024.0f; }
  static void clampVector(float &x, float &y, float maximum) {
    const float largest = fmaxf(fabsf(x), fabsf(y));
    if (largest > maximum) { x = x / largest * maximum; y = y / largest * maximum; }
    const float squared = x * x + y * y;
    if (squared > maximum * maximum) {
      const float scale = maximum / sqrtf(squared); x *= scale; y *= scale;
    }
  }
  static void addVelocity(Particle &p, float dx, float dy) {
    p.vx += dx; p.vy += dy;
    limitSpeed(p);
  }
  // Internal velocities are already bounded by input and solver corrections.
  // Keep overflow-safe scaling at the public input boundary, not every dot.
  static void limitSpeed(Particle &p) {
    const float squared = p.vx * p.vx + p.vy * p.vy;
    if (squared > config::kMaxSpeed * config::kMaxSpeed) {
      const float scale = config::kMaxSpeed / sqrtf(squared);
      p.vx *= scale; p.vy *= scale;
    }
  }

  bool needsFastSubsteps(float gx, float gy, float dt) const {
    const float travelLimit = minSeparation_ * config::kMaxSubstepTravelFraction;
    // Most size selections fit even the maximum speed at the normal cadence.
    // Avoid scanning particles at all when their scaled spacing permits it.
    if (config::kMaxSpeed * dt <= travelLimit) return false;
    float speed2 = 0;
    for (int i = 0; i < activeCount_; ++i) {
      const Particle &p = particles_[i];
      const float candidate = p.vx * p.vx + p.vy * p.vy;
      if (candidate > speed2) speed2 = candidate;
    }
    // Acceleration can increase speed before integration; damping only reduces
    // it, and advance() enforces the speed ceiling before changing positions.
    float predictedSpeed = sqrtf(speed2) +
        sqrtf(gx * gx + gy * gy) * config::kAcceleration * dt;
    if (predictedSpeed > config::kMaxSpeed) predictedSpeed = config::kMaxSpeed;
    return predictedSpeed * dt > travelLimit;
  }
  void projectWall(Particle &p) {
    const float x = p.x - kCenter, y = p.y - kCenter;
    const float r2 = x * x + y * y;
    if (r2 > boundaryRadius_ * boundaryRadius_) {
      const float scale = boundaryRadius_ / sqrtf(r2);
      p.x = kCenter + x * scale; p.y = kCenter + y * scale;
    }
  }

  void findNeighbors() {
    for (int i = 0; i < bins_ * bins_; ++i) heads_[i] = -1;
    const float inverseSupport = inverseSupport_;
    const float support2 = support_ * support_;
    for (int i = 0; i < activeCount_; ++i) {
      const int x = static_cast<int>(particles_[i].x * inverseSupport);
      const int y = static_cast<int>(particles_[i].y * inverseSupport);
      const int bin = y * bins_ + x;
      next_[i] = heads_[bin];
      heads_[bin] = static_cast<int16_t>(i);
    }
    pairCount_ = 0;
    for (int i = 0; i < activeCount_; ++i) {
      const int x = static_cast<int>(particles_[i].x * inverseSupport);
      const int y = static_cast<int>(particles_[i].y * inverseSupport);
      const int left = x > 0 ? x - 1 : 0, right = x + 1 < bins_ ? x + 1 : bins_ - 1;
      const int top = y > 0 ? y - 1 : 0, bottom = y + 1 < bins_ ? y + 1 : bins_ - 1;
      for (int row = top; row <= bottom; ++row) {
        for (int col = left; col <= right; ++col) {
          for (int j = heads_[row * bins_ + col]; j >= 0; j = next_[j]) {
            if (j <= i) continue;
            const float dx = particles_[j].x - particles_[i].x;
            const float dy = particles_[j].y - particles_[i].y;
            if (dx * dx + dy * dy < support2)
              pairs_[pairCount_++] = {static_cast<uint8_t>(i), static_cast<uint8_t>(j)};
          }
        }
      }
    }
  }

  void relax(float dt) {
    for (int i = 0; i < activeCount_; ++i) {
      density_[i] = nearDensity_[i] = 0;
      delta_[i] = {0, 0};
    }
    const float inverseSupport = inverseSupport_;
    findNeighbors();
    for (int pair = 0; pair < pairCount_; ++pair) {
        const int i = pairs_[pair].a, j = pairs_[pair].b;
        const float x = particles_[j].x - particles_[i].x;
        const float y = particles_[j].y - particles_[i].y;
        const float r2 = x * x + y * y;
        const float distance = sqrtf(r2);
        pairDistances_[pair] = distance;
        const float q = 1.0f - distance * inverseSupport;
        const float q2 = q * q, q3 = q2 * q;
        density_[i] += q2; density_[j] += q2;
        nearDensity_[i] += q3; nearDensity_[j] += q3;
    }
    for (int i = 0; i < activeCount_; ++i) {
      const float pressure = pressure_ * (density_[i] - config::kRestDensity);
      density_[i] = pressure < -maxAttraction_ ? -maxAttraction_ : pressure;
      nearDensity_[i] *= nearPressure_;
    }
    const float scale = 0.5f * dt * dt / config::kRelaxationIterations;
    for (int pair = 0; pair < pairCount_; ++pair) {
        const int i = pairs_[pair].a, j = pairs_[pair].b;
        float x = particles_[j].x - particles_[i].x;
        float y = particles_[j].y - particles_[i].y;
        const float r = pairDistances_[pair];
        const float q = 1.0f - r * inverseSupport;
        if (r > 0.0001f) { const float inverseR = 1.0f / r; x *= inverseR; y *= inverseR; }
        else { // Deterministic symmetric escape from exact coincidence.
          x = ((i + j) & 1) ? 1.0f : 0.0f;
          y = ((i + j) & 1) ? 0.0f : 1.0f;
        }
        float displacement = scale * ((density_[i] + density_[j]) * q +
                              (nearDensity_[i] + nearDensity_[j]) * q * q);
        if (r < minSeparation_) {
          const float separation = (minSeparation_ - r) * 0.5f;
          if (displacement < separation) displacement = separation;
        }
        delta_[i].x -= x * displacement; delta_[i].y -= y * displacement;
        delta_[j].x += x * displacement; delta_[j].y += y * displacement;
    }
    // A shared scale (rather than per-particle clipping) retains equal/opposite
    // internal corrections even during a very crowded impact.
    float largest2 = maxRelaxation_ * maxRelaxation_;
    for (int i = 0; i < activeCount_; ++i) {
      const float d2 = delta_[i].x * delta_[i].x + delta_[i].y * delta_[i].y;
      if (d2 > largest2) largest2 = d2;
    }
    const float correctionScale = maxRelaxation_ / sqrtf(largest2);
    for (int i = 0; i < activeCount_; ++i) {
      particles_[i].x += delta_[i].x * correctionScale;
      particles_[i].y += delta_[i].y * correctionScale;
      projectWall(particles_[i]);
    }
  }

  void advance(float gx, float gy, float dt) {
    const float damping = 1.0f / (1.0f + config::kAirDamping * dt);
    for (int i = 0; i < activeCount_; ++i) {
      impacts_[i].advance(dt);
      Particle &p = particles_[i];
      p.vx = (p.vx + gx * config::kAcceleration * dt) * damping;
      p.vy = (p.vy + gy * config::kAcceleration * dt) * damping;
      limitSpeed(p);
      p.x += p.vx * dt; p.y += p.vy * dt;
      predicted_[i] = {p.x, p.y};
      projectWall(p);
    }
    for (int pass = 0; pass < config::kRelaxationIterations; ++pass) relax(dt);
    const float inverseDt = 1.0f / dt;
    for (int i = 0; i < activeCount_; ++i) {
      Particle &p = particles_[i];
      // Keep the integrated velocity, then add only constraint corrections.
      // Reconstructing the entire velocity from two ~120px float positions
      // quantizes away small drag changes at 240Hz, causing perpetual drift.
      float vx = p.vx + (p.x - predicted_[i].x) * inverseDt;
      float vy = p.vy + (p.y - predicted_[i].y) * inverseDt;
      const float x = p.x - kCenter, y = p.y - kCenter;
      const float r2 = x * x + y * y;
      if (r2 >= (boundaryRadius_ - 0.01f) * (boundaryRadius_ - 0.01f)) {
        const float inverseRadius = 1.0f / sqrtf(r2);
        const float nx = x * inverseRadius, ny = y * inverseRadius;
        const float incoming = p.vx * nx + p.vy * ny;
        impacts_[i].contact(incoming, nx, ny);
        const float normal = vx * nx + vy * ny;
        const float bounce = incoming > config::kWallBounceMinSpeed ?
                             -config::kWallRestitution * incoming : 0.0f;
        // Preserve tangential speed so a sideways crash runs up the round rim.
        if (normal > bounce) { vx += (bounce - normal) * nx; vy += (bounce - normal) * ny; }
        const float radial = vx * nx + vy * ny;
        vx -= (vx - radial * nx) * config::kWallFriction;
        vy -= (vy - radial * ny) * config::kWallFriction;
      } else {
        impacts_[i].leaveWall();
      }
      p.vx = vx; p.vy = vy;
      delta_[i] = {0, 0};
    }
    // Mild, pairwise momentum exchange; no global velocity averaging.
    findNeighbors();
    for (int pair = 0; pair < pairCount_; ++pair) {
        const int i = pairs_[pair].a, j = pairs_[pair].b;
        const float x = particles_[j].x - particles_[i].x;
        const float y = particles_[j].y - particles_[i].y;
        const float r2 = x * x + y * y;
        const float q = 1.0f - sqrtf(r2) * inverseSupport_;
        const float viscosity = config::kViscosity * dt * q * 0.5f;
        const float amount = viscosity < 0.025f ? viscosity : 0.025f;
        const float dx = (particles_[j].vx - particles_[i].vx) * amount;
        const float dy = (particles_[j].vy - particles_[i].vy) * amount;
        delta_[i].x += dx; delta_[i].y += dy;
        delta_[j].x -= dx; delta_[j].y -= dy;
    }
    for (int i = 0; i < activeCount_; ++i)
      addVelocity(particles_[i], delta_[i].x, delta_[i].y);
  }
};
} // namespace sand

#if defined(ARDUINO_ARCH_ESP32) && defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
#endif
