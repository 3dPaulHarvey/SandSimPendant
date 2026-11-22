#ifndef GRAVITY_SAND_SIMULATION_H
#define GRAVITY_SAND_SIMULATION_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "SandConfig.h"

// A small, allocation-free particle/grid hybrid. Each grain owns exactly one
// cell. Subcell velocity gives falling grains inertia; blocked grains can roll
// into a free downhill neighbor, making piles instead of overlapping points.
namespace sand {

struct Particle {
  float x, y;       // Cell coordinates, including a fractional position.
  float vx, vy;     // Cells per second.
  float slide;     // Fractional opportunity to roll along a pile.
  uint16_t cell;
  uint8_t color;    // Palette index 0..7, stable for this grain.
};

class Simulation {
 public:
  static constexpr int kDisplaySize = 240;
  static constexpr int kCellSize = 3;
  static constexpr int kGridSize = kDisplaySize / kCellSize;
  static constexpr int kParticleCount = config::kParticleCount;
  static constexpr int kCircleRadius = 117;
  static constexpr uint16_t kEmpty = 0xFFFF;
  static constexpr uint16_t kWall = 0xFFFE;

  Simulation() { reset(); }

  // Start with grains scattered through the disk. The seed also determines
  // grain colors and the unbiased update/slide order.
  void reset(uint32_t seed = 1) {
    rng_ = seed ? seed : 1;
    turnReferenceReady_ = false;
    turnReferenceX_ = turnReferenceY_ = 0;
    turnBoostRemaining_ = 0;
    for (int row = 0; row < kGridSize; ++row) {
      for (int col = 0; col < kGridSize; ++col) {
        cells_[row * kGridSize + col] = inside(col, row) ? kEmpty : kWall;
      }
    }
    for (int i = 0; i < kParticleCount; ++i) {
      int col, row;
      do {
        col = static_cast<int>(random() % kGridSize);
        row = static_cast<int>(random() % kGridSize);
      } while (cells_[row * kGridSize + col] != kEmpty);
      Particle &p = particles_[i];
      p.x = col + 0.5f;
      p.y = row + 0.5f;
      p.vx = p.vy = p.slide = 0.0f;
      p.cell = static_cast<uint16_t>(row * kGridSize + col);
      p.color = static_cast<uint8_t>(random() & 7);
      cells_[p.cell] = static_cast<uint16_t>(i);
    }
  }

  // Explicit shake impulse, in cells/sec. XY is a coherent velocity change;
  // agitation is zero-mean per-axis RMS velocity noise, useful for normal/Z
  // bumps. Callers control event timing; no hidden dt multiplier is applied.
  // Balanced pairs add no random net momentum before contact/speed limiting.
  // Quiet tilt physics is unchanged when this method is not called (or zero).
  void kick(float deltaVx, float deltaVy, float agitation) {
    if (!isfinite(deltaVx) || !isfinite(deltaVy) || !isfinite(agitation)) return;
    if (agitation < 0.0f) agitation = 0.0f;
    if (agitation > config::kMaxKick) agitation = config::kMaxKick;
    if (deltaVx == 0.0f && deltaVy == 0.0f && agitation == 0.0f) return;

    // Scale first so even huge but finite input cannot overflow its norm.
    float largest = fabsf(deltaVx);
    if (fabsf(deltaVy) > largest) largest = fabsf(deltaVy);
    if (largest > config::kMaxKick) {
      deltaVx = (deltaVx / largest) * config::kMaxKick;
      deltaVy = (deltaVy / largest) * config::kMaxKick;
    }
    const float coherent2 = deltaVx * deltaVx + deltaVy * deltaVy;
    if (coherent2 > config::kMaxKick * config::kMaxKick) {
      const float scale = config::kMaxKick / sqrtf(coherent2);
      deltaVx *= scale;
      deltaVy *= scale;
    }

    // Eight directions avoid per-grain trig or sqrt. Each axis has unit RMS;
    // total random impulse magnitude is sqrt(2)*agitation. Opposite impulses
    // in each pair give exact zero-sum jitter before float rounding/clamping.
    static const float directionX[8] = {1.41421356f, 1, 0, -1, -1.41421356f, -1, 0, 1};
    static const float directionY[8] = {0, 1, 1.41421356f, 1, 0, -1, -1.41421356f, -1};
    for (int i = 0; i + 1 < kParticleCount; i += 2) {
      float jitterX = 0, jitterY = 0;
      if (agitation > 0.0f) {
        const uint32_t direction = random() & 7;
        jitterX = directionX[direction] * agitation;
        jitterY = directionY[direction] * agitation;
      }
      addVelocity(particles_[i], deltaVx + jitterX, deltaVy + jitterY);
      addVelocity(particles_[i + 1], deltaVx - jitterX, deltaVy - jitterY);
    }
    // A trailing odd grain receives coherent motion only, preserving the
    // zero-sum random impulse without restricting configurable grain counts.
    if (kParticleCount & 1) addVelocity(particles_[kParticleCount - 1], deltaVx, deltaVy);
  }

  // Screen-space acceleration in g: +X right, +Y down. At zero acceleration
  // grains coast with damping; there is no artificial fallback gravity here.
  // Long frames are capped so a stalled USB reader cannot explode the solver.
  void step(float gravityX, float gravityY, float dtSeconds) {
    if (!isfinite(gravityX) || !isfinite(gravityY) ||
        !isfinite(dtSeconds) || dtSeconds <= 0.0f) return;
    if (dtSeconds > config::kMaxFrameSeconds) dtSeconds = config::kMaxFrameSeconds;
    float largest = fabsf(gravityX);
    if (fabsf(gravityY) > largest) largest = fabsf(gravityY);
    if (largest > config::kMaxGravity) {
      gravityX = (gravityX / largest) * config::kMaxGravity;
      gravityY = (gravityY / largest) * config::kMaxGravity;
    }
    float magnitude = sqrtf(gravityX * gravityX + gravityY * gravityY);
    if (magnitude > config::kMaxGravity) {
      gravityX *= config::kMaxGravity / magnitude;
      gravityY *= config::kMaxGravity / magnitude;
      magnitude = config::kMaxGravity;
    }
    // Suppress accelerometer noise while the screen is nearly face-up.
    if (magnitude < config::kGravityDeadzone) gravityX = gravityY = magnitude = 0.0f;
    updateTurnResponse(gravityX, gravityY, magnitude);
    // Derive the safety rate from the configured speed cap. Raising speed or
    // lowering the minimum rate cannot make grains skip an occupied cell.
    float rate = config::kMaxSpeed / config::kMaxCellTravel;
    if (rate < config::kMinimumSubstepHz) rate = config::kMinimumSubstepHz;
    int substeps = static_cast<int>(ceilf(dtSeconds * rate - 0.0001f));
    if (substeps < 1) substeps = 1;
    const float dt = dtSeconds / substeps;
    StepParameters parameters;
    parameters.dt = dt;
    parameters.damping = 1.0f - config::kAirDamping * dt;
    if (parameters.damping < 0.0f) parameters.damping = 0.0f;
    const float boost = turnBoostRemaining_ > 0 ? config::kTurnAccelerationBoost : 1.0f;
    parameters.accelX = gravityX * config::kAcceleration * boost * dt;
    parameters.accelY = gravityY * config::kAcceleration * boost * dt;
    float slideRate = config::kSlideRate * magnitude;
    if (magnitude > 0 && slideRate < config::kMinimumSlideRate)
      slideRate = config::kMinimumSlideRate;
    parameters.slideIncrement = slideRate * dt;
    parameters.magnitude = magnitude;
    parameters.neighborCount = 0;
    // Preserve the original neighbor order and tie-break RNG calls, while
    // doing gravity-dependent work once rather than once per blocked grain.
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const float downhill = dx * gravityX + dy * gravityY;
        if (downhill < magnitude * 0.15f) continue;
        Neighbor &neighbor = parameters.neighbors[parameters.neighborCount++];
        neighbor.dx = dx;
        neighbor.dy = dy;
        neighbor.offset = dy * kGridSize + dx;
        neighbor.alignment = downhill * ((dx && dy) ? 0.70710678f : 1.0f);
      }
    }
    for (int s = 0; s < substeps; ++s) {
      int i = static_cast<int>(random() % kParticleCount);
      const int direction = (random() & 1) ? 1 : -1;
      for (int n = 0; n < kParticleCount; ++n) {
        advance(static_cast<uint16_t>(i), parameters);
        i += direction;
        if (i < 0) i = kParticleCount - 1;
        if (i >= kParticleCount) i = 0;
      }
    }
    turnBoostRemaining_ -= dtSeconds;
    if (turnBoostRemaining_ < 0) turnBoostRemaining_ = 0;
  }

  const Particle &particle(size_t index) const { return particles_[index]; }
  int pixelX(size_t index) const {
    return (particles_[index].cell % kGridSize) * kCellSize;
  }
  int pixelY(size_t index) const {
    return (particles_[index].cell / kGridSize) * kCellSize;
  }
  uint16_t particleAt(int col, int row) const {
    if (col < 0 || row < 0 || col >= kGridSize || row >= kGridSize) return kWall;
    return cells_[row * kGridSize + col];
  }

  // Check the farthest pixel corner of a 3x3 grain against a circle centered
  // at (119.5,119.5). Arithmetic is doubled to avoid float work in this mask.
  static bool inside(int col, int row) {
    if (col < 0 || row < 0 || col >= kGridSize || row >= kGridSize) return false;
    int dx = col * 6 - 237;
    int dy = row * 6 - 237;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    dx += 2;
    dy += 2;
    return dx * dx + dy * dy <= 4 * kCircleRadius * kCircleRadius;
  }

 private:
  struct Neighbor {
    int dx, dy, offset;
    float alignment;
  };
  struct StepParameters {
    float dt, damping, accelX, accelY, slideIncrement, magnitude;
    int neighborCount;
    Neighbor neighbors[8];
  };
  Particle particles_[kParticleCount];
  uint16_t cells_[kGridSize * kGridSize];
  uint32_t rng_;
  bool turnReferenceReady_;
  float turnReferenceX_, turnReferenceY_, turnBoostRemaining_;

  void updateTurnResponse(float gravityX, float gravityY, float magnitude) {
    if (magnitude < config::kTurnMinGravity) {
      turnReferenceReady_ = false;
      turnBoostRemaining_ = 0;
      return;
    }
    const float directionX = gravityX / magnitude;
    const float directionY = gravityY / magnitude;
    if (turnReferenceReady_ && directionX * turnReferenceX_ +
        directionY * turnReferenceY_ >= config::kTurnCosine) return;
    // The reference only advances after a full threshold crossing. Small
    // sensor fluctuations cannot repeatedly release a resting pile.
    if (turnReferenceReady_) {
      turnBoostRemaining_ = config::kTurnBoostSeconds;
      for (int i = 0; i < kParticleCount; ++i) {
        Particle &p = particles_[i];
        p.slide = 1.0f;
        const float uphill = p.vx * directionX + p.vy * directionY;
        if (uphill < 0) {
          const float remove = uphill * (1.0f - config::kOpposingRetention);
          p.vx -= directionX * remove;
          p.vy -= directionY * remove;
        }
      }
    }
    turnReferenceX_ = directionX;
    turnReferenceY_ = directionY;
    turnReferenceReady_ = true;
  }

  static void addVelocity(Particle &particle, float deltaVx, float deltaVy) {
    particle.vx += deltaVx;
    particle.vy += deltaVy;
    const float speed2 = particle.vx * particle.vx + particle.vy * particle.vy;
    if (speed2 > config::kMaxSpeed * config::kMaxSpeed) {
      const float scale = config::kMaxSpeed / sqrtf(speed2);
      particle.vx *= scale;
      particle.vy *= scale;
    }
  }

  uint32_t random() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
  }

  bool moveTo(uint16_t id, float x, float y) {
    // Reject invalid proposals before the unchecked cell helper, even if
    // tuning values are changed later. Valid circular cells have positive x/y.
    if (!(x >= 0 && y >= 0 && x < kGridSize && y < kGridSize)) return false;
    const int col = static_cast<int>(x);
    const int row = static_cast<int>(y);
    const uint16_t cell = static_cast<uint16_t>(row * kGridSize + col);
    return moveToCell(id, cell, x, y);
  }

  bool moveToCell(uint16_t id, uint16_t cell, float x, float y) {
    if (cells_[cell] != kEmpty && cells_[cell] != id) return false;
    Particle &p = particles_[id];
    if (cell != p.cell) {
      cells_[p.cell] = kEmpty;
      cells_[cell] = id;
      p.cell = cell;
    }
    p.x = x;
    p.y = y;
    return true;
  }

  void advance(uint16_t id, const StepParameters &parameters) {
    Particle &p = particles_[id];
    p.vx = (p.vx + parameters.accelX) * parameters.damping;
    p.vy = (p.vy + parameters.accelY) * parameters.damping;
    const float speed2 = p.vx * p.vx + p.vy * p.vy;
    // Less than one cell per substep prevents tunneling through a grain.
    if (speed2 > config::kMaxSpeed * config::kMaxSpeed) {
      const float scale = config::kMaxSpeed / sqrtf(speed2);
      p.vx *= scale;
      p.vy *= scale;
    }
    const float nx = p.x + p.vx * parameters.dt;
    const float ny = p.y + p.vy * parameters.dt;
    // Small, speed-aware substeps and the circle's one-cell border make this
    // direct lookup safe. moveTo() also guards contact-resolution proposals.
    const uint16_t nextCell = static_cast<uint16_t>(static_cast<int>(ny) *
                                                  kGridSize + static_cast<int>(nx));
    const bool hitWall = cells_[nextCell] == kWall;
    const float incomingVx = p.vx;
    const float incomingVy = p.vy;
    const uint16_t oldCell = p.cell;
    // Diagonal cell moves are allowed: the grains are dots, and forbidding
    // these moves can lock an entire old pile in place after a 90 degree turn.
    if (moveToCell(id, nextCell, nx, ny)) {
      // Fractional movement inside a blocked grain's old cell must not erase
      // its accumulated rolling opportunity on every other substep.
      if (nextCell != oldCell || parameters.magnitude == 0) p.slide = 0.0f;
      return;
    }

    // Preserve movement along a contact, discard movement into the obstacle.
    // Resolve both orders over time so diagonal gravity has no fixed bias.
    if (random() & 1) {
      moveX(id, nx);
      moveY(id, ny);
    } else {
      moveY(id, ny);
      moveX(id, nx);
    }
    if (hitWall) {
      // Project impact velocity onto the circular wall. Without this, square
      // grid stair-steps can hold an old pile in place after a quarter-turn.
      const float normalX = p.x - kGridSize * 0.5f;
      const float normalY = p.y - kGridSize * 0.5f;
      const float normal2 = normalX * normalX + normalY * normalY;
      const float outward = incomingVx * normalX + incomingVy * normalY;
      if (normal2 > 1.0f && outward > 0.0f) {
        p.vx = (incomingVx - normalX * outward / normal2) * config::kWallRetention;
        p.vy = (incomingVy - normalY * outward / normal2) * config::kWallRetention;
      }
    }
    if (parameters.magnitude == 0.0f) { p.slide = 0.0f; return; }

    p.slide += parameters.slideIncrement;
    if (p.slide < 1.0f) return;
    p.slide = 1.0f;
    const int currentCol = p.cell % kGridSize;
    const int currentRow = p.cell / kGridSize;
    float best = -1.0f;
    int bestX = 0, bestY = 0;
    for (int n = 0; n < parameters.neighborCount; ++n) {
      const Neighbor &neighbor = parameters.neighbors[n];
      // A valid cell is at least one cell from the grid edge. Every neighbor
      // index is therefore safe; kWall already marks the circular exterior.
      if (cells_[p.cell + neighbor.offset] != kEmpty) continue;
      const float tieBreak = (random() & 255) * (0.04f / 255.0f) * parameters.magnitude;
      if (neighbor.alignment + tieBreak > best) {
        best = neighbor.alignment + tieBreak;
        bestX = neighbor.dx;
        bestY = neighbor.dy;
      }
    }
    if (best >= 0.0f) {
      moveTo(id, currentCol + bestX + 0.5f, currentRow + bestY + 0.5f);
      p.vx *= config::kRollingRetention;
      p.vy *= config::kRollingRetention;
      p.slide = 0.0f;
    }
  }

  void moveX(uint16_t id, float nx) {
    Particle &p = particles_[id];
    if (!moveTo(id, nx, p.y)) {
      p.x = (p.cell % kGridSize) + ((p.vx > 0.0f) ? 0.999f : 0.001f);
      p.vx = 0.0f;
    }
  }

  void moveY(uint16_t id, float ny) {
    Particle &p = particles_[id];
    if (!moveTo(id, p.x, ny)) {
      p.y = (p.cell / kGridSize) + ((p.vy > 0.0f) ? 0.999f : 0.001f);
      p.vy = 0.0f;
    }
  }
};

}  // namespace sand

#endif
