#ifndef GRAVITY_SAND_SIMULATION_H
#define GRAVITY_SAND_SIMULATION_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

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
  static constexpr int kParticleCount = 1200;
  static constexpr int kCircleRadius = 117;
  static constexpr uint16_t kEmpty = 0xFFFF;
  static constexpr uint16_t kWall = 0xFFFE;

  Simulation() { reset(); }

  // Start with grains scattered through the disk. The seed also determines
  // grain colors and the unbiased update/slide order.
  void reset(uint32_t seed = 1) {
    rng_ = seed ? seed : 1;
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

  // Screen-space acceleration in g: +X right, +Y down. At zero acceleration
  // grains coast with damping; there is no artificial fallback gravity here.
  // Long frames are capped so a stalled USB reader cannot explode the solver.
  void step(float gravityX, float gravityY, float dtSeconds) {
    if (!isfinite(gravityX) || !isfinite(gravityY) ||
        !isfinite(dtSeconds) || dtSeconds <= 0.0f) return;
    if (dtSeconds > 0.05f) dtSeconds = 0.05f;
    float magnitude = sqrtf(gravityX * gravityX + gravityY * gravityY);
    if (magnitude > 2.0f) {
      gravityX *= 2.0f / magnitude;
      gravityY *= 2.0f / magnitude;
      magnitude = 2.0f;
    }
    // Suppress accelerometer noise while the screen is nearly face-up.
    if (magnitude < 0.025f) gravityX = gravityY = magnitude = 0.0f;
    int substeps = static_cast<int>(ceilf(dtSeconds * 120.0f - 0.0001f));
    if (substeps < 1) substeps = 1;
    const float dt = dtSeconds / substeps;
    for (int s = 0; s < substeps; ++s) {
      int i = static_cast<int>(random() % kParticleCount);
      const int direction = (random() & 1) ? 1 : -1;
      for (int n = 0; n < kParticleCount; ++n) {
        advance(static_cast<uint16_t>(i), gravityX, gravityY, magnitude, dt);
        i += direction;
        if (i < 0) i = kParticleCount - 1;
        if (i >= kParticleCount) i = 0;
      }
    }
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
  Particle particles_[kParticleCount];
  uint16_t cells_[kGridSize * kGridSize];
  uint32_t rng_;

  uint32_t random() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
  }

  bool freeFor(int col, int row, uint16_t id) const {
    const uint16_t occupant = particleAt(col, row);
    return occupant == kEmpty || occupant == id;
  }

  bool moveTo(uint16_t id, float x, float y) {
    const int col = static_cast<int>(floorf(x));
    const int row = static_cast<int>(floorf(y));
    if (!freeFor(col, row, id)) return false;
    Particle &p = particles_[id];
    const uint16_t cell = static_cast<uint16_t>(row * kGridSize + col);
    if (cell != p.cell) {
      cells_[p.cell] = kEmpty;
      cells_[cell] = id;
      p.cell = cell;
    }
    p.x = x;
    p.y = y;
    return true;
  }

  void advance(uint16_t id, float gx, float gy, float magnitude, float dt) {
    Particle &p = particles_[id];
    const float damping = 1.0f - 0.7f * dt;
    p.vx = (p.vx + gx * 240.0f * dt) * damping;
    p.vy = (p.vy + gy * 240.0f * dt) * damping;
    const float speed2 = p.vx * p.vx + p.vy * p.vy;
    // Less than one cell per substep prevents tunneling through a grain.
    if (speed2 > 90.0f * 90.0f) {
      const float scale = 90.0f / sqrtf(speed2);
      p.vx *= scale;
      p.vy *= scale;
    }
    const float nx = p.x + p.vx * dt;
    const float ny = p.y + p.vy * dt;
    const bool hitWall = particleAt(static_cast<int>(floorf(nx)),
                                   static_cast<int>(floorf(ny))) == kWall;
    const float incomingVx = p.vx;
    const float incomingVy = p.vy;
    // Diagonal cell moves are allowed: the grains are dots, and forbidding
    // these moves can lock an entire old pile in place after a 90 degree turn.
    if (moveTo(id, nx, ny)) {
      p.slide = 0.0f;
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
        p.vx = (incomingVx - normalX * outward / normal2) * 0.8f;
        p.vy = (incomingVy - normalY * outward / normal2) * 0.8f;
      }
    }
    if (magnitude == 0.0f) { p.slide = 0.0f; return; }

    p.slide += 35.0f * magnitude * dt;
    if (p.slide < 1.0f) return;
    p.slide = 1.0f;
    const int currentCol = p.cell % kGridSize;
    const int currentRow = p.cell / kGridSize;
    float best = -1.0f;
    int bestX = 0, bestY = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if ((dx == 0 && dy == 0) ||
            !freeFor(currentCol + dx, currentRow + dy, id)) continue;
        const float downhill = dx * gx + dy * gy;
        if (downhill < magnitude * 0.15f) continue;
        const float alignment = downhill * ((dx && dy) ? 0.70710678f : 1.0f);
        const float tieBreak = (random() & 255) * (0.04f / 255.0f) * magnitude;
        if (alignment + tieBreak > best) {
          best = alignment + tieBreak;
          bestX = dx;
          bestY = dy;
        }
      }
    }
    if (best >= 0.0f) {
      moveTo(id, currentCol + bestX + 0.5f, currentRow + bestY + 0.5f);
      p.vx *= 0.5f;
      p.vy *= 0.5f;
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
