#ifndef GRAVITY_SAND_CONFIG_H
#define GRAVITY_SAND_CONFIG_H

#include <stdint.h>
#include "../shared/BoardConfig.h"

// Fluid prototype. All simulation distances are display pixels, time is seconds.
namespace sand {
namespace config {
constexpr int kParticleCount = 100;
constexpr int kMinParticleCount = 3;
constexpr int kMaxParticleCount = 120;
constexpr float kDotRadius = 2.5f;
constexpr float kVisualSizeScale = 2.0f;     // Draw larger, overlapping shapes without changing physics spacing.
constexpr float kMinVisualVariation = 1.0f; // 100%: the original 10px visual baseline.
constexpr float kMaxVisualVariation = 3.5f; // 350%: 35px across, before motion stretch.
constexpr float kMaxPackedScale = 10.0f;    // Internal solver/raster capacity; randomize stays within visual bounds.
constexpr float kPackedCountBudget = 2400.0f; // Allows crowded/jammed rolls while bounding pair work.
constexpr float kMinSpawnCoverage = 0.16f; // Loose targets leave room inside roughly 15-35% visible fill.
constexpr float kMaxSpawnCoverage = 0.34f; // No crowded exception rolls.
constexpr float kDotStretchMin = 1.0f;       // Randomize chooses a population-wide cap from 1x to 3x.
constexpr float kDotStretchMax = 3.0f;       // Hard render limit; reciprocal width preserves area.
constexpr float kDotStretchStartSpeed = 35.0f; // Pixels/s; quiet dots use the original circle fast path.
constexpr float kDotStretchFullSpeed = 400.0f; // Pixels/s; faster motion keeps the same bounded shape.
constexpr int kCircleRadius = 117;
constexpr float kMinSizeScale = 1.0f;
constexpr float kMaxSizeScale = 5.0f;         // Legacy sparse solver API; packed mode has its own larger range.
constexpr float kCountJitterFraction = 0.20f;// +/-20% about the inverse-area baseline.
constexpr float kExplosionMinImpulse = 600.0f; // 100%: the previous strongest explosion.
constexpr float kExplosionMaxImpulse = 1200.0f; // 200%: twice the previous maximum, pixels/s at origin.
constexpr float kExplosionEdgeFraction = 0.25f; // Far dots still receive a visible push.
constexpr uint32_t kPhysicsHz = 240;
constexpr uint32_t kDisplayHz = 60; // Fixed visual cadence; leave comfortable drawing headroom.
constexpr uint32_t kLcdSPIHz = hardware::kLcdSpiHz;
constexpr uint32_t kImuI2cHz = 100000; // More margin for the pendant's wired GY-521 bus.
constexpr int kRenderMergeGapCells = 1;      // Renderer tiles, independent of physics.
constexpr int kRenderBandRows = 4;
constexpr int kRenderMergeOverdrawCells = 3;
constexpr uint32_t kRenderTaskStackBytes = 8192;
constexpr uint32_t kRenderTaskPriority = 1;
constexpr int kMaxCatchUpSteps = 4;
constexpr uint32_t kStatsIntervalMs = 2000;
constexpr uint32_t kLoopDelayMs = 1;

// A small, lively liquid: pressure keeps volume, near-pressure prevents clumps.
constexpr float kAcceleration = 1800.0f;    // Pixels/s^2 per g; quick shallow-tilt response.
constexpr float kMaxSpeed = 1200.0f;        // Allow the full 200% explosion impulse.
constexpr float kInteractionRadius = 15.0f;
constexpr float kRestDensity = 2.2f;
constexpr float kPressure = 20000.0f;
constexpr float kNearPressure = 35000.0f;
constexpr float kMaxAttraction = 6000.0f;    // Bounds free-surface cohesion.
constexpr float kViscosity = 5.0f;          // Local velocity exchange / second.
constexpr float kAirDamping = 0.55f;        // Global drag / second; lively splash, then settle.
constexpr int kRelaxationIterations = 2;
constexpr float kMinSeparation = 4.5f;      // Emergency anti-overlap, below normal spacing.
constexpr float kMaxRelaxation = 1.5f;      // Max pressure correction / particle / pass.
constexpr float kWallRestitution = 0.32f;
constexpr float kWallFriction = 0.005f;     // Retain almost all tangential wall motion.
constexpr float kWallBounceMinSpeed = 35.0f;// Stop quiet contacts from chattering.
constexpr float kGravityDeadzone = 0.012f;
constexpr float kMaxGravity = 2.0f;
constexpr float kMaxFrameSeconds = 0.05f;
constexpr float kMaxSubstepSeconds = 1.0f / 240.0f;
constexpr float kMaxSubstepTravelFraction = 0.95f; // Split only fast ticks that exceed this spacing fraction.
constexpr float kMaxKick = 360.0f;
constexpr float kShakeGain = 1.75f;
constexpr float kShoveGain = 2.0f;
constexpr float kShakePixelScale = 3.0f;    // Shared shake detector uses legacy 3px units.

constexpr bool kButtonsEnabled = true;
constexpr int kButtonExplosionPin = hardware::kButtonPrimary;
constexpr int kButtonPopulationPin = hardware::kButtonSecondary;
constexpr uint32_t kButtonDebounceMs = 25;

static_assert(kParticleCount >= kMinParticleCount && kParticleCount <= kMaxParticleCount &&
              kMinParticleCount >= 2 && kMaxParticleCount <= 200,
              "Choose a bounded default/capacity for the fluid arrays");
static_assert(kDotStretchMin >= 1 && kDotStretchMin <= kDotStretchMax && kDotStretchMax <= 3 &&
              kDotStretchStartSpeed >= 0 && kDotStretchFullSpeed > kDotStretchStartSpeed,
              "Keep visual stretch and its speed range bounded");
static_assert(kVisualSizeScale >= 1 && kVisualSizeScale <= 2,
              "Keep visual overlap within the bounded renderer footprint");
static_assert(kMinVisualVariation >= 1 && kMaxVisualVariation >= kMinVisualVariation &&
              kMaxVisualVariation <= kMaxPackedScale && kMaxPackedScale <= 10 &&
              kPackedCountBudget > 0 && kMinSpawnCoverage >= 0.15f &&
              kMaxSpawnCoverage <= 0.35f && kMaxSpawnCoverage > kMinSpawnCoverage,
              "Keep randomized fill targets within the lighter 15-35% range");
static_assert(kMinSizeScale == 1 && kMaxSizeScale >= kMinSizeScale && kMaxSizeScale <= 5 &&
              kCountJitterFraction >= 0 && kCountJitterFraction <= 1,
              "Invalid random size/count distribution");
static_assert(kExplosionMinImpulse > 0 && kExplosionMaxImpulse >= kExplosionMinImpulse &&
              kExplosionMaxImpulse <= kMaxSpeed && kExplosionEdgeFraction > 0 &&
              kExplosionEdgeFraction <= 1, "Invalid explosion impulse range");
static_assert(kPhysicsHz >= 60 && kPhysicsHz <= 1000 && kDisplayHz > 0 && kDisplayHz <= 240,
              "Choose practical positive update targets");
static_assert(kRenderBandRows >= 1 && kRenderBandRows <= 8 &&
              kRenderMergeGapCells >= 0 && kRenderMergeOverdrawCells >= 0,
              "Invalid renderer batching");
static_assert(kDotRadius > 0 && kDotRadius <= 5 && kInteractionRadius > 2 * kDotRadius &&
              kRestDensity > 0 && kPressure > 0 && kNearPressure > 0 && kMaxAttraction >= 0,
              "Invalid fluid support/density/pressure");
static_assert(kMaxSpeed > 0 && kMaxSubstepSeconds > 0 &&
              kMaxSubstepTravelFraction > 0 && kMaxSubstepTravelFraction < 1 &&
              kMaxSpeed * kMaxSubstepSeconds * 0.5f <
                  kMinSeparation * kMaxSubstepTravelFraction && kMinSeparation > 0 &&
              kMaxFrameSeconds > 0 && kMaxRelaxation > 0 &&
              kRelaxationIterations >= 1 && kRelaxationIterations <= 4,
              "Bound particle travel and pressure corrections");
static_assert(kAcceleration >= 0 && kMaxGravity > 0 && kGravityDeadzone >= 0 &&
              kViscosity >= 0 && kAirDamping >= 0 && kMaxKick > 0 &&
              kWallRestitution >= 0 && kWallRestitution <= 1 &&
              kWallFriction >= 0 && kWallFriction <= 1,
              "Invalid force/damping limits");
static_assert(kMaxCatchUpSteps >= 1 && kStatsIntervalMs >= 100 && kLoopDelayMs <= 10 &&
              kRenderTaskStackBytes >= 4096 && kRenderTaskPriority >= 1 && kRenderTaskPriority <= 3,
              "Invalid scheduler limits");
} // namespace config
} // namespace sand
#endif
