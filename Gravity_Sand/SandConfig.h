#ifndef GRAVITY_SAND_CONFIG_H
#define GRAVITY_SAND_CONFIG_H

#include <stdint.h>

// Edit these values, then compile/upload Gravity_Sand. Units use one 3px cell.
namespace sand {
namespace config {

// Display/input scheduling. Keep these at 60Hz for a responsive screen.
constexpr uint32_t kPhysicsHz = 60;             // Simulation ticks/s; substeps stay safe.
constexpr uint32_t kDisplayHz = 60;             // Snapshots/s; stale pending frames replaced.
constexpr uint32_t kLcdSPIHz = 40000000;         // LCD bus Hz; 40MHz already worked on this wiring.
constexpr int kRenderMergeGapCells = 4;         // Bridge this many unchanged cells horizontally.
constexpr int kRenderBandRows = 4;              // 1..8 cell rows per display buffer/window.
constexpr int kRenderMergeOverdrawCells = 8;    // Extra cells allowed to merge vertical windows.
constexpr uint32_t kRenderTaskStackBytes = 8192;// Separate LCD task, pinned to the other core.
constexpr uint32_t kRenderTaskPriority = 1;     // Leave system/USB tasks able to run.
constexpr int kMaxCatchUpSteps = 3;             // Limit overdue physics work before resyncing.
constexpr uint32_t kStatsIntervalMs = 2000;     // Interval for Serial PERF diagnostics.
constexpr uint32_t kLoopDelayMs = 1;            // Yield between input polls; keep small.
constexpr float kShakeGain = 1.75f;             // Default agitation strength; +/- also adjusts live.
constexpr float kShoveGain = 4.0f;              // Extra directional response to detected movement.

// Normally open buttons: connect each pin to GND when pressed.
constexpr bool kButtonsEnabled = true;         // Unconnected pins remain high via INPUT_PULLUP.
constexpr int kButtonScatterPin = 3;            // D2/GPIO3: scatter. GPIO3 also selects JTAG at reset.
constexpr int kButtonPalettePin = 8;            // D9/GPIO8: palette. LCD MISO remains disabled.
constexpr uint32_t kButtonDebounceMs = 25;      // Stable press required; one action per press.

// Sand feel and computational work. One cell is 3 display pixels.
constexpr int kParticleCount = 1200;            // More grains cost more CPU/drawing.
constexpr float kAcceleration = 240.0f;        // Cells/s^2 per g; higher starts faster.
constexpr float kMaxSpeed = 90.0f;              // Cells/s; higher needs more substeps.
constexpr float kMinimumSubstepHz = 120.0f;     // Minimum internal update rate.
constexpr float kMaxCellTravel = 0.75f;         // Max cells/substep; keep below 1.
constexpr float kSlideRate = 70.0f;             // Rolls/s per g; higher loosens piles.
constexpr float kMinimumSlideRate = 12.0f;      // Rolls/s at shallow, nonzero tilt.
constexpr float kRollingRetention = 0.75f;      // Velocity kept after a roll, 0..1.
constexpr float kAirDamping = 0.7f;            // Velocity loss/s while moving freely.
constexpr float kWallRetention = 0.8f;         // Tangential velocity kept at wall, 0..1.
constexpr float kGravityDeadzone = 0.025f;      // In-plane g ignored to suppress noise.
constexpr float kMaxGravity = 2.0f;            // Maximum in-plane acceleration in g.
constexpr float kMaxFrameSeconds = 0.05f;       // Cap elapsed time after a long stall.
constexpr float kMaxKick = 30.0f;              // Cells/s maximum coherent/random kick.
constexpr float kTurnMinGravity = 0.08f;       // Minimum g for direction-change response.
constexpr float kTurnCosine = 0.70710678f;      // 45-degree change triggers pile release.
constexpr float kTurnBoostSeconds = 0.12f;     // Duration of faster acceleration on turn.
constexpr float kTurnAccelerationBoost = 1.4f; // Multiplier during the short turn boost.
constexpr float kOpposingRetention = 0.25f;    // Old uphill velocity kept on a turn, 0..1.

static_assert(kPhysicsHz > 0 && kPhysicsHz <= 1000 && kDisplayHz > 0 && kDisplayHz <= 240,
              "Choose practical positive update rates");
static_assert(kRenderBandRows >= 1 && kRenderBandRows <= 8 &&
              kRenderMergeGapCells >= 0 && kRenderMergeGapCells <= 80 &&
              kRenderMergeOverdrawCells >= 0 && kRenderMergeOverdrawCells <= 6400,
              "Invalid display batching settings");
static_assert(kMaxCatchUpSteps >= 1 && kStatsIntervalMs >= 100 && kLoopDelayMs <= 10 &&
              kRenderTaskStackBytes >= 4096 && kRenderTaskPriority >= 1 && kRenderTaskPriority <= 3,
              "Invalid scheduler/diagnostic settings");
static_assert(kShakeGain >= 0.25f && kShakeGain <= 3.0f && kShoveGain >= 0 &&
              kButtonScatterPin >= 0 && kButtonScatterPin <= 48 &&
              kButtonPalettePin >= 0 && kButtonPalettePin <= 48,
              "Invalid shake strength or GPIO number");
static_assert(kParticleCount > 0 && kParticleCount <= 4500,
              "Choose 1..4500 grains so reset always has free circular cells");
static_assert(kMaxSpeed > 0 && kMinimumSubstepHz > 0 &&
              kMaxCellTravel > 0 && kMaxCellTravel < 1,
              "Positive speed/rate and travel below one cell are required");
static_assert(kAcceleration >= 0 && kAirDamping >= 0 && kMaxFrameSeconds > 0 &&
              kSlideRate >= 0 && kMinimumSlideRate >= 0 && kMaxGravity > 0 &&
              kMaxKick > 0 && kGravityDeadzone >= 0,
              "Physics rates and limits must be nonnegative");
static_assert(kRollingRetention >= 0 && kRollingRetention <= 1 &&
              kWallRetention >= 0 && kWallRetention <= 1 &&
              kOpposingRetention >= 0 && kOpposingRetention <= 1,
              "Velocity retention must be between zero and one");
static_assert(kTurnCosine > -1 && kTurnCosine < 1 && kTurnMinGravity > 0 &&
              kTurnAccelerationBoost >= 1 && kTurnBoostSeconds >= 0,
              "Turn threshold/boost settings are invalid");

}  // namespace config
}  // namespace sand

#endif
