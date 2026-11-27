#include "../Gravity_Sand/FluidBackground.h"
#include "../Gravity_Sand/FluidRenderer.h"
#include "../Gravity_Sand/FluidVariation.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

using sand::render::Dot;
using sand::render::FluidBackground;
using sand::render::FluidRaster;
using sand::render::Options;
using sand::render::PixelRect;
using sand::render::Shape;

static FluidRaster raster;
static FluidBackground background;
static uint16_t panel[FluidRaster::kPixels];

struct Random {
  uint32_t state;
  explicit Random(uint32_t seed) : state(seed) {}
  uint32_t operator()() {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
  }
};

struct StuckRandom {
  unsigned calls = 0;
  uint32_t operator()() { ++calls; return 0; }
};

// Independent channel arithmetic verifies both tint interpolation and the
// particle edge's coverage against its actual row, rather than against black.
static uint16_t referenceBlend(uint16_t foreground, uint16_t backdrop,
                               unsigned coverage, unsigned denominator) {
  const unsigned shifts[] = {11, 5, 0};
  const unsigned masks[] = {31, 63, 31};
  uint16_t result = 0;
  for (int channel = 0; channel < 3; ++channel) {
    const unsigned fg = (foreground >> shifts[channel]) & masks[channel];
    const unsigned bg = (backdrop >> shifts[channel]) & masks[channel];
    const unsigned mixed = (fg * coverage + bg * (denominator - coverage)) / denominator;
    result |= static_cast<uint16_t>(mixed << shifts[channel]);
  }
  return result;
}

struct Transfers {
  int windows = 0;
  int pixels = 0;
  int invalidatedTiles = 0;
  int fullRedraws = 0;
};

static bool tileVisible(int x, int y) {
  for (int row = y; row < y + FluidBackground::kCellSize; ++row)
    for (int column = x; column < x + FluidBackground::kCellSize; ++column) {
      const uint8_t encoded = raster.pixel(column, row);
      if (!encoded || (encoded & 3)) return true;
    }
  return false;
}

// Apply only the dirty windows sent to the panel, including cells updated while
// hidden under opaque particles. Every revealed pixel must use the current BG.
static Transfers repaint(uint8_t palette, uint32_t seed, uint32_t now, int budget,
                         const Dot *dots = nullptr, int count = 0,
                         Shape shape = Shape::Circle, float radius = 8.25f) {
  Transfers result;
  raster.draw(dots, count, radius, shape);
  background.prepare(palette, seed, now, budget, tileVisible,
    [&](int x, int y, int width, int height) {
      assert(x >= 0 && y >= 0 && width > 0 && height > 0);
      assert(x + width <= 240 && y + height <= 240);
      assert(!(x % 4) && !(y % 4) && !(width % 4) && !(height % 4));
      result.invalidatedTiles += width * height / 16;
      if (width == 240 && height == 240) ++result.fullRedraws;
      else assert(width == 4 && height == 4);
      raster.invalidate(x, y, width, height);
    });
  raster.planPixels(Options(1, 4, 3), [&](const PixelRect &rectangle) {
    assert(rectangle.w && rectangle.h && rectangle.x + rectangle.w <= 240);
    assert(rectangle.y + rectangle.h <= 240 && rectangle.h <= 16);
    ++result.windows;
    result.pixels += rectangle.w * rectangle.h;
    for (int y = rectangle.y; y < rectangle.y + rectangle.h; ++y)
      for (int x = rectangle.x; x < rectangle.x + rectangle.w; ++x)
        panel[y * 240 + x] = background.composite(raster.pixel(x, y), x, y);
  });
  for (int y = 0; y < 240; ++y)
    for (int x = 0; x < 240; ++x)
      assert(panel[y * 240 + x] == background.composite(raster.pixel(x, y), x, y));
  raster.commit();
  return result;
}

static void capture(const FluidBackground &field, uint16_t *cells) {
  for (int y = 0; y < FluidBackground::kCells; ++y)
    for (int x = 0; x < FluidBackground::kCells; ++x)
      cells[y * FluidBackground::kCells + x] = field.colorAt(x * 4, y * 4);
}

static int differences(const FluidBackground &field, const uint16_t *cells) {
  int changed = 0;
  for (int y = 0; y < FluidBackground::kCells; ++y)
    for (int x = 0; x < FluidBackground::kCells; ++x)
      changed += cells[y * FluidBackground::kCells + x] != field.colorAt(x * 4, y * 4);
  return changed;
}

static bool allVisible(int, int) { return true; }
static bool allHidden(int, int) { return false; }
static void ignoreRectangle(int, int, int, int) {}

static void catalogAndCoverage() {
  namespace palette = sand::palette;
  assert(palette::kCount == 21 && palette::kForegroundCount == 5 && palette::kBackgroundCount == 3);
  assert(palette::rgb565(0xFF0000) == 0xF800);
  assert(palette::rgb565(0x00FF00) == 0x07E0);
  assert(palette::rgb565(0x0000FF) == 0x001F);
  assert(palette::rgb565(0xFFFFFF) == 0xFFFF);
  assert(&palette::get(255) == &palette::get(0));
  for (int index = 0; index < palette::kCount; ++index) {
    const palette::Swatch &swatch = palette::get(static_cast<uint8_t>(index));
    assert(swatch.id && swatch.id[0] && swatch.name && swatch.name[0]);
    for (int earlier = 0; earlier < index; ++earlier) {
      assert(strcmp(swatch.id, palette::get(static_cast<uint8_t>(earlier)).id) != 0);
      assert(strcmp(swatch.name, palette::get(static_cast<uint8_t>(earlier)).name) != 0);
    }
    const Transfers change = repaint(static_cast<uint8_t>(index), 0xFA3C1234U,
                                     index * 1000U, 0);
    assert(change.fullRedraws == 1 && change.pixels == 240 * 240);
    bool horizontalVariation = false, verticalVariation = false;
    for (int y = 0; y < 240; y += 7) {
      for (int x = 0; x < 240; x += 7) {
        const uint16_t backdrop = background.colorAt(x, y);
        assert(background.composite(0, x, y) == backdrop);
        if (x >= 7) horizontalVariation |= backdrop != background.colorAt(x - 7, y);
        if (y >= 7) verticalVariation |= backdrop != background.colorAt(x, y - 7);
        for (int color = 0; color < 8; ++color)
          for (int coverage = 1; coverage <= 4; ++coverage) {
            const uint16_t foreground = swatch.foreground[color % 5];
            const uint8_t encoded = static_cast<uint8_t>(color * 4 + coverage);
            assert(background.composite(encoded, x, y) ==
                   referenceBlend(foreground, backdrop, coverage, 4));
          }
      }
    }
    assert(horizontalVariation && verticalVariation);
    assert(repaint(static_cast<uint8_t>(index), 0xFA3C1234U, index * 1000U, 0).windows == 0);
  }
  const uint16_t foregrounds[] = {0x0000, 0xFFFF, 0xF800, 0x07E0, 0x001F, 0x5AB3};
  for (uint16_t fg : foregrounds)
    for (uint16_t bg : foregrounds)
      for (unsigned amount = 0; amount <= 8; ++amount)
        assert(FluidBackground::blend(fg, bg, amount, 8) == referenceBlend(fg, bg, amount, 8));
  printf("All %d swatches produce 2D gradient fields; foreground AA blends against each pixel's current backdrop.\n", palette::kCount);
}

static void animatedRepaints() {
  const uint32_t seed = 0x19D5A42BU;
  uint32_t now = 20000;
  repaint(0, seed, now, 0);
  Dot dots[] = {{80.25f, 72.25f, 0, 150, 30},
                {87.5f, 77.75f, 4, -100, 90},
                {135.5f, 174.25f, 2, 100, -170},
                {2.5f, 120.5f, 1, 0, 230},
                {237.25f, 45.5f, 3, 130, 30}};
  int animatedFrames = 0, maximumTiles = 0;
  for (int frame = 0; frame < 240; ++frame) {
    now += 33;
    dots[0].y += (frame < 120 ? 0.13f : -0.13f);
    dots[1].x += (frame < 120 ? 0.17f : -0.17f);
    const Shape shape = static_cast<Shape>((frame / 24) % static_cast<int>(Shape::Count));
    const int count = frame % 37 == 0 ? 0 : 5;
    const int budget = frame % 3 == 0 ? 12 : 80;
    const Transfers result = repaint(0, seed, now, budget, dots, count, shape);
    assert(!result.fullRedraws && result.invalidatedTiles <= budget);
    if (result.invalidatedTiles) ++animatedFrames;
    if (result.invalidatedTiles > maximumTiles) maximumTiles = result.invalidatedTiles;
  }
  assert(animatedFrames > 40 && maximumTiles > 0);
  repaint(0, seed, now, 0);

  // Removing a status overlay must restore gradients where no dot was drawn.
  for (int y = 24; y < 48; ++y)
    for (int x = 72; x < 168; ++x) panel[y * 240 + x] = 0xFFFF;
  raster.invalidate(72, 24, 96, 24);
  assert(repaint(0, seed, now, 0).windows > 0);

  uint16_t paused[FluidBackground::kCellCount];
  capture(background, paused);
  for (int frame = 0; frame < 12; ++frame) {
    now += 1000;
    assert(repaint(0, seed, now, 0).windows == 0);
    assert(differences(background, paused) == 0);
  }
  assert(repaint(0, seed, now + FluidBackground::kTickMs, 12).invalidatedTiles <= 12);
  printf("Moving gradients repaint exactly beneath overlapping shapes; visible updates obey a %d-tile budget.\n",
         maximumTiles);
}

static void seedMotionAndHiddenUpdates() {
  FluidBackground field, repeat, hidden;
  const uint32_t seed = 0x4793AA12U;
  uint32_t now = 1000;
  field.prepare(8, seed, now, FluidBackground::kCellCount, allVisible, ignoreRectangle);
  repeat.prepare(8, seed, now, FluidBackground::kCellCount, allVisible, ignoreRectangle);
  hidden.prepare(8, seed, now, 1, allHidden, ignoreRectangle);
  uint16_t initial[FluidBackground::kCellCount], reference[FluidBackground::kCellCount];
  capture(field, initial);
  assert(differences(repeat, initial) == 0 && differences(hidden, initial) == 0);
  int callbacks = 0, firstSecondChanges = 0;
  for (int frame = 1; frame <= 30; ++frame) {
    now += FluidBackground::kTickMs;
    field.prepare(8, seed, now, FluidBackground::kCellCount, allVisible, ignoreRectangle);
    repeat.prepare(8, seed, now, FluidBackground::kCellCount, allVisible, ignoreRectangle);
    hidden.prepare(8, seed, now, 1, allHidden, [&](int, int, int, int) { ++callbacks; });
    capture(field, reference);
    assert(differences(repeat, reference) == 0);
    // Hidden cells advance without wasting SPI bandwidth. Revealing them later
    // must show exactly the same latest field as the unrestricted reference.
    assert(differences(hidden, reference) == 0);
    if (frame == 10) firstSecondChanges = differences(field, initial);
  }
  assert(callbacks == 0 && firstSecondChanges > 20);
  field.prepare(8, seed + 1, now, 0, allVisible, ignoreRectangle);
  assert(differences(field, initial) > 20);
  field.prepare(8, seed, now, 0, allVisible, ignoreRectangle);
  assert(differences(field, initial) == 0);

  // Exercise actual occlusion and subsequent particle removal through the LCD
  // model, including partially covered cells around the foreground silhouette.
  Dot giant[] = {{120, 120, 4}, {100, 120, 2}, {140, 120, 0}};
  repaint(8, seed, now, 0, giant, 3, Shape::Circle, 50);
  for (int frame = 0; frame < 24; ++frame) {
    now += FluidBackground::kTickMs;
    const Transfers change = repaint(8, seed, now, 24, giant, 3, Shape::Circle, 50);
    assert(change.invalidatedTiles <= 24);
  }
  assert(repaint(8, seed, now, 0).windows > 0);
  printf("Seeded effects repeat exactly, hidden updates match the visible reference, and %d grid cells change within one second.\n",
         firstSecondChanges);
}

static void timerWrapAndPaletteChange() {
  const uint32_t seed = 0x304A2211U;
  repaint(1, seed, UINT32_MAX - 250U, 0);
  assert(repaint(1, seed, UINT32_MAX - 20U, 16).invalidatedTiles <= 16);
  assert(repaint(1, seed, 200U, 16).invalidatedTiles <= 16);
  const Dot fixed[] = {{120.25f, 120.75f, 4}, {70.5f, 62.5f, 1}};
  repaint(1, seed, 250, 0, fixed, 2);
  const Transfers changed = repaint(2, seed, 250, 0, fixed, 2);
  assert(changed.fullRedraws == 1 && changed.pixels == 240 * 240);
  assert(repaint(2, seed, 250, 0, fixed, 2).windows == 0);
  const Transfers newEffect = repaint(2, seed + 1, 250, 0, fixed, 2);
  assert(newEffect.fullRedraws == 1 && newEffect.pixels == 240 * 240);
  assert(repaint(2, seed + 1, 250, 0, fixed, 2).windows == 0);
  assert(repaint(2, seed + 1, 250, 0).windows > 0);
  puts("Palette/effect changes repaint the complete gradient; pause and timer wrap keep updates bounded.");
}

static void pausedAndLateClocks() {
  FluidBackground paused, reference;
  const uint32_t seed = 0x155C204BU;
  paused.prepare(4, seed, 1000, 0, allVisible, ignoreRectangle);
  reference.prepare(4, seed, 1000, 0, allVisible, ignoreRectangle);
  uint16_t picture[FluidBackground::kCellCount];
  capture(reference, picture);
  int callbacks = 0;
  paused.prepare(4, seed, 61000, -1, allVisible,
                 [&](int, int, int, int) { ++callbacks; });
  assert(!callbacks && differences(paused, picture) == 0);
  paused.prepare(4, seed, 61100, FluidBackground::kCellCount, allVisible, ignoreRectangle);
  reference.prepare(4, seed, 1100, FluidBackground::kCellCount, allVisible, ignoreRectangle);
  capture(reference, picture);
  assert(differences(paused, picture) == 0); // A minute paused does not accrue motion.

  paused.prepare(4, seed + 1, UINT32_MAX - 99U, 0, allVisible, ignoreRectangle);
  reference.prepare(4, seed + 1, 1000, 0, allVisible, ignoreRectangle);
  paused.prepare(4, seed + 1, 100, FluidBackground::kCellCount, allVisible, ignoreRectangle);
  reference.prepare(4, seed + 1, 1200, FluidBackground::kCellCount, allVisible, ignoreRectangle);
  capture(reference, picture);
  assert(differences(paused, picture) == 0); // 200ms across wrap equals ordinary elapsed time.

  paused.prepare(4, seed + 2, 1000, 0, allVisible, ignoreRectangle);
  reference.prepare(4, seed + 2, 1000, 0, allVisible, ignoreRectangle);
  paused.prepare(4, seed + 2, 61000, FluidBackground::kCellCount, allVisible, ignoreRectangle);
  reference.prepare(4, seed + 2, 1200, FluidBackground::kCellCount, allVisible, ignoreRectangle);
  capture(reference, picture);
  assert(differences(paused, picture) == 0); // Late frames take one bounded motion step.
  assert(paused.colorAt(-1, 0) == 0 && paused.colorAt(0, -1) == 0);
  assert(paused.colorAt(240, 0) == 0 && paused.colorAt(0, 240) == 0);
  assert(paused.composite(255, 120, 120) == paused.colorAt(120, 120));
}

static void randomPaletteAndBalancedColors() {
  namespace palette = sand::palette;
  namespace variation = sand::variation;
  Random random(0x6D726F34);
  for (int current = 0; current < palette::kCount; ++current) {
    int counts[palette::kCount] = {};
    for (int trial = 0; trial < 5000 * (palette::kCount - 1); ++trial) {
      const uint8_t next = variation::samplePalette(random, static_cast<uint8_t>(current));
      assert(next < palette::kCount && next != current);
      ++counts[next];
    }
    for (int choice = 0; choice < palette::kCount; ++choice)
      assert(choice == current ? counts[choice] == 0 : counts[choice] > 4600 && counts[choice] < 5400);
  }
  assert(variation::samplePalette(random, 255) < palette::kCount);
  StuckRandom stuck;
  assert(variation::samplePalette(stuck, 0) == 1 && stuck.calls <= 64);

  uint8_t slots[sand::config::kMaxParticleCount + 1];
  for (int count = 0; count <= sand::config::kMaxParticleCount; ++count) {
    memset(slots, 0xEE, sizeof(slots));
    variation::assignColorSlots(slots, count, random);
    int counts[palette::kForegroundCount] = {};
    for (int dot = 0; dot < count; ++dot) {
      assert(slots[dot] < palette::kForegroundCount);
      ++counts[slots[dot]];
    }
    for (int color = 0; color < palette::kForegroundCount; ++color) {
      assert(counts[color] >= count / palette::kForegroundCount);
      assert(counts[color] <= (count + palette::kForegroundCount - 1) / palette::kForegroundCount);
    }
    for (int dot = count; dot <= sand::config::kMaxParticleCount; ++dot) assert(slots[dot] == 0xEE);
  }
  memset(slots, 0xEE, sizeof(slots));
  variation::assignColorSlots(slots, -1, random);
  variation::assignColorSlots(slots, sand::config::kMaxParticleCount + 1, random);
  variation::assignColorSlots(nullptr, 10, random);
  for (uint8_t slot : slots) assert(slot == 0xEE);
  const unsigned before = stuck.calls;
  variation::assignColorSlots(slots, sand::config::kMaxParticleCount, stuck);
  assert(stuck.calls - before <= 64 * (sand::config::kMaxParticleCount - 1));
  puts("Every swatch is reachable with uniform alternative selection; five foreground slots remain balanced and bounded.");
}

int main() {
  catalogAndCoverage();
  animatedRepaints();
  seedMotionAndHiddenUpdates();
  timerWrapAndPaletteChange();
  pausedAndLateClocks();
  randomPaletteAndBalancedColors();
}
