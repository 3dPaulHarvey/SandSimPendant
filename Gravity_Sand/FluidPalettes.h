#ifndef GRAVITY_SAND_FLUID_PALETTES_H
#define GRAVITY_SAND_FLUID_PALETTES_H

#include <stdint.h>

// User-selected sRGB swatches, in catalog order. Keep the complete metadata in
// assets/color-swatches.json; RGB565 conversion is the only display adaptation.
namespace sand {
namespace palette {
constexpr int kCount = 21;
constexpr int kForegroundCount = 5;
constexpr int kBackgroundCount = 3;

constexpr uint16_t rgb565(uint32_t hex) {
  return static_cast<uint16_t>(((hex >> 8) & 0xF800) |
                               ((hex >> 5) & 0x07E0) | ((hex >> 3) & 0x001F));
}

struct Swatch {
  const char *id;
  const char *name;
  uint16_t foreground[kForegroundCount];
  uint16_t background[kBackgroundCount];
};

static const Swatch kSwatches[kCount] = {
  {"paracas-mantle", "Paracas mantle",
   {rgb565(0x233E38), rgb565(0xB54246), rgb565(0xDCAD4B), rgb565(0x506F83), rgb565(0xE8BD92)},
   {rgb565(0x381E2D), rgb565(0x203044), rgb565(0x39331D)}},
  {"the-great-wave", "The Great Wave",
   {rgb565(0x173956), rgb565(0x276897), rgb565(0x84AAB4), rgb565(0xD4B891), rgb565(0xF6EFDF)},
   {rgb565(0x101D2A), rgb565(0x263D3C), rgb565(0x3A302B)}},
  {"water-lilies", "Water Lilies",
   {rgb565(0x546E64), rgb565(0x8DA880), rgb565(0x86A5AE), rgb565(0xA19ABB), rgb565(0xE4B3B7)},
   {rgb565(0x1B3434), rgb565(0x32283F), rgb565(0x263547)}},
  {"composition-with-red-blue-and-yellow", "Composition with Red, Blue and Yellow",
   {rgb565(0xDD332A), rgb565(0x174999), rgb565(0xF0C733), rgb565(0xF3F0E5), rgb565(0x20201F)},
   {rgb565(0x31353B), rgb565(0x1C2940), rgb565(0x3E2928)}},
  {"the-snail", "The Snail",
   {rgb565(0xF27A36), rgb565(0x9B69AF), rgb565(0x2560A5), rgb565(0x3E9861), rgb565(0xE8C945)},
   {rgb565(0x242343), rgb565(0x183A32), rgb565(0x412534)}},
  {"dexters-laboratory", "Dexter’s Laboratory",
   {rgb565(0xF87927), rgb565(0x73479D), rgb565(0x9FDBED), rgb565(0xF5F2EA), rgb565(0x222135)},
   {rgb565(0x30213D), rgb565(0x183448), rgb565(0x373044)}},
  {"samurai-jack", "Samurai Jack",
   {rgb565(0x111218), rgb565(0xF1E8D7), rgb565(0xE74334), rgb565(0x60AD58), rgb565(0x688BA4)},
   {rgb565(0x421D2B), rgb565(0x263E50), rgb565(0x382B4A)}},
  {"quattro-rally-tricolor", "Quattro rally tricolor",
   {rgb565(0x191A1B), rgb565(0xDB3027), rgb565(0xFFCC19), rgb565(0xF3F0E6), rgb565(0xA8AFB1)},
   {rgb565(0x292D32), rgb565(0x402024), rgb565(0x162E32)}},
  {"halo-combat-evolved", "Halo: Combat Evolved",
   {rgb565(0x657643), rgb565(0xE8A537), rgb565(0x785D9E), rgb565(0x6BC2D8), rgb565(0x526574)},
   {rgb565(0x1E2217), rgb565(0x251F2E), rgb565(0x171B1E)}},
  {"super-mario-bros", "Super Mario Bros.",
   {rgb565(0xB53120), rgb565(0x5C94FC), rgb565(0x00A800), rgb565(0xC84C0C), rgb565(0xFCFCF4)},
   {rgb565(0x1F2B41), rgb565(0x0A320A), rgb565(0x2F1609)}},
  {"doom", "DOOM",
   {rgb565(0x546D3F), rgb565(0xF47724), rgb565(0xAC2D32), rgb565(0xD8A04D), rgb565(0x343C79)},
   {rgb565(0x1A1F15), rgb565(0x2F1112), rgb565(0x0F101D)}},
  {"minecraft", "Minecraft",
   {rgb565(0x719B49), rgb565(0x835637), rgb565(0x858585), rgb565(0x3F76E4), rgb565(0x56D8CB)},
   {rgb565(0x232C1A), rgb565(0x2B2B2B), rgb565(0x152036)}},
  {"grand-theft-auto", "Grand Theft Auto",
   {rgb565(0x464650), rgb565(0xE9C333), rgb565(0xB53A37), rgb565(0xB8B4A6), rgb565(0x5E773D)},
   {rgb565(0x141417), rgb565(0x321514), rgb565(0x1A1F14)}},
  {"the-legend-of-zelda", "The Legend of Zelda",
   {rgb565(0x328738), rgb565(0xF3CF99), rgb565(0x477BCB), rgb565(0xD94330), rgb565(0xA65D2D)},
   {rgb565(0x132514), rgb565(0x1B273A), rgb565(0x291A10)}},
  {"candy-crush-soda-saga", "Candy Crush Soda Saga",
   {rgb565(0x9352D8), rgb565(0xF389BC), rgb565(0x73CFF0), rgb565(0x9BD741), rgb565(0xF5E5FC)},
   {rgb565(0x281A36), rgb565(0x472E3A), rgb565(0x24373D)}},
  {"mortal-kombat", "Mortal Kombat",
   {rgb565(0xE5AF32), rgb565(0x236AB6), rgb565(0xBD2729), rgb565(0x212125), rgb565(0xA3A6A2)},
   {rgb565(0x3F3318), rgb565(0x102133), rgb565(0x2B0D0D)}},
  {"ultraviolet", "Ultraviolet",
   {rgb565(0xA63BF7), rgb565(0xB762F8), rgb565(0xC989FA), rgb565(0xDBB1FC), rgb565(0xEFDDFE)},
   {rgb565(0x0B0410), rgb565(0x190925), rgb565(0x270E39)}},
  {"glacier-cyan", "Glacier cyan",
   {rgb565(0x05C1D6), rgb565(0x1FE3F9), rgb565(0x5BEBFB), rgb565(0x9CF3FC), rgb565(0xD7FAFE)},
   {rgb565(0x040F10), rgb565(0x092225), rgb565(0x0E3539)}},
  {"solar-gold", "Solar gold",
   {rgb565(0xF0B905), rgb565(0xFBCC32), rgb565(0xFCDA69), rgb565(0xFDE69B), rgb565(0xFEF5D7)},
   {rgb565(0x100D04), rgb565(0x251E09), rgb565(0x392F0E)}},
  {"emerald", "Emerald",
   {rgb565(0x0BD06E), rgb565(0x25F48C), rgb565(0x5FF7AB), rgb565(0x9EFACC), rgb565(0xD8FDEB)},
   {rgb565(0x04100A), rgb565(0x092517), rgb565(0x0E3924)}},
  {"crimson", "Crimson",
   {rgb565(0xF62323), rgb565(0xF84F4F), rgb565(0xF97B7B), rgb565(0xFBA7A7), rgb565(0xFDD8D8)},
   {rgb565(0x100404), rgb565(0x250909), rgb565(0x390E0E)}},
};

inline const Swatch &get(uint8_t index) {
  return kSwatches[index < kCount ? index : 0];
}

} // namespace palette
} // namespace sand
#endif
