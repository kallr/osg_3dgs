#ifndef SPARK_PACKED_ENCODING_H
#define SPARK_PACKED_ENCODING_H

#include <cstdint>
#include <cmath>
#include <algorithm>
#include <cstring>

// ============================================================================
// Spark PackedSplat / ExtSplat encoding utilities
// Based on Spark TypeScript/Rust reference implementation
// ============================================================================

namespace SparkPacked
{

// ---- Constants (from Spark defines.ts) ----

constexpr float LN_SCALE_MIN = -12.0f;
constexpr float LN_SCALE_MAX = 9.0f;
constexpr float SCALE_ZERO = 2.938735877055719e-13f; // exp(-30.0)

// ---- Half-float (f16) utilities ----

inline uint16_t floatToHalf(float f)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof(float));
    uint32_t sign = (bits >> 31) & 0x1;
    uint32_t exp = (bits >> 23) & 0xff;
    uint32_t frac = bits & 0x7fffff;
    uint16_t halfSign = (uint16_t)(sign << 15);
    if (exp == 0xff)
    {
        // Preserve the sign and available payload, and quiet signaling NaNs.
        if (frac != 0) return halfSign | 0x7c00 | 0x0200 | (uint16_t)(frac >> 13);
        return halfSign | 0x7c00; // Infinity
    }
    int32_t newExp = (int32_t)exp - 127 + 15;
    if (newExp >= 0x1f) return halfSign | 0x7c00; // Overflow -> Infinity

    // Round discarded bits to nearest, resolving exact midpoints to an even
    // low bit. This also carries a rounded fraction into the next exponent.
    auto roundShift = [](uint32_t value, uint32_t shift) {
        uint32_t rounded = value >> shift;
        uint32_t remainder = value & ((1u << shift) - 1u);
        uint32_t midpoint = 1u << (shift - 1u);
        return rounded + (remainder > midpoint || (remainder == midpoint && (rounded & 1u)));
    };
    if (newExp <= 0)
    {
        if (newExp < -10) return halfSign; // Below the zero/min-subnormal midpoint
        uint16_t subFrac = (uint16_t)roundShift(frac | 0x800000u, (uint32_t)(14 - newExp));
        return halfSign | subFrac; // Rounding may produce the smallest normal.
    }
    uint16_t halfFrac = (uint16_t)roundShift(frac, 13);
    return halfSign | (uint16_t)((newExp << 10) + halfFrac); // Includes rounded overflow
}

inline float halfToFloat(uint16_t h)
{
    uint32_t sign = (h >> 15) & 0x1;
    uint32_t exp = (h >> 10) & 0x1f;
    uint32_t frac = h & 0x3ff;
    uint32_t f32bits;
    if (exp == 0)
    {
        if (frac == 0)
        {
            f32bits = sign << 31; // Zero
        }
        else
        {
            // Subnormal: normalize
            uint32_t mant = frac;
            int32_t e = -14;
            while ((mant & 0x400) == 0) { mant <<= 1; e--; }
            mant &= 0x3ff;
            uint32_t newExp = (uint32_t)(e + 127);
            uint32_t newFrac = mant << 13;
            f32bits = (sign << 31) | (newExp << 23) | newFrac;
        }
    }
    else if (exp == 0x1f)
    {
        f32bits = frac == 0
            ? (sign << 31) | 0x7f800000 // Infinity
            : (sign << 31) | 0x7fc00000; // NaN
    }
    else
    {
        uint32_t newExp = exp - 15 + 127;
        uint32_t newFrac = frac << 13;
        f32bits = (sign << 31) | (newExp << 23) | newFrac;
    }
    float result;
    memcpy(&result, &f32bits, sizeof(float));
    return result;
}

// ---- Octahedral mapping helpers ----

// Folded octahedral mapping: axis direction -> (u, v) in [-1, 1]
inline void octEncode(float ax, float ay, float az, float& outU, float& outV)
{
    float sum = std::fabs(ax) + std::fabs(ay) + std::fabs(az);
    float px = ax / sum;
    float py = ay / sum;
    if (az < 0.0f)
    {
        float tmp = px;
        px = (1.0f - std::fabs(py)) * (px >= 0.0f ? 1.0f : -1.0f);
        py = (1.0f - std::fabs(tmp)) * (py >= 0.0f ? 1.0f : -1.0f);
    }
    outU = px;
    outV = py;
}

// Folded octahedral mapping: (u, v) in [-1, 1] -> axis direction
inline void octDecode(float u, float v, float& outX, float& outY, float& outZ)
{
    float z = 1.0f - (std::fabs(u) + std::fabs(v));
    float t = std::max(-z, 0.0f);
    float fx = u + (u >= 0.0f ? -t : t);
    float fy = v + (v >= 0.0f ? -t : t);
    float len = std::sqrt(fx * fx + fy * fy + z * z);
    if (len < 1e-6f) { outX = 1.0f; outY = 0.0f; outZ = 0.0f; return; }
    outX = fx / len;
    outY = fy / len;
    outZ = z / len;
}

// ---- OctXy88R8 quaternion encoding (24-bit) ----
// Layout: bits 0-7 = quantU, bits 8-15 = quantV, bits 16-23 = angleInt

inline uint32_t encodeQuatOctXy88R8(float qx, float qy, float qz, float qw)
{
    // Normalize and ensure w >= 0
    float len = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (len < 1e-12f) return 0;
    float nx = qx / len, ny = qy / len, nz = qz / len, nw = qw / len;
    if (nw < 0.0f) { nx = -nx; ny = -ny; nz = -nz; nw = -nw; }

    // Axis-angle
    float theta = 2.0f * std::acos(std::min(nw, 1.0f));
    float s = std::sin(theta * 0.5f);
    float ax, ay, az;
    if (s < 1e-6f) { ax = 1.0f; ay = 0.0f; az = 0.0f; }
    else { ax = nx / s; ay = ny / s; az = nz / s; }

    // Octahedral mapping
    float ou, ov;
    octEncode(ax, ay, az, ou, ov);

    uint8_t quantU = (uint8_t)std::round((ou * 0.5f + 0.5f) * 255.0f);
    uint8_t quantV = (uint8_t)std::round((ov * 0.5f + 0.5f) * 255.0f);
    uint8_t angleInt = (uint8_t)std::round(theta * (255.0f / 3.141592653589793f));

    return (uint32_t)angleInt << 16 | (uint32_t)quantV << 8 | (uint32_t)quantU;
}

inline void decodeQuatOctXy88R8(uint32_t encoded, float& outX, float& outY, float& outZ, float& outW)
{
    uint8_t quantU = encoded & 0xff;
    uint8_t quantV = (encoded >> 8) & 0xff;
    uint8_t angleInt = (encoded >> 16) & 0xff;

    float uf = quantU / 255.0f;
    float vf = quantV / 255.0f;
    float fx = (uf - 0.5f) * 2.0f;
    float fy = (vf - 0.5f) * 2.0f;

    float ax, ay, az;
    octDecode(fx, fy, ax, ay, az);

    float theta = (angleInt / 255.0f) * 3.141592653589793f;
    float halfTheta = theta * 0.5f;
    float s = std::sin(halfTheta);
    outW = std::cos(halfTheta);
    outX = ax * s;
    outY = ay * s;
    outZ = az * s;
}

// ---- OctXy1010R12 quaternion encoding (32-bit) ----
// Layout: bits 0-9 = quantU, bits 10-19 = quantV, bits 20-31 = angleInt

inline uint32_t encodeQuatOctXy1010R12(float qx, float qy, float qz, float qw)
{
    float len = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (len < 1e-12f) return 0;
    float nx = qx / len, ny = qy / len, nz = qz / len, nw = qw / len;
    if (nw < 0.0f) { nx = -nx; ny = -ny; nz = -nz; nw = -nw; }

    float theta = 2.0f * std::acos(std::min(nw, 1.0f));
    float s = std::sin(theta * 0.5f);
    float ax, ay, az;
    if (s < 1e-6f) { ax = 1.0f; ay = 0.0f; az = 0.0f; }
    else { ax = nx / s; ay = ny / s; az = nz / s; }

    float ou, ov;
    octEncode(ax, ay, az, ou, ov);

    uint32_t quantU = (uint32_t)std::round(std::clamp((ou * 0.5f + 0.5f) * 1023.0f, 0.0f, 1023.0f));
    uint32_t quantV = (uint32_t)std::round(std::clamp((ov * 0.5f + 0.5f) * 1023.0f, 0.0f, 1023.0f));
    uint32_t angleInt = (uint32_t)std::round(std::clamp(theta / 3.141592653589793f * 4095.0f, 0.0f, 4095.0f));

    return (angleInt << 20) | (quantV << 10) | quantU;
}

inline void decodeQuatOctXy1010R12(uint32_t encoded, float& outX, float& outY, float& outZ, float& outW)
{
    uint32_t quantU = encoded & 0x3ff;
    uint32_t quantV = (encoded >> 10) & 0x3ff;
    uint32_t angleInt = (encoded >> 20) & 0xfff;

    float fx = quantU / 1023.0f * 2.0f - 1.0f;
    float fy = quantV / 1023.0f * 2.0f - 1.0f;

    float ax, ay, az;
    octDecode(fx, fy, ax, ay, az);

    float theta = (angleInt / 4095.0f) * 3.141592653589793f;
    float halfTheta = theta * 0.5f;
    float s = std::sin(halfTheta);
    outW = std::cos(halfTheta);
    outX = ax * s;
    outY = ay * s;
    outZ = az * s;
}

// ---- Ln0R8 scale encoding (8-bit logarithmic quantization) ----

inline uint8_t encodeScaleLn0R8(float scale, float lnMin = LN_SCALE_MIN, float lnMax = LN_SCALE_MAX)
{
    if (scale <= SCALE_ZERO) return 0;
    float n = (std::log(scale) - lnMin) / (lnMax - lnMin);
    return (uint8_t)(1 + (uint32_t)std::round(std::clamp(n, 0.0f, 1.0f) * 254.0f));
}

inline float decodeScaleLn0R8(uint8_t encoded, float lnMin = LN_SCALE_MIN, float lnMax = LN_SCALE_MAX)
{
    if (encoded == 0) return 0.0f;
    float lnScaleScale = (lnMax - lnMin) / 254.0f;
    return std::exp(lnMin + (float)(encoded - 1) * lnScaleScale);
}

// ---- LnF16 scale encoding (16-bit logarithmic) ----

inline uint16_t encodeScaleLnF16(float scale)
{
    return floatToHalf(std::log(std::max(scale, 1e-30f)));
}

inline float decodeScaleLnF16(uint16_t encoded)
{
    return std::exp(halfToFloat(encoded));
}

// ---- PackedSplat (16 bytes = 4 x uint32) ----
// Layout:
//   word0: R(8) | G(8) | B(8) | A(8)
//   word1: centerX(f16) | centerY(f16)
//   word2: centerZ(f16) | quatU(8) | quatV(8)
//   word3: scaleX(ln0r8) | scaleY(ln0r8) | scaleZ(ln0r8) | quatR(8)

struct PackedSplat
{
    uint32_t words[4];

    void encode(float cx, float cy, float cz,
                float sx, float sy, float sz,
                float qx, float qy, float qz, float qw,
                float r, float g, float b, float a,
                float rgbMin = 0.0f, float rgbMax = 1.0f,
                float lnMin = LN_SCALE_MIN, float lnMax = LN_SCALE_MAX)
    {
        // RGB: 8-bit linear quantization within [rgbMin, rgbMax]
        float rgbRange = rgbMax - rgbMin;
        float invRange = rgbRange > 1e-6f ? 1.0f / rgbRange : 1.0f;
        uint8_t uR = (uint8_t)std::round(std::clamp((r - rgbMin) * invRange, 0.0f, 1.0f) * 255.0f);
        uint8_t uG = (uint8_t)std::round(std::clamp((g - rgbMin) * invRange, 0.0f, 1.0f) * 255.0f);
        uint8_t uB = (uint8_t)std::round(std::clamp((b - rgbMin) * invRange, 0.0f, 1.0f) * 255.0f);
        // Spark uses 127.5 for alpha to allow up to 2.0 opacity in PackedSplat
        uint8_t uA = (uint8_t)std::round(std::clamp(a * 0.5f, 0.0f, 1.0f) * 255.0f);

        words[0] = (uint32_t)uR | ((uint32_t)uG << 8) | ((uint32_t)uB << 16) | ((uint32_t)uA << 24);

        // Center: half-float
        uint16_t hCX = floatToHalf(cx);
        uint16_t hCY = floatToHalf(cy);
        uint16_t hCZ = floatToHalf(cz);
        words[1] = (uint32_t)hCX | ((uint32_t)hCY << 16);
        words[2] = (uint32_t)hCZ;

        // Quaternion: OctXy88R8 (24-bit)
        uint32_t quatEncoded = encodeQuatOctXy88R8(qx, qy, qz, qw);
        uint8_t quatU = quatEncoded & 0xff;
        uint8_t quatV = (quatEncoded >> 8) & 0xff;
        uint8_t quatR = (quatEncoded >> 16) & 0xff;
        words[2] |= (uint32_t)quatU << 16 | (uint32_t)quatV << 24;
        words[3] = (uint32_t)encodeScaleLn0R8(sx, lnMin, lnMax)
                  | ((uint32_t)encodeScaleLn0R8(sy, lnMin, lnMax) << 8)
                  | ((uint32_t)encodeScaleLn0R8(sz, lnMin, lnMax) << 16)
                  | ((uint32_t)quatR << 24);
    }

    void decode(float& cx, float& cy, float& cz,
                float& sx, float& sy, float& sz,
                float& qx, float& qy, float& qz, float& qw,
                float& r, float& g, float& b, float& a,
                float rgbMin = 0.0f, float rgbMax = 1.0f,
                float lnMin = LN_SCALE_MIN, float lnMax = LN_SCALE_MAX) const
    {
        // RGB
        float rgbRange = rgbMax - rgbMin;
        r = rgbMin + ((words[0] & 0xff) / 255.0f) * rgbRange;
        g = rgbMin + (((words[0] >> 8) & 0xff) / 255.0f) * rgbRange;
        b = rgbMin + (((words[0] >> 16) & 0xff) / 255.0f) * rgbRange;
        a = ((words[0] >> 24) & 0xff) / 255.0f * 2.0f;

        // Center
        cx = halfToFloat(words[1] & 0xffff);
        cy = halfToFloat((words[1] >> 16) & 0xffff);
        cz = halfToFloat(words[2] & 0xffff);

        // Scale
        sx = decodeScaleLn0R8(words[3] & 0xff, lnMin, lnMax);
        sy = decodeScaleLn0R8((words[3] >> 8) & 0xff, lnMin, lnMax);
        sz = decodeScaleLn0R8((words[3] >> 16) & 0xff, lnMin, lnMax);

        // Quaternion
        uint32_t quatEncoded = ((words[2] >> 16) & 0xffff) | ((words[3] >> 8) & 0xff0000);
        decodeQuatOctXy88R8(quatEncoded, qx, qy, qz, qw);
    }
};

// ---- ExtSplat (32 bytes = 8 x uint32, two arrays of 4) ----
// Layout:
//   extA[0-2]: center xyz (f32), extA[3]: opacity (f16)
//   extB[0]: R(f16) | G(f16), extB[1]: B(f16) | lnScaleX(f16)
//   extB[2]: lnScaleY(f16) | lnScaleZ(f16), extB[3]: quat(OctXy1010R12)

struct ExtSplat
{
    uint32_t extA[4];
    uint32_t extB[4];

    void encode(float cx, float cy, float cz,
                float sx, float sy, float sz,
                float qx, float qy, float qz, float qw,
                float r, float g, float b, float opacity)
    {
        // Center: float32
        memcpy(&extA[0], &cx, sizeof(float));
        memcpy(&extA[1], &cy, sizeof(float));
        memcpy(&extA[2], &cz, sizeof(float));
        // Opacity: f16
        extA[3] = (uint32_t)floatToHalf(opacity);

        // RGB: f16 pairs
        extB[0] = (uint32_t)floatToHalf(r) | ((uint32_t)floatToHalf(g) << 16);
        extB[1] = (uint32_t)floatToHalf(b) | ((uint32_t)floatToHalf(std::log(std::max(sx, 1e-30f))) << 16);
        extB[2] = (uint32_t)floatToHalf(std::log(std::max(sy, 1e-30f))) | ((uint32_t)floatToHalf(std::log(std::max(sz, 1e-30f))) << 16);
        // Quaternion: OctXy1010R12
        extB[3] = encodeQuatOctXy1010R12(qx, qy, qz, qw);
    }

    void decode(float& cx, float& cy, float& cz,
                float& sx, float& sy, float& sz,
                float& qx, float& qy, float& qz, float& qw,
                float& r, float& g, float& b, float& opacity) const
    {
        // Center
        memcpy(&cx, &extA[0], sizeof(float));
        memcpy(&cy, &extA[1], sizeof(float));
        memcpy(&cz, &extA[2], sizeof(float));
        // Opacity
        opacity = halfToFloat((uint16_t)(extA[3] & 0xffff));

        // RGB
        r = halfToFloat((uint16_t)(extB[0] & 0xffff));
        g = halfToFloat((uint16_t)(extB[0] >> 16));
        b = halfToFloat((uint16_t)(extB[1] & 0xffff));

        // Scale (ln f16 -> exp)
        sx = std::exp(halfToFloat((uint16_t)(extB[1] >> 16)));
        sy = std::exp(halfToFloat((uint16_t)(extB[2] & 0xffff)));
        sz = std::exp(halfToFloat((uint16_t)(extB[2] >> 16)));

        // Quaternion
        decodeQuatOctXy1010R12(extB[3], qx, qy, qz, qw);
    }
};

} // namespace SparkPacked

#endif
