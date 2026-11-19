#version 430 core
layout (location = 0) in vec3 inPos;

uniform isamplerBuffer indexBuffer;
uniform samplerBuffer dataBuffer;
uniform samplerBuffer shBuffer;
uniform usamplerBuffer packedDataBuffer;  // RGBA32UI for packed/ext splats
uniform samplerBuffer covSplatBuffer;     // RGBA32F for CovSplat data (2 texels/splat: xxyyzz+xyxzyz)
uniform bool useGpuSort;
uniform bool hasSH;
uniform int globalShOrder;
uniform float supportRadiusScale;
uniform float maxScreenRadius;
uniform float minPixelRadius;
uniform float preBlurAmount;
uniform float blurAmount;
uniform bool enableBlurCompensation;
uniform bool sparkRadiusClamp;
uniform bool lodInflate;
uniform int usePackedEncoding;  // 0=legacy, 1=PackedSplat(16B), 2=ExtSplat(32B)
uniform bool enable2DGS;        // support scale=0 (2D Gaussian) splats
uniform bool isOrthographic;    // orthographic projection mode
uniform float focalDistance;     // depth-of-field: focus distance (>0 to enable)
uniform float apertureAngle;    // depth-of-field: aperture full-width angle (radians)
uniform bool enableCovSplats;   // use direct 3D covariance (xxyyzz+xyxzyz) instead of scale+quat
uniform float clipXY;           // NDC guard band half-width (default 2.0)
uniform int debugFlag;          // 0=off, 1=depth, 2=normal
uniform bool isColorPremultiplied; // true for PLY, false for RAD

// Splat edit uniforms
uniform bool enableSplatEdit;
uniform usampler2D sdfTexture;
uniform int numSdfs;
uniform int numEdits;
uniform vec4 editData[8]; // max 8 edits, packed as float bits of uint32

// Splat skinning uniforms
uniform bool enableSkinning;
uniform int skinningMode;  // 0=DQ, 1=LB
uniform usamplerBuffer skinSampler;
uniform samplerBuffer boneSampler;
uniform int skinningNumSplats;
uniform int skinningNumBones;

uniform mat4 view;
uniform mat4 proj;
uniform vec2 viewport_size;

out vec4 PassColor;
out vec4 PassCov2Dinv;
out vec2 PassCenter2D;
out vec2 PassSplatUv;
flat out int PassIs2DGS;
out float PassAdjustedStdDev;
out vec3 vEyePos;

layout(std430, binding = 2) readonly buffer SortIndexBuffer
{
    int sortIndices[];
};

// ============================================================================
// Packed encoding decode utilities (matching SparkPackedEncoding.h)
// ============================================================================

const float LN_SCALE_MIN = -12.0;
const float LN_SCALE_MAX = 9.0;
const float LN_SCALE_SCALE = (LN_SCALE_MAX - LN_SCALE_MIN) / 254.0;

// Half-float decode
float fromHalf(uint h)
{
    uint sign = (h >> 15) & 1u;
    uint exp = (h >> 10) & 0x1fu;
    uint frac = h & 0x3ffu;
    uint f32bits;
    if (exp == 0u) {
        if (frac == 0u) {
            f32bits = sign << 31u;
        } else {
            uint mant = frac;
            int e = -14;
            while ((mant & 0x400u) == 0u) { mant <<= 1u; e--; }
            mant &= 0x3ffu;
            f32bits = (sign << 31u) | (uint(e + 127) << 23u) | (mant << 13u);
        }
    } else if (exp == 0x1fu) {
        f32bits = (frac == 0u) ? ((sign << 31u) | 0x7f800000u) : ((sign << 31u) | 0x7fc00000u);
    } else {
        f32bits = (sign << 31u) | ((exp - 15u + 127u) << 23u) | (frac << 13u);
    }
    return uintBitsToFloat(f32bits);
}

// Octahedral mapping decode: (u, v) in [-1,1] -> axis direction
vec3 octDecode(float u, float v)
{
    float z = 1.0 - (abs(u) + abs(v));
    float t = max(-z, 0.0);
    float fx = u + (u >= 0.0 ? -t : t);
    float fy = v + (v >= 0.0 ? -t : t);
    vec3 axis = vec3(fx, fy, z);
    float len = length(axis);
    return len < 1e-6 ? vec3(1.0, 0.0, 0.0) : axis / len;
}

// Decode OctXy88R8 quaternion (24-bit packed in lower 24 bits of a uint)
vec4 decodeQuatOctXy88R8(uint encoded)
{
    uint quantU = encoded & 0xffu;
    uint quantV = (encoded >> 8u) & 0xffu;
    uint angleInt = (encoded >> 16u) & 0xffu;

    float uf = float(quantU) / 255.0;
    float vf = float(quantV) / 255.0;
    float fx = (uf - 0.5) * 2.0;
    float fy = (vf - 0.5) * 2.0;

    vec3 axis = octDecode(fx, fy);
    float theta = float(angleInt) / 255.0 * 3.141592653589793;
    float halfTheta = theta * 0.5;
    float s = sin(halfTheta);
    return vec4(axis * s, cos(halfTheta));
}

// Decode OctXy1010R12 quaternion (32-bit)
vec4 decodeQuatOctXy1010R12(uint encoded)
{
    uint quantU = encoded & 0x3ffu;
    uint quantV = (encoded >> 10u) & 0x3ffu;
    uint angleInt = (encoded >> 20u) & 0xfffu;

    float fx = float(quantU) / 1023.0 * 2.0 - 1.0;
    float fy = float(quantV) / 1023.0 * 2.0 - 1.0;

    vec3 axis = octDecode(fx, fy);
    float theta = float(angleInt) / 4095.0 * 3.141592653589793;
    float halfTheta = theta * 0.5;
    float s = sin(halfTheta);
    return vec4(axis * s, cos(halfTheta));
}

// Decode Ln0R8 scale
float decodeScaleLn0R8(uint encoded)
{
    if (encoded == 0u) return 0.0;
    return exp(LN_SCALE_MIN + float(encoded - 1u) * LN_SCALE_SCALE);
}

// ============================================================================
// PackedSplat decode (16 bytes = 1 texel from RGBA32UI)
// Layout:
//   word0: R(8) | G(8) | B(8) | A(8)
//   word1: centerX(f16) | centerY(f16)
//   word2: centerZ(f16) | quatU(8) | quatV(8)
//   word3: scaleX(ln0r8) | scaleY(ln0r8) | scaleZ(ln0r8) | quatR(8)
// ============================================================================
void decodePackedSplat(int sortedIdx, out vec3 position, out vec3 scale,
                       out vec4 quat, out vec3 color, out float alpha, out int shOrder)
{
    uvec4 w = texelFetch(packedDataBuffer, sortedIdx);

    // RGB
    color = vec3(float(w.x & 0xffu), float((w.x >> 8u) & 0xffu), float((w.x >> 16u) & 0xffu)) / 255.0;
    alpha = float((w.x >> 24u) & 0xffu) / 255.0;

    // Center
    position = vec3(
        fromHalf(w.y & 0xffffu),
        fromHalf((w.y >> 16u) & 0xffffu),
        fromHalf(w.z & 0xffffu)
    );

    // Scale
    scale = vec3(
        decodeScaleLn0R8(w.w & 0xffu),
        decodeScaleLn0R8((w.w >> 8u) & 0xffu),
        decodeScaleLn0R8((w.w >> 16u) & 0xffu)
    );

    // Quaternion: reassemble 24-bit OctXy88R8 from split fields
    uint quatU = (w.z >> 16u) & 0xffu;
    uint quatV = (w.z >> 24u) & 0xffu;
    uint quatR = (w.w >> 24u) & 0xffu;
    uint quatEncoded = quatR << 16u | quatV << 8u | quatU;
    quat = decodeQuatOctXy88R8(quatEncoded);

    shOrder = hasSH ? globalShOrder : 0;
}

// ============================================================================
// ExtSplat decode (32 bytes = 2 texels from RGBA32UI)
// Layout:
//   extA (texel 0): center.xyz(f32), opacity(f16)
//   extB (texel 1): R(f16)|G(f16), B(f16)|lnScaleX(f16), lnScaleY(f16)|lnScaleZ(f16), quat(OctXy1010R12)
// ============================================================================
void decodeExtSplat(int sortedIdx, out vec3 position, out vec3 scale,
                    out vec4 quat, out vec3 color, out float alpha, out int shOrder)
{
    uvec4 extA = texelFetch(packedDataBuffer, sortedIdx * 2);
    uvec4 extB = texelFetch(packedDataBuffer, sortedIdx * 2 + 1);

    // Center (float32)
    position = vec3(
        uintBitsToFloat(extA.x),
        uintBitsToFloat(extA.y),
        uintBitsToFloat(extA.z)
    );

    // Opacity (f16)
    alpha = fromHalf(extA.w & 0xffffu);

    // RGB (f16 pairs)
    color = vec3(
        fromHalf(extB.x & 0xffffu),
        fromHalf((extB.x >> 16u) & 0xffffu),
        fromHalf(extB.y & 0xffffu)
    );

    // Scale (ln f16 -> exp)
    scale = vec3(
        exp(fromHalf((extB.y >> 16u) & 0xffffu)),
        exp(fromHalf(extB.z & 0xffffu)),
        exp(fromHalf((extB.z >> 16u) & 0xffffu))
    );

    // Quaternion (OctXy1010R12)
    quat = decodeQuatOctXy1010R12(extB.w);

    shOrder = hasSH ? globalShOrder : 0;
}

// ============================================================================
// Core math (unchanged)
// ============================================================================

// Compute rotation matrix from quaternion (x, y, z, w)
mat3 quatToRotation(vec4 q)
{
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float x2 = x + x, y2 = y + y, z2 = z + z;
    float xx = x * x2, xy = x * y2, xz = x * z2;
    float yy = y * y2, yz = y * z2, zz = z * z2;
    float wx = w * x2, wy = w * y2, wz = w * z2;
    return mat3(
        1.0 - (yy + zz), xy + wz, xz - wy,
        xy - wz, 1.0 - (xx + zz), yz + wx,
        xz + wy, yz - wx, 1.0 - (xx + yy)
    );
}

// Compute 3D covariance from scale and quaternion (R * S^2 * R^T)
mat3 computeCovariance(vec3 scale, vec4 quat)
{
    mat3 R = quatToRotation(quat);
    mat3 S2 = mat3(
        scale.x * scale.x, 0.0, 0.0,
        0.0, scale.y * scale.y, 0.0,
        0.0, 0.0, scale.z * scale.z
    );
    return R * S2 * transpose(R);
}

// Build 3D covariance from direct xxyyzz+xyxzyz representation and transform to view space
mat3 computeCovarianceDirect(vec3 xxyyzz, vec3 xyxzyz, mat3 viewBasis)
{
    mat3 cov = mat3(
        xxyyzz.x, xyxzyz.x, xyxzyz.y,
        xyxzyz.x, xxyyzz.y, xyxzyz.z,
        xyxzyz.y, xyxzyz.z, xxyyzz.z
    );
    return viewBasis * cov * transpose(viewBasis);
}

// SH constants
const float SH_C1 = 0.4886025119029199;
const float SH_C2[5] = float[5](
    1.0925484305920792, -1.0925484305920792,
    0.31539156525252005, -1.0925484305920792, 0.5462742152960396
);
const float SH_C3[7] = float[7](
    -0.5900435899266435, 2.890611442640554, -0.4570457994644658,
    0.3731763325901154, -0.4570457994644658, 1.445305721320277, -0.5900435899266435
);

vec3 evalSphericalHarmonics(vec3 dir, vec3 dc, int shBase, int shOrder)
{
    float x = dir.x, y = dir.y, z = dir.z;
    vec3 col = dc;

    float sh[45] = float[45](
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    // CPU uploads 15 coefficients per color channel, padded to 12 RGBA
    // texels per splat. A low SH order is not a contiguous texel prefix:
    // green starts at float 15 and blue at float 30 even for SH1.
    int coefficientCount = shOrder == 1 ? 3 : (shOrder == 2 ? 8 : 15);
    int lastTexel = -1;
    vec4 values = vec4(0.0);
    for (int channel = 0; channel < 3; ++channel) {
        int begin = channel * 15;
        int end = begin + coefficientCount;
        for (int texel = begin / 4; texel <= (end - 1) / 4; ++texel) {
            // Reuse a boundary texel shared by adjacent full-order channels.
            if (texel != lastTexel) {
                values = texelFetch(shBuffer, shBase + texel);
                lastTexel = texel;
            }
            for (int component = 0; component < 4; ++component) {
                int index = texel * 4 + component;
                if (index >= begin && index < end)
                    sh[index] = values[component];
            }
        }
    }

    col.r += SH_C1 * (-y * sh[0]  + z * sh[1]  - x * sh[2]);
    col.g += SH_C1 * (-y * sh[15] + z * sh[16] - x * sh[17]);
    col.b += SH_C1 * (-y * sh[30] + z * sh[31] - x * sh[32]);
    // SH radiance can exceed one. Preserve it until alpha blending; clipping
    // each translucent splat here loses highlights even in an LDR framebuffer.
    if (shOrder < 2) return max(col, vec3(0.0));

    float xx = x*x, yy = y*y, zz = z*z, xy = x*y, xz = x*z, yz = y*z;
    col.r += SH_C2[0]*xy*sh[3] + SH_C2[1]*yz*sh[4] + SH_C2[2]*(2*zz-xx-yy)*sh[5]
           + SH_C2[3]*xz*sh[6] + SH_C2[4]*(xx-yy)*sh[7];
    col.g += SH_C2[0]*xy*sh[18] + SH_C2[1]*yz*sh[19] + SH_C2[2]*(2*zz-xx-yy)*sh[20]
           + SH_C2[3]*xz*sh[21] + SH_C2[4]*(xx-yy)*sh[22];
    col.b += SH_C2[0]*xy*sh[33] + SH_C2[1]*yz*sh[34] + SH_C2[2]*(2*zz-xx-yy)*sh[35]
           + SH_C2[3]*xz*sh[36] + SH_C2[4]*(xx-yy)*sh[37];
    if (shOrder < 3) return max(col, vec3(0.0));

    col.r += SH_C3[0]*y*(3*xx-yy)*sh[8] + SH_C3[1]*xy*z*sh[9]
           + SH_C3[2]*y*(4*zz-xx-yy)*sh[10] + SH_C3[3]*z*(2*zz-3*xx-3*yy)*sh[11]
           + SH_C3[4]*x*(4*zz-xx-yy)*sh[12] + SH_C3[5]*z*(xx-yy)*sh[13]
           + SH_C3[6]*x*(xx-3*yy)*sh[14];
    col.g += SH_C3[0]*y*(3*xx-yy)*sh[23] + SH_C3[1]*xy*z*sh[24]
           + SH_C3[2]*y*(4*zz-xx-yy)*sh[25] + SH_C3[3]*z*(2*zz-3*xx-3*yy)*sh[26]
           + SH_C3[4]*x*(4*zz-xx-yy)*sh[27] + SH_C3[5]*z*(xx-yy)*sh[28]
           + SH_C3[6]*x*(xx-3*yy)*sh[29];
    col.b += SH_C3[0]*y*(3*xx-yy)*sh[38] + SH_C3[1]*xy*z*sh[39]
           + SH_C3[2]*y*(4*zz-xx-yy)*sh[40] + SH_C3[3]*z*(2*zz-3*xx-3*yy)*sh[41]
           + SH_C3[4]*x*(4*zz-xx-yy)*sh[42] + SH_C3[5]*z*(xx-yy)*sh[43]
           + SH_C3[6]*x*(xx-3*yy)*sh[44];

    return max(col, vec3(0.0));
}

uniform bool useSparkPagedPool;     // Flag to indicate if we are reading from the Spark Paged Pool

// Decode PackedSplat / ExtSplat from a page-table resolved physical index
void decodePackedSplatPhysical(int physicalIdx, out vec3 position, out vec3 scale,
                       out vec4 quat, out vec3 color, out float alpha, out int shOrder)
{
    uvec4 w = texelFetch(packedDataBuffer, physicalIdx);
    color = vec3(float(w.x & 0xffu), float((w.x >> 8u) & 0xffu), float((w.x >> 16u) & 0xffu)) / 255.0;
    alpha = float((w.x >> 24u) & 0xffu) / 255.0;
    position = vec3(fromHalf(w.y & 0xffffu), fromHalf((w.y >> 16u) & 0xffffu), fromHalf(w.z & 0xffffu));
    scale = vec3(decodeScaleLn0R8(w.w & 0xffu), decodeScaleLn0R8((w.w >> 8u) & 0xffu), decodeScaleLn0R8((w.w >> 16u) & 0xffu));
    uint quatU = (w.z >> 16u) & 0xffu;
    uint quatV = (w.z >> 24u) & 0xffu;
    uint quatR = (w.w >> 24u) & 0xffu;
    uint quatEncoded = quatR << 16u | quatV << 8u | quatU;
    quat = decodeQuatOctXy88R8(quatEncoded);
    shOrder = hasSH ? globalShOrder : 0;
}

void decodeExtSplatPhysical(int physicalIdx, out vec3 position, out vec3 scale,
                    out vec4 quat, out vec3 color, out float alpha, out int shOrder)
{
    uvec4 extA = texelFetch(packedDataBuffer, physicalIdx * 2);
    uvec4 extB = texelFetch(packedDataBuffer, physicalIdx * 2 + 1);
    position = vec3(uintBitsToFloat(extA.x), uintBitsToFloat(extA.y), uintBitsToFloat(extA.z));
    alpha = fromHalf(extA.w & 0xffffu);
    color = vec3(fromHalf(extB.x & 0xffffu), fromHalf((extB.x >> 16u) & 0xffffu), fromHalf(extB.y & 0xffffu));
    scale = vec3(exp(fromHalf((extB.y >> 16u) & 0xffffu)), exp(fromHalf(extB.z & 0xffffu)), exp(fromHalf((extB.z >> 16u) & 0xffffu)));
    quat = decodeQuatOctXy1010R12(extB.w);
    shOrder = hasSH ? globalShOrder : 0;
}

void main()
{
    PassIs2DGS = 0;
    PassSplatUv = vec2(0.0);
    PassCenter2D = vec2(0.0);
    PassCov2Dinv = vec4(1.0, 0.0, 0.0, 1.0);
    // Fetch sorted instance index
    int sortedIdx = useGpuSort ? sortIndices[gl_InstanceID] : texelFetch(indexBuffer, gl_InstanceID).x;
    if (sortedIdx < 0) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        PassColor = vec4(0.0);
        PassCov2Dinv = vec4(1.0, 0.0, 0.0, 1.0);
        PassCenter2D = vec2(0.0);
        PassAdjustedStdDev = 3.5;
        return;
    }

    // Decode splat data based on encoding mode
    vec3 position;
    vec3 scale;
    vec4 quat;
    vec3 color;
    float alpha;
    int shOrder;

    if (useSparkPagedPool) {
        // In Paged Pool mode, the index stored in the active buffer is already
        // the resolved physical index (pageIndex * pageSize + pageOffset).
        // It was resolved by the CPU during the sort pass.
        if (usePackedEncoding == 1) {
            decodePackedSplatPhysical(sortedIdx, position, scale, quat, color, alpha, shOrder);
            alpha *= 2.0;
        } else if (usePackedEncoding == 2) {
            decodeExtSplatPhysical(sortedIdx, position, scale, quat, color, alpha, shOrder);
        } else {
            // Legacy path with physical addressing
            int dataBase = sortedIdx * 4;
            vec4 posAlpha = texelFetch(dataBuffer, dataBase + 0);
            vec4 scaleR   = texelFetch(dataBuffer, dataBase + 1);
            vec4 quatXYZG = texelFetch(dataBuffer, dataBase + 2);
            vec4 quatWB   = texelFetch(dataBuffer, dataBase + 3);

            position = posAlpha.xyz;
            alpha    = posAlpha.w;
            scale    = scaleR.xyz;
            color    = vec3(scaleR.w, quatXYZG.w, quatWB.w);
            quat     = vec4(quatXYZG.xyz, quatWB.x);
            shOrder  = int(quatWB.y);
        }
    } else {
        if (usePackedEncoding == 1) {
            decodePackedSplat(sortedIdx, position, scale, quat, color, alpha, shOrder);
            alpha *= 2.0;
        } else if (usePackedEncoding == 2) {
            decodeExtSplat(sortedIdx, position, scale, quat, color, alpha, shOrder);
        } else {
            // Legacy path: 4x vec4 per splat
            int dataBase = sortedIdx * 4;
            vec4 posAlpha = texelFetch(dataBuffer, dataBase + 0);
            vec4 scaleR   = texelFetch(dataBuffer, dataBase + 1);
            vec4 quatXYZG = texelFetch(dataBuffer, dataBase + 2);
            vec4 quatWB   = texelFetch(dataBuffer, dataBase + 3);

            position = posAlpha.xyz;
            alpha    = posAlpha.w;
            scale    = scaleR.xyz;
            color    = vec3(scaleR.w, quatXYZG.w, quatWB.w);
            quat     = vec4(quatXYZG.xyz, quatWB.x);
            shOrder  = int(quatWB.y);
        }
    }

    // ------------------------------------------------------------------------
    // Spark LOD Inflation & Alpha stretching
    // ------------------------------------------------------------------------
    float adjustedStdDev = 2.8284271; // maxStdDev
    float covRescale = 1.0; // CovSplat LOD inflate rescale factor
    if (alpha > 1.0) {
        alpha = min(alpha * 4.0 - 3.0, 5.0);

        if (lodInflate) {
            float density = exp((alpha * alpha - 1.0) / 2.718281828459045);
            float rescale = pow(max(density, 1.0e-6), 1.0 / 3.0);
            scale *= rescale;
            covRescale = rescale * rescale; // cov = rescale^2 * cov
            alpha = 1.0;
        }

        adjustedStdDev += 0.7 * (alpha - 1.0);
    }

    // Apply skinning before view transform
    vec3 skinnedXxyyzz = vec3(0.0);
    vec3 skinnedXyxzyz = vec3(0.0);
    bool hasSkinnedCov = false;
    if (enableSkinning) {
        if (enableCovSplats) {
            // CovSplat path: read covariance, then skin
            vec4 covA = texelFetch(covSplatBuffer, sortedIdx * 2);
            vec4 covB = texelFetch(covSplatBuffer, sortedIdx * 2 + 1);
            skinnedXxyyzz = covA.xyz;
            skinnedXyxzyz = vec3(covA.w, covB.xy);
            if (skinningMode == 0) {
                applyCovSplatDQSkinning(skinSampler, boneSampler, skinningNumSplats, skinningNumBones,
                                        sortedIdx, position, skinnedXxyyzz, skinnedXyxzyz);
            } else {
                applyCovSplatLBSkinning(skinSampler, boneSampler, skinningNumSplats, skinningNumBones,
                                        sortedIdx, position, skinnedXxyyzz, skinnedXyxzyz);
            }
            hasSkinnedCov = true;
        } else {
            // Gsplat path: skin position + quaternion
            if (skinningMode == 0) {
                applyGsplatSkinningDQ(skinSampler, boneSampler, skinningNumSplats, skinningNumBones,
                                      sortedIdx, position, quat);
            }
            // LB mode for Gsplat is not directly supported; use DQ or CovSplat path
        }
    }

    // Apply Splat Edit displacement before view/projection so culling and
    // screen-space placement use the edited center position.  RGBA edits are
    // applied after SH evaluation so edited colors are not treated as SH DC.
    vec4 editRgbaScale = vec4(1.0);
    vec4 editRgbaOffset = vec4(0.0);
    if (enableSplatEdit) {
        for (int i = 0; i < numEdits && i < 8; ++i) {
            uvec4 packedEdit = uvec4(floatBitsToUint(editData[i].x),
                                     floatBitsToUint(editData[i].y),
                                     floatBitsToUint(editData[i].z),
                                     floatBitsToUint(editData[i].w));
            accumulatePackedRgbaDisplaceEdit(packedEdit, sdfTexture, numSdfs,
                                             position, editRgbaScale, editRgbaOffset);
        }
    }

    // Transform to eye space
    vec4 eyePos = view * vec4(position, 1.0);
    vEyePos = eyePos.xyz;

    // Frustum culling: discard splats behind camera
    if (eyePos.z > 0.0) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    // 2DGS path: when enable2DGS and any scale axis is zero, use direct 2D offset
    bvec3 zeroScales = equal(scale, vec3(0.0));
    if (enable2DGS && any(zeroScales)) {
        PassIs2DGS = 1;
        if (all(zeroScales)) {
            // All scales zero — degenerate, discard
            gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
            return;
        }
        // Construct 2D offset in the two non-zero axes
        vec3 offset;
        if (zeroScales.z) {
            offset = vec3(inPos.xy * scale.xy, 0.0);
        } else if (zeroScales.y) {
            offset = vec3(inPos.x * scale.x, 0.0, inPos.y * scale.z);
        } else {
            offset = vec3(0.0, inPos.xy * scale.yz);
        }
        // Rotate offset by quaternion and add to view-space center
        mat3 R = quatToRotation(quat);
        vec3 viewPos = eyePos.xyz + R * offset;
        gl_Position = proj * vec4(viewPos, 1.0);

        // Compute center in screen coordinates
        vec3 ndc = gl_Position.xyz / gl_Position.w;
        vec2 center = vec2(0.5 * (viewport_size.x + ndc.x * viewport_size.x),
                           0.5 * (viewport_size.y + ndc.y * viewport_size.y));
        PassCenter2D = center;
        PassCov2Dinv = vec4(1.0, 0.0, 0.0, 1.0);  // identity covariance for 2DGS
        vec3 baseColor = color;
        if (isColorPremultiplied) {
            baseColor = color / max(min(alpha, 1.0), 1.0e-6);
        }
        if (enableSplatEdit) {
            vec4 editRgba = vec4(baseColor, alpha) * editRgbaScale + editRgbaOffset;
            baseColor = editRgba.rgb;
            alpha = editRgba.a;
        }
        PassColor = vec4(baseColor, alpha);
        PassAdjustedStdDev = 2.8284271;
        return;
    }

    // Compute 3D covariance on GPU
    mat3 sigma;
    mat3 W = mat3(view);

    if (enableCovSplats) {
        // CovSplat path: use direct 3D covariance
        vec3 xxyyzz, xyxzyz;
        if (hasSkinnedCov) {
            xxyyzz = skinnedXxyyzz;
            xyxzyz = skinnedXyxzyz;
        } else {
            vec4 covA = texelFetch(covSplatBuffer, sortedIdx * 2);
            vec4 covB = texelFetch(covSplatBuffer, sortedIdx * 2 + 1);
            xxyyzz = covA.xyz;
            xyxzyz = vec3(covA.w, covB.xy);
        }
        // Degenerate check
        if (all(equal(xxyyzz, vec3(0.0))) && all(equal(xyxzyz, vec3(0.0)))) {
            gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
            return;
        }
        sigma = computeCovarianceDirect(xxyyzz, xyxzyz, W);
        if (covRescale != 1.0) {
            sigma *= covRescale; // LOD inflate: cov = rescale^2 * cov
        }
    } else {
        sigma = computeCovariance(scale, quat);
    }

    // Jacobian of projection (perspective or orthographic)
    float focal_x = proj[0][0] * viewport_size.x * 0.5;
    float focal_y = proj[1][1] * viewport_size.y * 0.5;

    mat3 J;
    if (isOrthographic) {
        J = mat3(
            focal_x, 0.0, 0.0,
            0.0, focal_y, 0.0,
            0.0, 0.0, 0.0
        );
    } else {
        float invZ = 1.0 / eyePos.z;
        vec2 J1 = vec2(focal_x * invZ, focal_y * invZ);
        vec2 J2 = -(J1 * eyePos.xy) * invZ;
        J = mat3(
            J1.x, 0.0, 0.0,
            0.0, J1.y, 0.0,
            J2.x, J2.y, 0.0
        );
    }

    // Project 3D covariance to 2D: V' = J * W * sigma * W^T * J^T
    // For CovSplat, sigma is already in view space, so use J * sigma * J^T
    mat3 sigma_prime;
    if (enableCovSplats) {
        sigma_prime = J * sigma * transpose(J);
    } else {
        mat3 JW = J * W;
        sigma_prime = JW * sigma * transpose(JW);
    }

    // Spark separates uncompensated pre-blur from compensated anti-alias blur.
    sigma_prime[0][0] += preBlurAmount;
    sigma_prime[1][1] += preBlurAmount;
    mat2 covOrig = mat2(sigma_prime);
    float detOrig = covOrig[0][0] * covOrig[1][1] - covOrig[0][1] * covOrig[1][0];

    // Depth-of-field blur (Spark-style)
    float fullBlurAmount = blurAmount;
    if (focalDistance > 0.0 && apertureAngle > 0.0) {
        float maxPixelRadius = maxScreenRadius;
        if (eyePos.z < 0.0) {
            float focusBlur = abs((-eyePos.z - focalDistance) / eyePos.z);
            float apertureRadius = focal_x * tan(0.5 * apertureAngle);
            float focusRadius = focusBlur * apertureRadius;
            fullBlurAmount = clamp(focusRadius * focusRadius, blurAmount, maxPixelRadius * maxPixelRadius);
        }
    }

    sigma_prime[0][0] += fullBlurAmount;
    sigma_prime[1][1] += fullBlurAmount;

    mat2 cov2D = mat2(sigma_prime);
    float detBlur = cov2D[0][0] * cov2D[1][1] - cov2D[0][1] * cov2D[1][0];

    // NDC projection
    vec4 projPos = proj * eyePos;
    vec3 ndc = projPos.xyz / projPos.w;

    // Guard band culling
    if (ndc.x < -clipXY || ndc.x > clipXY || ndc.y < -clipXY || ndc.y > clipXY) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    // Compute ellipse extents from 2D covariance eigenvalues
    float a = cov2D[0][0], b = cov2D[0][1], c = cov2D[1][1];
    float apco2 = (a + c) * 0.5;
    float amco2 = (a - c) * 0.5;
    float term = sqrt(amco2 * amco2 + b * b);
    float lambda1 = max(apco2 + term, 0.0);
    // Subtracting nearly equal eigenvalues loses the thin axis of very
    // anisotropic splats. Spark obtains it from their product instead.
    float lambda2 = lambda1 > 0.0 ? max(0.0, detBlur / lambda1) : 0.0;
    vec2 majorUnnormalized = amco2 >= 0.0
        ? vec2(term + amco2, b) : vec2(b, term - amco2);
    float majorLength2 = dot(majorUnnormalized, majorUnnormalized);
    vec2 majorDirection = majorLength2 > 0.0
        ? majorUnnormalized * inversesqrt(majorLength2) : vec2(1.0, 0.0);
    vec2 minorDirection = vec2(-majorDirection.y, majorDirection.x);

    if (enableBlurCompensation) {
        // A zero-area splat has no energy to spread into the blurred footprint.
        // Skipping compensation for tiny/negative determinants instead made
        // these splats fully opaque relative to their original alpha.
        alpha *= detBlur > 0.0 ? sqrt(max(0.0, detOrig / detBlur)) : 0.0;
    }

    adjustedStdDev *= supportRadiusScale;

    float r1 = adjustedStdDev * sqrt(lambda1);
    float r2 = adjustedStdDev * sqrt(lambda2);
    float maxRadius = max(r1, r2);
    if (sparkRadiusClamp) {
        r1 = min(r1, maxScreenRadius);
        r2 = min(r2, maxScreenRadius);
    } else if (maxRadius > maxScreenRadius) {
        float radiusScale = maxScreenRadius / maxRadius;
        r1 *= radiusScale;
        r2 *= radiusScale;
    }
    if (r1 < minPixelRadius && r2 < minPixelRadius) {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);
        return;
    }

    vec2 majAxis = r1 * majorDirection;
    vec2 minAxis = r2 * minorDirection;

    // Offset quad vertices along ellipse axes
    vec2 offset = majAxis * inPos.x + minAxis * inPos.y;
    offset.x *= (2.0 / viewport_size.x) * projPos.w;
    offset.y *= (2.0 / viewport_size.y) * projPos.w;

    gl_Position = projPos;
    gl_Position.xy += offset;

    // Interpolate Gaussian coordinates over the final quad, as Spark does.
    // Clamping either axis must shrink its falloff along with its geometry;
    // an inverse of the original covariance would leave an opaque hard edge.
    PassSplatUv = inPos.xy * adjustedStdDev;

    vec3 baseColor = color;
    if (isColorPremultiplied) {
        baseColor = color / max(min(alpha, 1.0), 1.0e-6);
    }
    if (hasSH && shOrder > 0) {
        mat3 viewRot = mat3(view);
        vec3 camPos = -transpose(viewRot) * view[3].xyz;
        vec3 dir = normalize(position - camPos);
        int shBase = sortedIdx * 12;
        baseColor = evalSphericalHarmonics(dir, baseColor, shBase, shOrder);
    }

    if (enableSplatEdit) {
        vec4 editRgba = vec4(baseColor, alpha) * editRgbaScale + editRgbaOffset;
        baseColor = editRgba.rgb;
        alpha = editRgba.a;
    }

    // Debug visualization
    if (debugFlag == 1) {
        // Depth: map eye-space Z to grayscale (near=bright, far=dark)
        float depthVis = clamp(-eyePos.z / 100.0, 0.0, 1.0);
        baseColor = vec3(1.0 - depthVis);
        alpha = 1.0;
    } else if (debugFlag == 2) {
        // Normal: map view-space normal to RGB
        vec3 n = normalize(mat3(view) * quatToRotation(quat) * vec3(0.0, 0.0, 1.0));
        baseColor = n * 0.5 + 0.5;
        alpha = 1.0;
    }

    PassColor = vec4(baseColor, alpha);
    PassAdjustedStdDev = adjustedStdDev;
}
