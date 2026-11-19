// ============================================================================
// splat_edit.glsl — Splat Editing (SDF-based RGBA + Displacement)
// Ported from Spark's SplatEdit.ts
// ============================================================================

// SDF type constants
const uint SDF_TYPE_ALL = 0u;
const uint SDF_TYPE_PLANE = 1u;
const uint SDF_TYPE_SPHERE = 2u;
const uint SDF_TYPE_BOX = 3u;
const uint SDF_TYPE_ELLIPSOID = 4u;
const uint SDF_TYPE_CYLINDER = 5u;
const uint SDF_TYPE_CAPSULE = 6u;
const uint SDF_TYPE_INFINITE_CONE = 7u;
const uint SDF_FLAG_TYPE = 0xFFu;
const uint SDF_FLAG_INVERT = 1u << 8u;

// Edit blend constants
const uint EDIT_BLEND_MULTIPLY = 0u;
const uint EDIT_BLEND_SET_RGB = 1u;
const uint EDIT_BLEND_ADD_RGBA = 2u;
const uint EDIT_FLAG_BLEND = 0xFFu;
const uint EDIT_FLAG_INVERT = 0x100u;

const int MAX_SDF_VALUES = 4;

// Unpack a single SDF from the texture (8 texels of uvec4 starting at sdfIndex)
void unpackSdf(usampler2D sdfTex, int sdfIndex, int numSdfs,
               out vec3 center, out vec4 quaternion, out vec3 scale,
               out vec4 sizes, out vec4 values[MAX_SDF_VALUES])
{
    // Each SDF occupies 8 texels in a row of width=8
    // texel coord: x = texelInRow (0..7), y = sdfIndex
    uvec4 t0 = texelFetch(sdfTex, ivec2(0, sdfIndex), 0);
    uvec4 t1 = texelFetch(sdfTex, ivec2(1, sdfIndex), 0);
    uvec4 t2 = texelFetch(sdfTex, ivec2(2, sdfIndex), 0);
    uvec4 t3 = texelFetch(sdfTex, ivec2(3, sdfIndex), 0);
    uvec4 t4 = texelFetch(sdfTex, ivec2(4, sdfIndex), 0);
    uvec4 t5 = texelFetch(sdfTex, ivec2(5, sdfIndex), 0);
    uvec4 t6 = texelFetch(sdfTex, ivec2(6, sdfIndex), 0);
    uvec4 t7 = texelFetch(sdfTex, ivec2(7, sdfIndex), 0);

    center = vec3(uintBitsToFloat(t0.x), uintBitsToFloat(t0.y), uintBitsToFloat(t0.z));
    // t0.w = flags (type | invert<<8) — extracted separately
    quaternion = vec4(uintBitsToFloat(t1.x), uintBitsToFloat(t1.y),
                      uintBitsToFloat(t1.z), uintBitsToFloat(t1.w));
    scale = vec3(uintBitsToFloat(t2.x), uintBitsToFloat(t2.y), uintBitsToFloat(t2.z));
    sizes = vec4(uintBitsToFloat(t3.x), uintBitsToFloat(t3.y),
                 uintBitsToFloat(t3.z), uintBitsToFloat(t3.w));
    values[0] = vec4(uintBitsToFloat(t4.x), uintBitsToFloat(t4.y),
                     uintBitsToFloat(t4.z), uintBitsToFloat(t4.w));
    values[1] = vec4(uintBitsToFloat(t5.x), uintBitsToFloat(t5.y),
                     uintBitsToFloat(t5.z), uintBitsToFloat(t5.w));
    values[2] = vec4(uintBitsToFloat(t6.x), uintBitsToFloat(t6.y),
                     uintBitsToFloat(t6.z), uintBitsToFloat(t6.w));
    values[3] = vec4(uintBitsToFloat(t7.x), uintBitsToFloat(t7.y),
                     uintBitsToFloat(t7.z), uintBitsToFloat(t7.w));
}

// Get SDF type and invert flag from texture
void unpackSdfFlags(usampler2D sdfTex, int sdfIndex, out uint sdfType, out bool sdfInvert)
{
    uvec4 t0 = texelFetch(sdfTex, ivec2(0, sdfIndex), 0);
    sdfType = t0.w & SDF_FLAG_TYPE;
    sdfInvert = (t0.w & SDF_FLAG_INVERT) != 0u;
}

// Evaluate a single SDF primitive distance
float evaluateSdfPrimitive(uint sdfType, vec3 p, vec3 center, vec4 quaternion,
                           vec3 scale, vec4 sizes)
{
    float opacity = sizes.x;
    float radius = sizes.y;

    // Transform point into SDF local space
    vec3 localP = p - center;
    // Rotate into SDF orientation
    mat3 R = quatToRotation(quaternion);
    vec3 rp = R * localP;

    float d = 0.0;

    if (sdfType == SDF_TYPE_ALL) {
        d = 0.0; // always inside
    } else if (sdfType == SDF_TYPE_PLANE) {
        // Plane: distance along local Z axis
        d = rp.z;
    } else if (sdfType == SDF_TYPE_SPHERE) {
        d = length(rp) - radius;
    } else if (sdfType == SDF_TYPE_BOX) {
        vec3 q = abs(rp) - scale;
        d = length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
    } else if (sdfType == SDF_TYPE_ELLIPSOID) {
        vec3 q = abs(rp) / max(scale, vec3(1e-6));
        d = (length(q) - 1.0) * min(min(scale.x, scale.y), scale.z);
    } else if (sdfType == SDF_TYPE_CYLINDER) {
        // Cylinder along local Z axis
        vec2 q = abs(vec2(length(rp.xy), rp.z)) - vec2(radius, scale.z);
        d = min(max(q.x, q.y), 0.0) + length(max(q, 0.0));
    } else if (sdfType == SDF_TYPE_CAPSULE) {
        // Capsule along local Z axis with a circular radial section.
        vec2 q = vec2(length(rp.xy), abs(rp.z)) - vec2(radius, scale.z);
        d = min(max(q.x, q.y), 0.0) + length(max(q, 0.0));
    } else if (sdfType == SDF_TYPE_INFINITE_CONE) {
        // Infinite cone along local Z axis
        float q = length(rp.xy);
        d = max(q * sign(radius) - rp.z, -rp.z);
    }

    return d;
}

// Evaluate combined SDF distance with softmax blending
float evaluateSdfArray(usampler2D sdfTex, int sdfFirst, int sdfCount,
                       float sdfSmooth, vec3 p, int numSdfs)
{
    if (sdfCount <= 0) return 1e10;

    float minD = 1e10;
    float sumW = 0.0;
    float sumWD = 0.0;

    for (int i = 0; i < sdfCount && sdfFirst + i < numSdfs; ++i) {
        int sdfIdx = sdfFirst + i;

        uint sdfType;
        bool sdfInvert;
        unpackSdfFlags(sdfTex, sdfIdx, sdfType, sdfInvert);

        vec3 center;
        vec4 quaternion;
        vec3 scale;
        vec4 sizes;
        vec4 values[MAX_SDF_VALUES];
        unpackSdf(sdfTex, sdfIdx, numSdfs, center, quaternion, scale, sizes, values);

        float d = evaluateSdfPrimitive(sdfType, p, center, quaternion, scale, sizes);

        if (sdfInvert) d = -d;

        float opacity = sizes.x;

        if (sdfSmooth > 0.0) {
            // Softmax blending（并集：对负值 d 给更高权重）
            float w = opacity * exp(clamp(-d / max(sdfSmooth, 1e-6), -80.0, 80.0));
            sumW += w;
            sumWD += w * d;
            minD = min(minD, d);
        } else {
            // Hard union（取最小距离=最近表面）
            minD = min(minD, d);
        }
    }

    if (sdfSmooth > 0.0 && sumW > 0.0) {
        return sumWD / sumW;
    }
    return minD;
}

// Compute modulation factor from SDF distance and soft edge
float modulateSdfArray(float dist, float softEdge)
{
    if (softEdge > 0.0) {
        return 1.0 - clamp(dist / softEdge + 1.0, 0.0, 1.0);
    }
    return dist <= 0.0 ? 1.0 : 0.0;
}

// Decode packed edit data from a uvec4
void decodeEdit(uvec4 packedEdit, out uint blendMode, out bool editInvert,
                out int sdfFirst, out int sdfCount,
                out float softEdge, out float sdfSmooth)
{
    blendMode = packedEdit.x & EDIT_FLAG_BLEND;
    editInvert = (packedEdit.x & EDIT_FLAG_INVERT) != 0u;
    sdfFirst = int(packedEdit.y & 0xFFFFu);
    sdfCount = int((packedEdit.y >> 16u) & 0xFFFFu);
    softEdge = uintBitsToFloat(packedEdit.z);
    sdfSmooth = uintBitsToFloat(packedEdit.w);
}

void accumulateEditValues(bool editInvert,
                          int sdfFirst, int sdfCount,
                          float softEdge, float sdfSmooth,
                          usampler2D sdfTex, int numSdfs,
                          vec3 position,
                          out float modulation, out vec4 totalColor, out vec3 totalDisplace)
{
    float dist = evaluateSdfArray(sdfTex, sdfFirst, sdfCount, sdfSmooth, position, numSdfs);
    modulation = modulateSdfArray(dist, softEdge);

    if (editInvert) modulation = 1.0 - modulation;

    totalColor = vec4(0.0);
    totalDisplace = vec3(0.0);
    if (modulation <= 0.0) return;

    float totalWeight = 0.0;

    for (int i = 0; i < sdfCount && sdfFirst + i < numSdfs; ++i) {
        int sdfIdx = sdfFirst + i;

        uint sdfType;
        bool sdfInvert;
        unpackSdfFlags(sdfTex, sdfIdx, sdfType, sdfInvert);

        vec3 center;
        vec4 quaternion;
        vec3 scale;
        vec4 sizes;
        vec4 values[MAX_SDF_VALUES];
        unpackSdf(sdfTex, sdfIdx, numSdfs, center, quaternion, scale, sizes, values);

        float opacity = sizes.x;

        // Compute per-SDF modulation
        float d = evaluateSdfPrimitive(sdfType, position, center, quaternion, scale, sizes);
        if (sdfInvert) d = -d;
        float sdfMod = modulateSdfArray(d, softEdge);

        float w = opacity * sdfMod;
        totalWeight += w;
        totalColor += values[0] * w;    // RGBA
        totalDisplace += values[1].xyz * w; // XYZ displacement
    }

    if (totalWeight > 0.0) {
        totalColor /= totalWeight;
        totalDisplace /= totalWeight;
    }
}

// Every supported blend is affine in RGBA: result = rgba * scale + offset.
// This lets us evaluate edits in position order once and apply their combined
// color effect after SH evaluation, without evaluating the SDFs a second time.
void getEditRgbaTransform(uint blendMode, float modulation, vec4 totalColor,
                          out vec4 rgbaScale, out vec4 rgbaOffset)
{
    rgbaScale = vec4(1.0);
    rgbaOffset = vec4(0.0);
    if (modulation <= 0.0) return;

    if (blendMode == EDIT_BLEND_MULTIPLY) {
        rgbaScale = mix(vec4(1.0), totalColor, modulation);
    } else if (blendMode == EDIT_BLEND_SET_RGB) {
        rgbaScale = vec4(vec3(1.0 - modulation), mix(1.0, totalColor.a, modulation));
        rgbaOffset = vec4(totalColor.rgb * modulation, 0.0);
    } else if (blendMode == EDIT_BLEND_ADD_RGBA) {
        rgbaOffset = totalColor * modulation;
    }
}

void applyEditRgba(uint blendMode, float modulation, vec4 totalColor, inout vec4 rgba)
{
    vec4 rgbaScale, rgbaOffset;
    getEditRgbaTransform(blendMode, modulation, totalColor, rgbaScale, rgbaOffset);
    rgba = rgba * rgbaScale + rgbaOffset;
}

// Apply RGBA modification and displacement from a single edit
void applyRgbaDisplaceEdit(uint blendMode, bool editInvert,
                           int sdfFirst, int sdfCount,
                           float softEdge, float sdfSmooth,
                           usampler2D sdfTex, int numSdfs,
                           inout vec3 position, inout vec4 rgba)
{
    float modulation;
    vec4 totalColor;
    vec3 totalDisplace;
    accumulateEditValues(editInvert, sdfFirst, sdfCount, softEdge, sdfSmooth,
                         sdfTex, numSdfs, position, modulation, totalColor, totalDisplace);

    position += totalDisplace * modulation;
    applyEditRgba(blendMode, modulation, totalColor, rgba);
}

void applyRgbaEditOnly(uint blendMode, bool editInvert,
                       int sdfFirst, int sdfCount,
                       float softEdge, float sdfSmooth,
                       usampler2D sdfTex, int numSdfs,
                       vec3 position, inout vec4 rgba)
{
    float modulation;
    vec4 totalColor;
    vec3 totalDisplace;
    accumulateEditValues(editInvert, sdfFirst, sdfCount, softEdge, sdfSmooth,
                         sdfTex, numSdfs, position, modulation, totalColor, totalDisplace);
    applyEditRgba(blendMode, modulation, totalColor, rgba);
}

void applyDisplaceEditOnly(bool editInvert,
                           int sdfFirst, int sdfCount,
                           float softEdge, float sdfSmooth,
                           usampler2D sdfTex, int numSdfs,
                           inout vec3 position)
{
    float modulation;
    vec4 totalColor;
    vec3 totalDisplace;
    accumulateEditValues(editInvert, sdfFirst, sdfCount, softEdge, sdfSmooth,
                         sdfTex, numSdfs, position, modulation, totalColor, totalDisplace);
    position += totalDisplace * modulation;
}

// Convenience wrapper: apply edit from packed uvec4
void applyPackedRgbaDisplaceEdit(uvec4 packedEdit, usampler2D sdfTex, int numSdfs,
                                 inout vec3 position, inout vec4 rgba)
{
    uint blendMode;
    bool editInvert;
    int sdfFirst, sdfCount;
    float softEdge, sdfSmooth;
    decodeEdit(packedEdit, blendMode, editInvert, sdfFirst, sdfCount, softEdge, sdfSmooth);

    if (sdfCount <= 0) return;

    applyRgbaDisplaceEdit(blendMode, editInvert, sdfFirst, sdfCount,
                          softEdge, sdfSmooth, sdfTex, numSdfs,
                          position, rgba);
}

// Match Spark's sequential RGBA/displacement edits. Accumulate the color
// transform while advancing position so each edit samples the displaced point.
void accumulatePackedRgbaDisplaceEdit(uvec4 packedEdit, usampler2D sdfTex, int numSdfs,
                                      inout vec3 position,
                                      inout vec4 rgbaScale, inout vec4 rgbaOffset)
{
    uint blendMode;
    bool editInvert;
    int sdfFirst, sdfCount;
    float softEdge, sdfSmooth;
    decodeEdit(packedEdit, blendMode, editInvert, sdfFirst, sdfCount, softEdge, sdfSmooth);
    if (sdfCount <= 0) return;

    float modulation;
    vec4 totalColor;
    vec3 totalDisplace;
    accumulateEditValues(editInvert, sdfFirst, sdfCount, softEdge, sdfSmooth,
                         sdfTex, numSdfs, position, modulation, totalColor, totalDisplace);
    position += totalDisplace * modulation;

    vec4 editScale, editOffset;
    getEditRgbaTransform(blendMode, modulation, totalColor, editScale, editOffset);
    rgbaScale *= editScale;
    rgbaOffset = rgbaOffset * editScale + editOffset;
}

void applyPackedRgbaEditOnly(uvec4 packedEdit, usampler2D sdfTex, int numSdfs,
                             vec3 position, inout vec4 rgba)
{
    uint blendMode;
    bool editInvert;
    int sdfFirst, sdfCount;
    float softEdge, sdfSmooth;
    decodeEdit(packedEdit, blendMode, editInvert, sdfFirst, sdfCount, softEdge, sdfSmooth);

    if (sdfCount <= 0) return;

    applyRgbaEditOnly(blendMode, editInvert, sdfFirst, sdfCount,
                      softEdge, sdfSmooth, sdfTex, numSdfs,
                      position, rgba);
}

void applyPackedDisplaceEditOnly(uvec4 packedEdit, usampler2D sdfTex, int numSdfs,
                                 inout vec3 position)
{
    uint blendMode;
    bool editInvert;
    int sdfFirst, sdfCount;
    float softEdge, sdfSmooth;
    decodeEdit(packedEdit, blendMode, editInvert, sdfFirst, sdfCount, softEdge, sdfSmooth);

    if (sdfCount <= 0) return;

    applyDisplaceEditOnly(editInvert, sdfFirst, sdfCount,
                          softEdge, sdfSmooth, sdfTex, numSdfs,
                          position);
}
