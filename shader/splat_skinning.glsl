// ============================================================================
// splat_skinning.glsl — Splat Skinning (Dual Quaternion + Linear Blend)
// Ported from Spark's SplatSkinning.ts
// Adapted for osg_3dgs TBO-based texture access (usamplerBuffer / samplerBuffer)
// ============================================================================

// Apply Dual Quaternion skinning to Gsplat (center + quaternion)
// Used in gaussian.vert path where splats have position + scale + quaternion
void applyGsplatSkinningDQ(
    usamplerBuffer skinSampler, samplerBuffer boneSampler,
    int numSplats, int numBones, int splatIndex,
    inout vec3 center, inout vec4 quaternion)
{
    if (splatIndex < 0 || splatIndex >= numSplats) return;

    uvec4 skinData = texelFetch(skinSampler, splatIndex);

    float weights[4];
    weights[0] = float(skinData.x & 0xFFu) / 255.0;
    weights[1] = float(skinData.y & 0xFFu) / 255.0;
    weights[2] = float(skinData.z & 0xFFu) / 255.0;
    weights[3] = float(skinData.w & 0xFFu) / 255.0;

    uint boneIndices[4];
    boneIndices[0] = (skinData.x >> 8u) & 0xFFu;
    boneIndices[1] = (skinData.y >> 8u) & 0xFFu;
    boneIndices[2] = (skinData.z >> 8u) & 0xFFu;
    boneIndices[3] = (skinData.w >> 8u) & 0xFFu;

    vec4 quat = vec4(0.0);
    vec4 dual = vec4(0.0);
    for (int i = 0; i < 4; i++) {
        if (weights[i] > 0.0) {
            int boneIndex = int(boneIndices[i]);
            vec4 boneQuat = vec4(0.0, 0.0, 0.0, 1.0);
            vec4 boneDual = vec4(0.0);
            if (boneIndex < numBones) {
                boneQuat = texelFetch(boneSampler, boneIndex * 2);
                boneDual = texelFetch(boneSampler, boneIndex * 2 + 1);
            }
            // Antipodality fix: ensure consistent quaternion sign
            if ((i > 0) && (dot(quat, boneQuat) < 0.0)) {
                boneQuat = -boneQuat;
                boneDual = -boneDual;
            }
            quat += weights[i] * boneQuat;
            dual += weights[i] * boneDual;
        }
    }

    // Normalize dual quaternion（防止除零产生 NaN）
    float norm = length(quat);
    if (norm < 1e-12) return;
    quat /= norm;
    dual /= norm;

    // Extract translation from dual quaternion
    vec3 translate = vec3(
        2.0 * (-dual.w * quat.x + dual.x * quat.w - dual.y * quat.z + dual.z * quat.y),
        2.0 * (-dual.w * quat.y + dual.x * quat.z + dual.y * quat.w - dual.z * quat.x),
        2.0 * (-dual.w * quat.z - dual.x * quat.y + dual.y * quat.x + dual.z * quat.w)
    );

    // Transform center: rotate + translate
    mat3 rotMat = quatToRotation(quat);
    center = rotMat * center + translate;

    // Transform quaternion: 使用四元数乘法复合旋转 q_result = quat * quaternion
    // （而非错误地将 xyz 用矩阵变换、保留 w）
    vec4 q = quaternion;
    quaternion.x = quat.w * q.x + quat.x * q.w + quat.y * q.z - quat.z * q.y;
    quaternion.y = quat.w * q.y - quat.x * q.z + quat.y * q.w + quat.z * q.x;
    quaternion.z = quat.w * q.z + quat.x * q.y - quat.y * q.x + quat.z * q.w;
    quaternion.w = quat.w * q.w - quat.x * q.x - quat.y * q.y - quat.z * q.z;
    // Re-normalize the result quaternion
    float qLen = length(quaternion);
    if (qLen > 0.0) quaternion /= qLen;
}

// Apply DQ skinning to CovSplat (center + covariance)
// Used when enableCovSplats is true
void applyCovSplatDQSkinning(
    usamplerBuffer skinSampler, samplerBuffer boneSampler,
    int numSplats, int numBones, int splatIndex,
    inout vec3 center, inout vec3 xxyyzz, inout vec3 xyxzyz)
{
    if (splatIndex < 0 || splatIndex >= numSplats) return;

    uvec4 skinData = texelFetch(skinSampler, splatIndex);

    float weights[4];
    weights[0] = float(skinData.x & 0xFFu) / 255.0;
    weights[1] = float(skinData.y & 0xFFu) / 255.0;
    weights[2] = float(skinData.z & 0xFFu) / 255.0;
    weights[3] = float(skinData.w & 0xFFu) / 255.0;

    uint boneIndices[4];
    boneIndices[0] = (skinData.x >> 8u) & 0xFFu;
    boneIndices[1] = (skinData.y >> 8u) & 0xFFu;
    boneIndices[2] = (skinData.z >> 8u) & 0xFFu;
    boneIndices[3] = (skinData.w >> 8u) & 0xFFu;

    vec4 quat = vec4(0.0);
    vec4 dual = vec4(0.0);
    for (int i = 0; i < 4; i++) {
        if (weights[i] > 0.0) {
            int boneIndex = int(boneIndices[i]);
            vec4 boneQuat = vec4(0.0, 0.0, 0.0, 1.0);
            vec4 boneDual = vec4(0.0);
            if (boneIndex < numBones) {
                boneQuat = texelFetch(boneSampler, boneIndex * 2);
                boneDual = texelFetch(boneSampler, boneIndex * 2 + 1);
            }
            if ((i > 0) && (dot(quat, boneQuat) < 0.0)) {
                boneQuat = -boneQuat;
                boneDual = -boneDual;
            }
            quat += weights[i] * boneQuat;
            dual += weights[i] * boneDual;
        }
    }

    float norm = length(quat);
    if (norm < 1e-12) return;
    quat /= norm;
    dual /= norm;

    vec3 translate = vec3(
        2.0 * (-dual.w * quat.x + dual.x * quat.w - dual.y * quat.z + dual.z * quat.y),
        2.0 * (-dual.w * quat.y + dual.x * quat.z + dual.y * quat.w - dual.z * quat.x),
        2.0 * (-dual.w * quat.z - dual.x * quat.y + dual.y * quat.x + dual.z * quat.w)
    );

    mat3 rotMat = quatToRotation(quat);
    center = rotMat * center + translate;

    // Transform covariance: cov' = R * cov * R^T
    mat3 cov = mat3(
        xxyyzz.x, xyxzyz.x, xyxzyz.y,
        xyxzyz.x, xxyyzz.y, xyxzyz.z,
        xyxzyz.y, xyxzyz.z, xxyyzz.z
    );
    cov = rotMat * cov * transpose(rotMat);

    // Pack back to xxyyzz + xyxzyz
    xxyyzz = vec3(cov[0][0], cov[1][1], cov[2][2]);
    xyxzyz = vec3(cov[0][1], cov[0][2], cov[1][2]);
}

// Apply LB skinning to CovSplat (center + covariance)
void applyCovSplatLBSkinning(
    usamplerBuffer skinSampler, samplerBuffer boneSampler,
    int numSplats, int numBones, int splatIndex,
    inout vec3 center, inout vec3 xxyyzz, inout vec3 xyxzyz)
{
    if (splatIndex < 0 || splatIndex >= numSplats) return;

    uvec4 skinData = texelFetch(skinSampler, splatIndex);

    float weights[4];
    weights[0] = float(skinData.x & 0xFFu) / 255.0;
    weights[1] = float(skinData.y & 0xFFu) / 255.0;
    weights[2] = float(skinData.z & 0xFFu) / 255.0;
    weights[3] = float(skinData.w & 0xFFu) / 255.0;

    uint boneIndices[4];
    boneIndices[0] = (skinData.x >> 8u) & 0xFFu;
    boneIndices[1] = (skinData.y >> 8u) & 0xFFu;
    boneIndices[2] = (skinData.z >> 8u) & 0xFFu;
    boneIndices[3] = (skinData.w >> 8u) & 0xFFu;

    // Blend 3x4 matrices
    mat3x4 skinMat = mat3x4(0.0);
    for (int i = 0; i < 4; i++) {
        if (weights[i] > 0.0) {
            int boneIndex = int(boneIndices[i]);
            if (boneIndex < numBones) {
                vec4 row0 = texelFetch(boneSampler, boneIndex * 3);
                vec4 row1 = texelFetch(boneSampler, boneIndex * 3 + 1);
                vec4 row2 = texelFetch(boneSampler, boneIndex * 3 + 2);
                mat3x4 boneMat = mat3x4(row0, row1, row2);
                skinMat += weights[i] * boneMat;
            }
        }
    }

    // Extract 3x3 rotation/scale and translation from 3x4 matrix
    // skinMat columns: col0 = (m00,m10,m20,tx), col1 = (m01,m11,m21,ty), col2 = (m02,m12,m22,tz)
    mat3 M = mat3(
        skinMat[0].xyz,
        skinMat[1].xyz,
        skinMat[2].xyz
    );
    vec3 translate = vec3(skinMat[0].w, skinMat[1].w, skinMat[2].w);

    // Transform center
    center = M * center + translate;

    // Transform covariance: cov' = M * cov * M^T
    mat3 cov = mat3(
        xxyyzz.x, xyxzyz.x, xyxzyz.y,
        xyxzyz.x, xxyyzz.y, xyxzyz.z,
        xyxzyz.y, xyxzyz.z, xxyyzz.z
    );
    cov = M * cov * transpose(M);

    // Pack back to xxyyzz + xyxzyz
    xxyyzz = vec3(cov[0][0], cov[1][1], cov[2][2]);
    xyxzyz = vec3(cov[0][1], cov[0][2], cov[1][2]);
}
