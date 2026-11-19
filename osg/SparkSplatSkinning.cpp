#include "SparkSplatSkinning.h"

#include <osg/Matrixf>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef GL_RGBA_INTEGER
#define GL_RGBA_INTEGER 0x8D99
#endif
#ifndef GL_UNSIGNED_SHORT
#define GL_UNSIGNED_SHORT 0x1403
#endif
#ifndef GL_RGBA
#define GL_RGBA 0x1908
#endif
#ifndef GL_FLOAT
#define GL_FLOAT 0x1406
#endif

// Internal GL constant for RGBA16UI (not always available as macro)
#ifndef GL_RGBA16UI
#define GL_RGBA16UI 0x8D76
#endif
#ifndef GL_RGBA32F
#define GL_RGBA32F 0x8814
#endif

SparkSplatSkinning::SparkSplatSkinning(int numSplats, int numBones, SplatSkinningMode mode)
    : _numSplats(std::max(0, numSplats)), _numBones(std::max(0, numBones)), _mode(mode)
{
    // 校验参数非负，避免负数隐式转换为巨大 size_t 导致 bad_alloc
    _skinData.resize(_numSplats * 4, 0);
    int boneFloats = (mode == SplatSkinningMode::DUAL_QUATERNION) ? _numBones * 8 : _numBones * 12;
    _boneData.resize(boneFloats, 0.0f);
    _restPoses.resize(_numBones);
    if (mode == SplatSkinningMode::LINEAR_BLEND)
        _restInvMats.resize(_numBones);
}

void SparkSplatSkinning::setSkinData(const uint16_t* data, int count)
{
    int copyCount = std::min(count, _numSplats * 4);
    if (data && copyCount > 0)
        std::memcpy(_skinData.data(), data, copyCount * sizeof(uint16_t));
    _dirty = true;

    // Create/update skin image
    if (!_skinImage || _skinImage->s() != _numSplats) {
        _skinImage = new osg::Image;
        _skinImage->allocateImage(_numSplats, 1, 1, GL_RGBA_INTEGER, GL_UNSIGNED_SHORT);
        _skinImage->setInternalTextureFormat(GL_RGBA16UI);
    }
    size_t bytesNeeded = (size_t)_numSplats * 4 * sizeof(uint16_t);
    if (_skinImage->getTotalSizeInBytesIncludingMipmaps() >= (int)bytesNeeded) {
        std::memcpy(_skinImage->data(), _skinData.data(), bytesNeeded);
    }
}

void SparkSplatSkinning::setRestQuatPos(int boneIndex, const osg::Vec4f& restQuat, const osg::Vec3f& restPos)
{
    if (boneIndex < 0 || boneIndex >= _numBones) return;
    _restPoses[boneIndex].quat = restQuat;
    _restPoses[boneIndex].pos = restPos;
    _restPoses[boneIndex].scale = osg::Vec3f(1.0f, 1.0f, 1.0f);

    // For LB mode, compute rest inverse matrix
    if (_mode == SplatSkinningMode::LINEAR_BLEND) {
        // Build rest matrix from (quat, pos, scale)
        osg::Matrixf restMat;
        // Quaternion to rotation matrix
        float x = restQuat.x(), y = restQuat.y(), z = restQuat.z(), w = restQuat.w();
        float x2 = x + x, y2 = y + y, z2 = z + z;
        float xx = x * x2, xy = x * y2, xz = x * z2;
        float yy = y * y2, yz = y * z2, zz = z * z2;
        float wx = w * x2, wy = w * y2, wz = w * z2;
        float sx = restPos.x(), sy = restPos.y(), sz = restPos.z();
        float scx = _restPoses[boneIndex].scale.x();
        float scy = _restPoses[boneIndex].scale.y();
        float scz = _restPoses[boneIndex].scale.z();

        // Column-major 4x4: rotation * scale + translation
        restMat(0,0) = (1.0f - (yy + zz)) * scx;
        restMat(1,0) = (xy + wz) * scx;
        restMat(2,0) = (xz - wy) * scx;
        restMat(3,0) = 0.0f;

        restMat(0,1) = (xy - wz) * scy;
        restMat(1,1) = (1.0f - (xx + zz)) * scy;
        restMat(2,1) = (yz + wx) * scy;
        restMat(3,1) = 0.0f;

        restMat(0,2) = (xz + wy) * scz;
        restMat(1,2) = (yz - wx) * scz;
        restMat(2,2) = (1.0f - (xx + yy)) * scz;
        restMat(3,2) = 0.0f;

        restMat(0,3) = sx;
        restMat(1,3) = sy;
        restMat(2,3) = sz;
        restMat(3,3) = 1.0f;

        // 检查 invert 返回值，奇异矩阵反演失败时回退为单位矩阵
        if (!_restInvMats[boneIndex].invert(restMat))
            _restInvMats[boneIndex].makeIdentity();
    }
}

void SparkSplatSkinning::setBoneQuatPos(int boneIndex, const osg::Vec4f& quat, const osg::Vec3f& pos)
{
    if (boneIndex < 0 || boneIndex >= _numBones) return;
    if (_mode != SplatSkinningMode::DUAL_QUATERNION) return;

    const osg::Vec4f& restQuat = _restPoses[boneIndex].quat;
    const osg::Vec3f& restPos = _restPoses[boneIndex].pos;

    // Compute relative quaternion: relQuat = quat * restQuat.inverse()
    // Quaternion inverse for unit quaternion: (−x, −y, −z, w)
    float invRx = -restQuat.x(), invRy = -restQuat.y(), invRz = -restQuat.z(), invRw = restQuat.w();
    // Quaternion multiplication: q1 * q2
    float qx = quat.x(), qy = quat.y(), qz = quat.z(), qw = quat.w();
    float relX = qw * invRx + qx * invRw + qy * invRz - qz * invRy;
    float relY = qw * invRy - qx * invRz + qy * invRw + qz * invRx;
    float relZ = qw * invRz + qx * invRy - qy * invRx + qz * invRw;
    float relW = qw * invRw - qx * invRx - qy * invRy - qz * invRz;

    // Relative position
    float relPosX = pos.x() - restPos.x();
    float relPosY = pos.y() - restPos.y();
    float relPosZ = pos.z() - restPos.z();

    // Dual part: 0.5 * quaternion(relPos, 0) * relQuat
    // quaternion(v, 0) = (vx, vy, vz, 0)
    float dvx = relPosX, dvy = relPosY, dvz = relPosZ, dvw = 0.0f;
    float dualX = 0.5f * (dvw * relX + dvx * relW + dvy * relZ - dvz * relY);
    float dualY = 0.5f * (dvw * relY - dvx * relZ + dvy * relW + dvz * relX);
    float dualZ = 0.5f * (dvw * relZ + dvx * relY - dvy * relX + dvz * relW);
    float dualW = 0.5f * (dvw * relW - dvx * relX - dvy * relY - dvz * relZ);

    // Store in bone data: boneIndex * 8 + 0..3 = relQuat, boneIndex * 8 + 4..7 = dual
    int base = boneIndex * 8;
    _boneData[base + 0] = relX;
    _boneData[base + 1] = relY;
    _boneData[base + 2] = relZ;
    _boneData[base + 3] = relW;
    _boneData[base + 4] = dualX;
    _boneData[base + 5] = dualY;
    _boneData[base + 6] = dualZ;
    _boneData[base + 7] = dualW;
}

void SparkSplatSkinning::setBoneQuatPosScale(int boneIndex, const osg::Vec4f& quat, const osg::Vec3f& pos, const osg::Vec3f& scale)
{
    if (boneIndex < 0 || boneIndex >= _numBones) return;
    if (_mode != SplatSkinningMode::LINEAR_BLEND) return;

    // Build current matrix from (quat, pos, scale)
    osg::Matrixf currentMat;
    float x = quat.x(), y = quat.y(), z = quat.z(), w = quat.w();
    float x2 = x + x, y2 = y + y, z2 = z + z;
    float xx = x * x2, xy = x * y2, xz = x * z2;
    float yy = y * y2, yz = y * z2, zz = z * z2;
    float wx = w * x2, wy = w * y2, wz = w * z2;

    currentMat(0,0) = (1.0f - (yy + zz)) * scale.x();
    currentMat(1,0) = (xy + wz) * scale.x();
    currentMat(2,0) = (xz - wy) * scale.x();
    currentMat(3,0) = 0.0f;

    currentMat(0,1) = (xy - wz) * scale.y();
    currentMat(1,1) = (1.0f - (xx + zz)) * scale.y();
    currentMat(2,1) = (yz + wx) * scale.y();
    currentMat(3,1) = 0.0f;

    currentMat(0,2) = (xz + wy) * scale.z();
    currentMat(1,2) = (yz - wx) * scale.z();
    currentMat(2,2) = (1.0f - (xx + yy)) * scale.z();
    currentMat(3,2) = 0.0f;

    currentMat(0,3) = pos.x();
    currentMat(1,3) = pos.y();
    currentMat(2,3) = pos.z();
    currentMat(3,3) = 1.0f;

    // skinMat = restInverse * currentMat
    osg::Matrixf skinMat = _restInvMats[boneIndex] * currentMat;

    // Store as 3x4 matrix (3 rows of vec4)
    int base = boneIndex * 12;
    _boneData[base + 0] = skinMat(0,0);
    _boneData[base + 1] = skinMat(1,0);
    _boneData[base + 2] = skinMat(2,0);
    _boneData[base + 3] = skinMat(3,0);

    _boneData[base + 4] = skinMat(0,1);
    _boneData[base + 5] = skinMat(1,1);
    _boneData[base + 6] = skinMat(2,1);
    _boneData[base + 7] = skinMat(3,1);

    _boneData[base + 8]  = skinMat(0,2);
    _boneData[base + 9]  = skinMat(1,2);
    _boneData[base + 10] = skinMat(2,2);
    _boneData[base + 11] = skinMat(3,2);
}

void SparkSplatSkinning::updateBones()
{
    // Create/update bone image
    int texelWidth;
    if (_mode == SplatSkinningMode::DUAL_QUATERNION) {
        texelWidth = _numBones * 2;  // 2 texels per bone (quat + dual)
    } else {
        texelWidth = _numBones * 3;  // 3 texels per bone (3x4 matrix)
    }

    if (!_boneImage || _boneImage->s() != texelWidth) {
        _boneImage = new osg::Image;
        _boneImage->allocateImage(texelWidth, 1, 1, GL_RGBA, GL_FLOAT);
        _boneImage->setInternalTextureFormat(GL_RGBA32F);
    }

    size_t bytesNeeded = (size_t)texelWidth * 4 * sizeof(float);
    if (_boneImage->getTotalSizeInBytesIncludingMipmaps() >= (int)bytesNeeded) {
        std::memcpy(_boneImage->data(), _boneData.data(), bytesNeeded);
    }

    _dirty = true;
}

osg::Image* SparkSplatSkinning::skinTextureImage()
{
    if (!_skinImage && _numSplats > 0) {
        _skinImage = new osg::Image;
        _skinImage->allocateImage(_numSplats, 1, 1, GL_RGBA_INTEGER, GL_UNSIGNED_SHORT);
        _skinImage->setInternalTextureFormat(GL_RGBA16UI);
        size_t bytesNeeded = (size_t)_numSplats * 4 * sizeof(uint16_t);
        if (_skinImage->getTotalSizeInBytesIncludingMipmaps() >= (int)bytesNeeded) {
            std::memcpy(_skinImage->data(), _skinData.data(), bytesNeeded);
        }
    }
    return _skinImage.get();
}

osg::Image* SparkSplatSkinning::boneTextureImage()
{
    if (!_boneImage) {
        // Force creation via updateBones
        updateBones();
    }
    return _boneImage.get();
}
