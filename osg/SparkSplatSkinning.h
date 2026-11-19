#ifndef SPARK_SPLAT_SKINNING_H
#define SPARK_SPLAT_SKINNING_H

#include <osg/Referenced>
#include <osg/Matrixf>
#include <osg/Vec3f>
#include <osg/Vec4f>
#include <osg/Image>
#include <vector>
#include <cstdint>

enum class SplatSkinningMode
{
    DUAL_QUATERNION = 0,
    LINEAR_BLEND = 1
};

class SparkSplatSkinning : public osg::Referenced
{
public:
    SparkSplatSkinning(int numSplats, int numBones, SplatSkinningMode mode = SplatSkinningMode::DUAL_QUATERNION);

    // Skin weight data: per-splat 4 x uint16 (low 8 bits = weight/255, high 8 bits = bone index)
    void setSkinData(const uint16_t* data, int count);
    const std::vector<uint16_t>& skinData() const { return _skinData; }

    // Bone rest pose (must be set before setBoneQuatPos)
    void setRestQuatPos(int boneIndex, const osg::Vec4f& restQuat, const osg::Vec3f& restPos);

    // Update bone transforms (DQ mode)
    void setBoneQuatPos(int boneIndex, const osg::Vec4f& quat, const osg::Vec3f& pos);
    // Update bone transforms (LB mode)
    void setBoneQuatPosScale(int boneIndex, const osg::Vec4f& quat, const osg::Vec3f& pos, const osg::Vec3f& scale);

    // Call after all bones updated
    void updateBones();

    // GPU texture images
    osg::Image* skinTextureImage();
    osg::Image* boneTextureImage();

    int numSplats() const { return _numSplats; }
    int numBones() const { return _numBones; }
    SplatSkinningMode mode() const { return _mode; }
    bool dirty() const { return _dirty; }
    void setDirty(bool v) { _dirty = v; }

private:
    int _numSplats;
    int _numBones;
    SplatSkinningMode _mode;
    bool _dirty = true;

    std::vector<uint16_t> _skinData;  // numSplats * 4
    std::vector<float> _boneData;     // DQ: numBones * 8, LB: numBones * 12

    // Rest pose storage
    struct BoneRestPose {
        osg::Vec4f quat;
        osg::Vec3f pos;
        osg::Vec3f scale;
    };
    std::vector<BoneRestPose> _restPoses;
    std::vector<osg::Matrixf> _restInvMats;  // LB mode only

    osg::ref_ptr<osg::Image> _skinImage;
    osg::ref_ptr<osg::Image> _boneImage;
};

#endif
