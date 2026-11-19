#ifndef SPARK_PORTAL_H
#define SPARK_PORTAL_H

#include <osg/Referenced>
#include <osg/Vec3f>
#include <osg/Vec4f>
#include <osg/Matrixf>
#include <osg/Node>
#include <vector>
#include <functional>

struct PortalPair
{
    osg::Matrixf entryTransform;  // Entry portal world transform
    osg::Matrixf exitTransform;   // Exit portal world transform
    float radius = 1.0f;
    std::function<void()> onCross;  // Callback when camera crosses portal
};

class SparkPortal : public osg::Referenced
{
public:
    SparkPortal();

    int addPortalPair(const PortalPair& pair);
    void removePortalPair(int index);

    // Per-frame update: check for teleportation, compute portal uniforms
    void update(const osg::Matrixf& viewMatrix, const osg::Matrixf& projMatrix,
                const osg::Vec2f& viewportSize);

    // Portal uniforms for current frame
    bool enablePortal() const { return _enablePortal; }
    const osg::Vec3f& diskCenter() const { return _diskCenter; }
    const osg::Vec3f& diskNormal() const { return _diskNormal; }
    float diskRadius() const { return _diskRadius; }
    bool diskTwoSided() const { return _diskTwoSided; }

    // Behind-pass camera matrix
    const osg::Matrixf& behindViewMatrix() const { return _behindViewMatrix; }

    // Teleportation
    bool hasTeleported() const { return _hasTeleported; }
    const osg::Matrixf& teleportTransform() const { return _teleportTransform; }

    int numPortalPairs() const { return (int)_portals.size(); }

private:
    std::vector<PortalPair> _portals;
    bool _enablePortal = false;
    osg::Vec3f _diskCenter;
    osg::Vec3f _diskNormal;
    float _diskRadius = 0.0f;
    bool _diskTwoSided = false;
    osg::Matrixf _behindViewMatrix;
    bool _hasTeleported = false;
    osg::Matrixf _teleportTransform;

    osg::Vec3f _lastCameraPos;
    bool _hasLastCameraPos = false;

    // Find the portal most relevant to the current view
    int findPrimaryPortal(const osg::Matrixf& viewMatrix) const;
    // Compute entry->exit transform
    osg::Matrixf getEntryToExitTransform(const PortalPair& pair) const;
    // Check if camera path crosses a portal disk
    bool checkDiskCrossing(const osg::Vec3f& from, const osg::Vec3f& to,
                           const osg::Vec3f& diskCenter, const osg::Vec3f& diskNormal,
                           float diskRadius) const;
};

#endif
