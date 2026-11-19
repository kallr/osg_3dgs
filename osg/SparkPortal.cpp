#include "SparkPortal.h"

#include <osg/Matrix>
#include <cmath>
#include <algorithm>
#include <limits>

namespace
{
    // Helper for dot product since OSG Vec3f operator* returns Vec3f (component-wise)
    // and there's no direct dot() method in older OSG versions
    float dot3(const osg::Vec3f& a, const osg::Vec3f& b)
    {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    }
}

SparkPortal::SparkPortal()
{
}

int SparkPortal::addPortalPair(const PortalPair& pair)
{
    _portals.push_back(pair);
    return (int)_portals.size() - 1;
}

void SparkPortal::removePortalPair(int index)
{
    if (index >= 0 && index < (int)_portals.size())
        _portals.erase(_portals.begin() + index);
}

int SparkPortal::findPrimaryPortal(const osg::Matrixf& viewMatrix) const
{
    if (_portals.empty())
        return -1;

    // Extract camera position from view matrix inverse
    osg::Matrixf invView = osg::Matrixf::inverse(viewMatrix);
    osg::Vec3f camPos(invView(3, 0), invView(3, 1), invView(3, 2));

    int bestIndex = -1;
    float bestScore = -std::numeric_limits<float>::max();

    for (int i = 0; i < (int)_portals.size(); ++i)
    {
        const PortalPair& pair = _portals[i];

        // Entry portal center in world space
        osg::Vec3f entryCenter(pair.entryTransform(3, 0), pair.entryTransform(3, 1), pair.entryTransform(3, 2));

        // Entry portal normal in world space (local Z axis of entry transform)
        osg::Vec3f entryNormal(pair.entryTransform(2, 0), pair.entryTransform(2, 1), pair.entryTransform(2, 2));
        entryNormal.normalize();

        // Distance from camera to portal center
        osg::Vec3f toPortal = entryCenter - camPos;
        float dist = toPortal.length();

        // Facing: how much the portal faces the camera
        osg::Vec3f toPortalDir = toPortal;
        toPortalDir.normalize();
        float facing = -dot3(toPortalDir, entryNormal);  // Negative because we want portal facing toward camera

        // Only consider portals that face the camera
        if (facing <= 0.0f)
            continue;

        // Score: prefer closer portals that face the camera
        float distScore = 1.0f / (1.0f + dist * dist);
        float score = facing * distScore;

        if (score > bestScore)
        {
            bestScore = score;
            bestIndex = i;
        }
    }

    return bestIndex;
}

osg::Matrixf SparkPortal::getEntryToExitTransform(const PortalPair& pair) const
{
    // exitTransform * inverse(entryTransform)
    osg::Matrixf invEntry = osg::Matrixf::inverse(pair.entryTransform);
    return invEntry * pair.exitTransform;
}

bool SparkPortal::checkDiskCrossing(const osg::Vec3f& from, const osg::Vec3f& to,
                                     const osg::Vec3f& diskCenter, const osg::Vec3f& diskNormal,
                                     float diskRadius) const
{
    // Ray-plane intersection
    osg::Vec3f dir = to - from;
    float denom = dot3(dir, diskNormal);
    if (std::abs(denom) < 1e-8f)
        return false;  // Path parallel to disk

    float t = dot3(diskCenter - from, diskNormal) / denom;
    if (t < 0.0f || t > 1.0f)
        return false;  // Intersection outside path segment

    osg::Vec3f hitPoint = from + dir * t;
    osg::Vec3f diff = hitPoint - diskCenter;
    float dist2 = dot3(diff, diff);

    return dist2 <= diskRadius * diskRadius;
}

void SparkPortal::update(const osg::Matrixf& viewMatrix, const osg::Matrixf& projMatrix,
                          const osg::Vec2f& viewportSize)
{
    _hasTeleported = false;
    _enablePortal = false;
    _diskRadius = 0.0f;

    if (_portals.empty())
    {
        _hasLastCameraPos = false;
        return;
    }

    // Extract camera position from view matrix
    osg::Matrixf invView = osg::Matrixf::inverse(viewMatrix);
    osg::Vec3f camPos(invView(3, 0), invView(3, 1), invView(3, 2));

    // Check for teleportation across any portal
    if (_hasLastCameraPos)
    {
        for (const auto& pair : _portals)
        {
            osg::Vec3f entryCenter(pair.entryTransform(3, 0), pair.entryTransform(3, 1), pair.entryTransform(3, 2));
            osg::Vec3f entryNormal(pair.entryTransform(2, 0), pair.entryTransform(2, 1), pair.entryTransform(2, 2));
            entryNormal.normalize();

            if (checkDiskCrossing(_lastCameraPos, camPos, entryCenter, entryNormal, pair.radius))
            {
                _hasTeleported = true;
                _teleportTransform = getEntryToExitTransform(pair);
                if (pair.onCross)
                    pair.onCross();
                break;
            }
        }
    }

    _lastCameraPos = camPos;
    _hasLastCameraPos = true;

    // Find primary portal
    int primaryIdx = findPrimaryPortal(viewMatrix);
    if (primaryIdx < 0)
        return;

    const PortalPair& primary = _portals[primaryIdx];

    // Compute disk center and normal in camera (view) space
    osg::Vec3f entryCenterWorld(primary.entryTransform(3, 0), primary.entryTransform(3, 1), primary.entryTransform(3, 2));
    osg::Vec3f entryNormalWorld(primary.entryTransform(2, 0), primary.entryTransform(2, 1), primary.entryTransform(2, 2));
    entryNormalWorld.normalize();

    // Transform to view space: viewMatrix * worldPos
    osg::Vec4f centerView = viewMatrix * osg::Vec4f(entryCenterWorld, 1.0f);
    _diskCenter = osg::Vec3f(centerView.x(), centerView.y(), centerView.z());

    // Normal only needs rotation (upper-left 3x3 of view matrix)
    osg::Vec4f normalView = viewMatrix * osg::Vec4f(entryNormalWorld, 0.0f);
    _diskNormal = osg::Vec3f(normalView.x(), normalView.y(), normalView.z());
    _diskNormal.normalize();

    _diskRadius = primary.radius;
    _diskTwoSided = false;
    _enablePortal = true;

    // Compute behind-pass view matrix: viewMatrix * entryToExit
    // OSG 使用列向量约定 (M*v)，viewMatrix * entryToExit 表示先在 world 空间应用 entryToExit 再转 view 空间
    osg::Matrixf entryToExit = getEntryToExitTransform(primary);
    _behindViewMatrix = viewMatrix * entryToExit;
}
