#pragma once

#include <osg/BoundingBox>
#include <osg/Camera>
#include <osg/Matrixd>
#include <osg/Node>
#include <osg/Vec2d>

#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

namespace osg_3dgs {

struct ModelStats {
    int sourcePoints = 0;
    int drawnPoints = 0;
    int deletedPoints = 0;
    int residentPages = 0;
    int totalPages = 0;
    bool paged = false;
};

// Call from the host's update thread, with rendering stopped during editing
// and destruction (for example, an OSG SingleThreaded viewer).
class GaussianModel {
public:
    static bool supportsPath(const std::string& path);
    // An empty shader directory is sufficient for data-only use.
    static std::unique_ptr<GaussianModel> load(const std::string& path,
                                               const std::string& shaderDirectory = {});
    ~GaussianModel();

    GaussianModel(const GaussianModel&) = delete;
    GaussianModel& operator=(const GaussianModel&) = delete;

    // Attach the returned node to the host scene. A model binds to one camera;
    // asking for another returns null. Destruction empties the node and removes
    // this model's camera callback. Do not retain or clone its internal nodes.
    osg::ref_ptr<osg::Node> createSceneNode(osg::Camera& camera);
    void requestSort();

    ModelStats stats() const;
    osg::BoundingBox bounds() const;
    osg::BoundingBox focusBounds() const;

    bool canEdit() const;
    bool canSave() const;
    bool canUndo() const;
    // IDs refer to source points and stay stable after deletion and undo.
    // The returned reference is owned by the model; its contents change on edit.
    const std::vector<int>& editablePointIds() const;
    bool pointPosition(int id, osg::Vec3f& position) const;
    int deletePoints(std::vector<int> ids);
    std::vector<int> undoDeletion();

    // Pixel coordinates are relative to the viewport's top-left corner.
    // modelToClip is the OSG model * view * projection matrix.
    bool projectPoint(int id, const osg::Matrixd& modelToClip,
                      const osg::Vec2d& viewportSize, osg::Vec2d& pixel) const;
    int pickPoint(const osg::Matrixd& modelToClip, const osg::Vec2d& viewportSize,
                  const osg::Vec2d& pixel, double radius = 8.0) const;
    std::vector<int> pickRectangle(const osg::Matrixd& modelToClip,
                                   const osg::Vec2d& viewportSize,
                                   const osg::Vec2d& cornerA, const osg::Vec2d& cornerB) const;

    // Supply a binary stream. The caller controls file creation and commit.
    bool writePly(std::ostream& output) const;

private:
    struct Impl;
    explicit GaussianModel(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> _impl;
};

} // namespace osg_3dgs
