#include <osg_3dgs/GaussianModel.h>

#include "GaussianDrawObj.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <ostream>

namespace osg_3dgs {
namespace {
bool project(const osg::Vec3f& p, const osg::Matrixd& clip, const osg::Vec2d& size,
             osg::Vec2d& pixel, double& depth)
{
    const osg::Vec4d h = osg::Vec4d(p.x(), p.y(), p.z(), 1.0) * clip;
    const double w = h.w();
    if (!(size.x() > 0 && size.y() > 0 && w > 0 && std::abs(h.x()) <= w &&
          std::abs(h.y()) <= w && std::abs(h.z()) <= w))
        return false;
    pixel.set((h.x() / w + 1.0) * 0.5 * size.x(), (1.0 - h.y() / w) * 0.5 * size.y());
    depth = h.z() / w;
    return true;
}
}

struct GaussianModel::Impl {
    Impl(const std::string& path, const std::string& shaderDirectory)
        : draw(path, shaderDirectory), canRender(!shaderDirectory.empty()) {}

    GaussianDrawObj draw;
    const bool canRender;
    osg::ref_ptr<osg::Group> scene;
    std::vector<unsigned char> deleted;
    std::vector<std::vector<int>> history;
};

GaussianModel::GaussianModel(std::unique_ptr<Impl> impl) : _impl(std::move(impl)) {}

GaussianModel::~GaussianModel()
{
    if (_impl->scene) _impl->scene->removeChildren(0, _impl->scene->getNumChildren());
}

bool GaussianModel::supportsPath(const std::string& path)
{
    const auto dot = path.find_last_of('.');
    if (dot == std::string::npos) return false;
    auto ext = path.substr(dot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return ext == "ply" || ext == "splat" || ext == "spz" || ext == "ksplat" || ext == "rad";
}

std::unique_ptr<GaussianModel> GaussianModel::load(const std::string& path, const std::string& shaderDirectory)
{
    if (!supportsPath(path)) return nullptr;
    auto impl = std::make_unique<Impl>(path, shaderDirectory);
    if (impl->draw.getRenderablePointCount() <= 0) return nullptr;
    return std::unique_ptr<GaussianModel>(new GaussianModel(std::move(impl)));
}

osg::ref_ptr<osg::Node> GaussianModel::createSceneNode(osg::Camera& camera)
{
    if (!_impl->canRender) return nullptr;
    if (_impl->scene)
        return _impl->draw.camera() == &camera ? _impl->scene.get() : nullptr;

    if (auto* context = camera.getGraphicsContext()) {
        if (auto* state = context->getState()) state->setUseModelViewAndProjectionUniforms(true);
    }
    auto node = _impl->draw.createNode(camera);
    if (!node) return nullptr;
    _impl->scene = new osg::Group;
    _impl->scene->addChild(node);
    // Building render resources initializes the draw node after data-only edits.
    if (!_impl->deleted.empty()) _impl->draw.setDeletedPoints(_impl->deleted);
    return _impl->scene.get();
}

void GaussianModel::requestSort() { _impl->draw.requestSort(); }
osg::BoundingBox GaussianModel::bounds() const { return _impl->draw.getBounds(); }
osg::BoundingBox GaussianModel::focusBounds() const { return _impl->draw.getFocusBounds(); }
bool GaussianModel::canEdit() const { return _impl->draw.supportsPointEditing(); }
bool GaussianModel::canSave() const { return canEdit() && !editablePointIds().empty(); }
bool GaussianModel::canUndo() const { return !_impl->history.empty(); }

ModelStats GaussianModel::stats() const
{
    const auto& draw = _impl->draw;
    ModelStats result;
    result.sourcePoints = draw.getOriginalPointCount();
    result.drawnPoints = draw.getGpuSortDrawCount();
    result.paged = draw.isRadFile();
    if (result.paged) {
        result.residentPages = draw.sparkDrawable().radLoadedChunkCount();
        result.totalPages = draw.sparkDrawable().radTotalChunkCount();
    } else if (canEdit()) {
        result.deletedPoints = result.sourcePoints - int(editablePointIds().size());
    }
    return result;
}

const std::vector<int>& GaussianModel::editablePointIds() const
{
    static const std::vector<int> empty;
    return canEdit() ? _impl->draw.editableIndices() : empty;
}

bool GaussianModel::pointPosition(int id, osg::Vec3f& position) const
{
    const auto& points = _impl->draw.getPoints();
    if (!canEdit() || id < 0 || size_t(id) >= points.size() ||
        (!_impl->deleted.empty() && _impl->deleted[id])) return false;
    position = points[id].position;
    return true;
}

int GaussianModel::deletePoints(std::vector<int> ids)
{
    if (!canEdit() || ids.empty()) return 0;
    auto& deleted = _impl->deleted;
    const auto count = _impl->draw.getOriginalPointCount();
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    ids.erase(std::remove_if(ids.begin(), ids.end(), [&](int id) {
        return id < 0 || id >= count || (!deleted.empty() && deleted[id]);
    }), ids.end());
    if (ids.empty()) return 0;
    if (deleted.empty()) deleted.resize(count);
    _impl->history.push_back(std::move(ids));
    for (int id : _impl->history.back()) deleted[id] = 1;
    _impl->draw.setDeletedPoints(deleted);
    return int(_impl->history.back().size());
}

std::vector<int> GaussianModel::undoDeletion()
{
    if (!canUndo()) return {};
    auto ids = std::move(_impl->history.back());
    _impl->history.pop_back();
    for (int id : ids) _impl->deleted[id] = 0;
    _impl->draw.setDeletedPoints(_impl->deleted);
    return ids;
}

bool GaussianModel::projectPoint(int id, const osg::Matrixd& clip, const osg::Vec2d& size, osg::Vec2d& pixel) const
{
    osg::Vec3f position;
    double depth;
    return pointPosition(id, position) && project(position, clip, size, pixel, depth);
}

int GaussianModel::pickPoint(const osg::Matrixd& clip, const osg::Vec2d& size,
                             const osg::Vec2d& pixel, double radius) const
{
    if (!(radius >= 0)) return -1;
    int picked = -1;
    double nearestDepth = 2.0;
    double nearestDistance = std::numeric_limits<double>::infinity();
    const auto& points = _impl->draw.getPoints();
    for (int id : editablePointIds()) {
        osg::Vec2d screen;
        double depth;
        if (points[id].color.a() <= 0 || !project(points[id].position, clip, size, screen, depth)) continue;
        const double distance = (screen - pixel).length2();
        if (distance <= radius * radius && (depth < nearestDepth || (depth == nearestDepth && distance < nearestDistance))) {
            picked = id;
            nearestDepth = depth;
            nearestDistance = distance;
        }
    }
    return picked;
}

std::vector<int> GaussianModel::pickRectangle(const osg::Matrixd& clip, const osg::Vec2d& size,
                                              const osg::Vec2d& a, const osg::Vec2d& b) const
{
    const auto& points = _impl->draw.getPoints();
    const osg::Vec2d low(std::min(a.x(), b.x()), std::min(a.y(), b.y()));
    const osg::Vec2d high(std::max(a.x(), b.x()), std::max(a.y(), b.y()));
    std::vector<int> hits;
    for (int id : editablePointIds()) {
        osg::Vec2d pixel;
        double depth;
        if (points[id].color.a() > 0 && project(points[id].position, clip, size, pixel, depth) &&
            pixel.x() >= low.x() && pixel.x() <= high.x() && pixel.y() >= low.y() && pixel.y() <= high.y())
            hits.push_back(id);
    }
    return hits;
}

bool GaussianModel::writePly(std::ostream& output) const
{
    if (!canSave() || !output) return false;
    const auto& indices = editablePointIds();
    const auto& points = _impl->draw.getPoints();
    int degree = 0;
    for (int id : indices) degree = std::max(degree, std::min(3, points[id].shOrder));
    const int shCount = (degree + 1) * (degree + 1) - 1;

    std::string header = "ply\nformat binary_little_endian 1.0\nelement vertex " +
        std::to_string(indices.size()) + "\n";
    const auto property = [&](const std::string& name) { header += "property float " + name + '\n'; };
    for (auto name : {"x", "y", "z", "nx", "ny", "nz", "f_dc_0", "f_dc_1", "f_dc_2"}) property(name);
    for (int i = 0; i < shCount * 3; ++i) property("f_rest_" + std::to_string(i));
    for (auto name : {"opacity", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"}) property(name);
    header += "end_header\n";
    if (!output.write(header.data(), header.size())) return false;

    constexpr float shC0 = 0.28209479177387814f;
    std::string batch;
    batch.reserve(256 * 1024);
    const auto appendFloat = [&](float value) {
        uint32_t bits;
        static_assert(sizeof(bits) == sizeof(value) && std::numeric_limits<float>::is_iec559);
        std::memcpy(&bits, &value, sizeof(bits));
        for (int shift = 0; shift < 32; shift += 8)
            batch.push_back(static_cast<char>((bits >> shift) & 0xff));
    };
    for (int id : indices) {
        const auto& p = points[id];
        for (int axis = 0; axis < 3; ++axis) appendFloat(p.position[axis]);
        for (int axis = 0; axis < 3; ++axis) appendFloat(0.0f);
        for (int channel = 0; channel < 3; ++channel) appendFloat((p.color[channel] - 0.5f) / shC0);
        const int pointDegree = std::clamp(p.shOrder, 0, 3);
        const int pointShCount = (pointDegree + 1) * (pointDegree + 1) - 1;
        for (int channel = 0; channel < 3; ++channel)
            for (int k = 0; k < shCount; ++k) appendFloat(k < pointShCount ? p.sh[channel * 15 + k] : 0.0f);
        // PLY stores pre-sigmoid opacity, log scales, and a W-first quaternion.
        const double alpha = p.color.a();
        appendFloat(alpha <= 0.0 ? -104.0f : alpha >= 1.0 ? 104.0f : float(std::log(alpha) - std::log1p(-alpha)));
        for (int axis = 0; axis < 3; ++axis)
            appendFloat(std::log(std::max(p.scale[axis], std::numeric_limits<float>::denorm_min())));
        appendFloat(p.rotation.w());
        for (int axis = 0; axis < 3; ++axis) appendFloat(p.rotation[axis]);
        if (batch.size() >= 128 * 1024) {
            if (!output.write(batch.data(), batch.size())) return false;
            batch.clear();
        }
    }
    return bool(output.write(batch.data(), batch.size()));
}

} // namespace osg_3dgs
