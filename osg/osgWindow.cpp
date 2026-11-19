#include "osgWindow.h"

#include <QInputEvent>
#include <QApplication>
#include <QDebug>
#include <QByteArray>
#include <QList>
#include <QLocale>
#include <QPainter>
#include <QRubberBand>
#include <QSaveFile>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <streambuf>

#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventAdapter>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>
#include <osg/ShapeDrawable>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/Geode>
#include <osg/Program>
#include <osg/Shape>
#include <osg/TextureBuffer>
#include "osg/Vec2ui"

#include <osg_3dgs/GaussianModel.h>

class SelectionOverlay : public QWidget
{
public:
    explicit SelectionOverlay(QWidget* parent) : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_NoSystemBackground);
        hide();
    }

    void setMarkers(QPolygonF markers)
    {
        if (markers == _markers) return;
        _markers = std::move(markers);
        setVisible(!_markers.isEmpty());
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(QPen(QColor(18, 28, 34), 5, Qt::SolidLine, Qt::RoundCap));
        painter.drawPoints(_markers);
        painter.setPen(QPen(QColor(52, 228, 255), 3, Qt::SolidLine, Qt::RoundCap));
        painter.drawPoints(_markers);
    }

private:
    QPolygonF _markers;
};
#include "SparkFrameMetrics.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif // !M_PI

namespace {
constexpr double defaultNearPlane = 0.02;

int renderSetting(const char* name, int defaultValue, int minimum, int maximum)
{
    bool valid = false;
    int value = qEnvironmentVariableIntValue(name, &valid);
    return valid ? std::max(minimum, std::min(value, maximum)) : defaultValue;
}

bool cameraVectorSetting(const char* name, osg::Vec3d& value)
{
    const auto fields = qgetenv(name).split(',');
    if (fields.size() != 3) return false;
    for (int axis = 0; axis < 3; ++axis) {
        bool valid = false;
        value[axis] = fields[axis].trimmed().toDouble(&valid);
        if (!valid || !std::isfinite(value[axis])) return false;
    }
    return true;
}

double cameraScalarSetting(const char* name, double defaultValue, double minimum, double maximum)
{
    const QByteArray input = qgetenv(name);
    if (input.isEmpty()) return defaultValue;
    bool valid = false;
    const double value = input.toDouble(&valid);
    if (!valid || !std::isfinite(value) || value < minimum || value > maximum) {
        std::cerr << name << " ignored: expected a finite value in ["
            << minimum << "," << maximum << "]" << std::endl;
        return defaultValue;
    }
    return value;
}

// A benchmark run uses a repeatable camera path independent of mouse input,
// focus changes and trackball inertia. Normal interactive runs keep Trackball.
class BenchmarkManipulator : public osgGA::TrackballManipulator {
public:
    bool handle(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&) override { return false; }
};

class ViewerManipulator : public osgGA::TrackballManipulator {
public:
    explicit ViewerManipulator(osg::Camera* camera) : _camera(camera) {}

    void updateCamera(osg::Camera& camera) override
    {
        osgGA::TrackballManipulator::updateCamera(camera);
        if (!_adaptiveNear) return;

        // Keep the usual depth range until the orbit closes in on a detail.
        const double nearPlane = std::clamp(getDistance() * 0.05, 1.0e-5, defaultNearPlane);
        double fovy, aspect, currentNear, farPlane;
        if (camera.getProjectionMatrixAsPerspective(fovy, aspect, currentNear, farPlane) &&
            std::abs(nearPlane - currentNear) > 1.0e-10)
            camera.setProjectionMatrixAsPerspective(fovy, aspect, nearPlane, farPlane);
    }

protected:
    bool handleMouseRelease(const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& actions) override
    {
        // OSG can replay the last drag on release even with inertia disabled.
        if (_ga_t0.valid() && (_ga_t0->getButtonMask() &
            (osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON | osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)))
            flushMouseEventStack();
        return osgGA::TrackballManipulator::handleMouseRelease(event, actions);
    }

    bool performMovementMiddleMouseButton(double, double dx, double dy) override
    {
        const auto* camera = _camera.get();
        if (!camera) return false;

        // dx and dy are normalized viewport deltas. Undo the projection at
        // the orbit depth so the point under the pivot follows the drag.
        const auto& projection = camera->getProjectionMatrix();
        const double scaleX = projection(0, 0);
        const double scaleY = projection(1, 1);
        if (!(scaleX > 0.0 && scaleY > 0.0)) return false;

        const auto modifiers = _ga_t0->getModKeyMask();
        double depth = getDistance();
        double gain = 1.0;
        if (modifiers & osgGA::GUIEventAdapter::MODKEY_CTRL) {
            gain = 0.25;
        } else if (modifiers & osgGA::GUIEventAdapter::MODKEY_SHIFT) {
            osg::Vec3d homeEye, homeCenter, homeUp;
            getHomePosition(homeEye, homeCenter, homeUp);
            // Keep fast panning useful even when zoomed close to the pivot.
            depth = std::max(depth, 0.08 * (homeEye - homeCenter).length());
            gain = 3.0;
        }
        const osg::Vec3d offset(-dx * gain * depth / scaleX, -dy * gain * depth / scaleY, 0.0);
        setCenter(getCenter() + getRotation() * offset);
        return true;
    }

    bool performMovementRightMouseButton(double dt, double dx, double dy) override
    {
        return performMovementMiddleMouseButton(dt, dx, dy);
    }

    bool handleMouseWheel(const osgGA::GUIEventAdapter& event, osgGA::GUIActionAdapter& actions) override
    {
        if (event.getScrollingMotion() != osgGA::GUIEventAdapter::SCROLL_2D)
            return osgGA::TrackballManipulator::handleMouseWheel(event, actions);
        const double steps = std::max(-8.0, std::min(8.0, double(event.getScrollingDeltaY())));
        if (steps == 0.0) return false;
        const bool fine = (event.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) != 0;
        if (event.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT) {
            osg::Vec3d eye, center, up;
            getHomePosition(eye, center, up);
            const double reference = std::max(getDistance(), (eye - center).length());
            const osg::Vec3d forward = getRotation() * osg::Vec3d(0.0, 0.0, -1.0);
            setCenter(getCenter() + forward * (steps * reference * (fine ? 0.01 : 0.05)));
        } else {
            zoomModel(float(std::pow(fine ? 1.08 : 1.28, -steps) - 1.0), false);
        }
        actions.requestRedraw();
        return true;
    }

private:
    osg::observer_ptr<osg::Camera> _camera;
    const bool _adaptiveNear = qgetenv("OSG_3DGS_CAMERA_NEAR").isEmpty() &&
        qgetenv("OSG_3DGS_CAMERA_EYE").isEmpty();
};

int osgMouseButton(Qt::MouseButton button)
{
    switch (button) {
    case Qt::LeftButton: return 1;
    case Qt::MiddleButton: return 2;
    case Qt::RightButton: return 3;
    default: return 0;
    }
}

osg::Geometry* createBoundsGeometry()
{
    auto* geometry = new osg::Geometry;
    geometry->setNodeMask(0);
    geometry->setUseDisplayList(false);
    geometry->setUseVertexBufferObjects(true);
    geometry->setDataVariance(osg::Object::DYNAMIC);

    constexpr unsigned short edges[] = {
        0, 1, 2, 3, 4, 5, 6, 7,
        0, 2, 1, 3, 4, 6, 5, 7,
        0, 4, 1, 5, 2, 6, 3, 7
    };
    geometry->addPrimitiveSet(new osg::DrawElementsUShort(GL_LINES, 24, edges));
    auto* state = geometry->getOrCreateStateSet();
    state->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    state->setMode(GL_BLEND, osg::StateAttribute::OFF);
    state->setRenderBinDetails(100, "RenderBin");
    auto* program = new osg::Program;
    program->addShader(new osg::Shader(osg::Shader::VERTEX,
        "#version 330\nlayout(location=0) in vec3 position;\n"
        "uniform mat4 osg_ModelViewProjectionMatrix;\n"
        "void main() { gl_Position = osg_ModelViewProjectionMatrix * vec4(position, 1.0); }"));
    program->addShader(new osg::Shader(osg::Shader::FRAGMENT,
        "#version 330\nout vec4 color;\n"
        "void main() { color = vec4(1.0, 0.78, 0.25, 1.0); }"));
    state->setAttributeAndModes(program, osg::StateAttribute::ON);
    return geometry;
}
}
 


GraphicsWindowQt::GraphicsWindowQt(QWidget *parent)
    : QOpenGLWidget(parent)
{
    _benchmarkEnabled = renderSetting("OSG_3DGS_BENCHMARK", 0, 0, 1) != 0;
    init3D();
    setFocusPolicy(Qt::StrongFocus);
    _selectionOverlay = new SelectionOverlay(this);
    _selectionBand = new QRubberBand(QRubberBand::Rectangle, this);
}
 
GraphicsWindowQt::~GraphicsWindowQt() = default;
 
void GraphicsWindowQt::setKeyboardModifiers(QInputEvent *event)
{
    int modkey = event->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier);
    unsigned int mask = 0;
    if (modkey & Qt::ShiftModifier) {
        mask |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
    }
    if (modkey & Qt::ControlModifier) {
        mask |= osgGA::GUIEventAdapter::MODKEY_CTRL;
    }
    if (modkey & Qt::AltModifier) {
        mask |= osgGA::GUIEventAdapter::MODKEY_ALT;
    }
 
    window->getEventQueue()->getCurrentEventState()->setModKeyMask(mask);
}
 
void GraphicsWindowQt::keyPressEvent(QKeyEvent *event)
{
    const auto key = getKey(event->key(), event->text());
    if (!key) {
        QOpenGLWidget::keyPressEvent(event);
        return;
    }
    setKeyboardModifiers(event);
    if (pObj && event->key() == Qt::Key_C && event->modifiers() == Qt::NoModifier && !event->isAutoRepeat())
        pObj->requestSort();
    window->getEventQueue()->keyPress(key, event->key());
    event->accept();
    update();
}

void GraphicsWindowQt::keyReleaseEvent(QKeyEvent *event)
{
    const auto key = getKey(event->key(), event->text());
    if (!key) {
        QOpenGLWidget::keyReleaseEvent(event);
        return;
    }
    setKeyboardModifiers(event);
    window->getEventQueue()->keyRelease(key, event->key());
    event->accept();
    update();
}

void GraphicsWindowQt::mousePressEvent(QMouseEvent *event)
{
    const int button = osgMouseButton(event->button());
    if (!button) return;
    setFocus(Qt::MouseFocusReason);
    if (_selectionMode && event->button() == Qt::LeftButton) {
        if (!_pressedButtons) {
            _selectionStart = event->pos();
            _selecting = true;
        }
        event->accept();
        return;
    }
    if (_selecting) {
        event->accept();
        return;
    }
    setKeyboardModifiers(event);
    _lastPointer = event->localPos();
    _pressedButtons |= event->button();
    const QPointF pixel = _lastPointer * devicePixelRatioF();
    window->getEventQueue()->mouseButtonPress(pixel.x(), pixel.y(), button);
    event->accept();
    update();
}

void GraphicsWindowQt::mouseReleaseEvent(QMouseEvent *event)
{
    if (_selecting && event->button() == Qt::LeftButton) {
        cancelSelectionDrag();
        finishSelection(event->pos(), event->modifiers());
        event->accept();
        return;
    }
    const int button = osgMouseButton(event->button());
    if (!button || !_pressedButtons.testFlag(event->button())) return;
    setKeyboardModifiers(event);
    _lastPointer = event->localPos();
    _pressedButtons &= ~Qt::MouseButtons(event->button());
    const QPointF pixel = _lastPointer * devicePixelRatioF();
    window->getEventQueue()->mouseButtonRelease(pixel.x(), pixel.y(), button);
    event->accept();
    update();
}

void GraphicsWindowQt::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::MiddleButton) {
        fitModel();
        event->accept();
    } else {
        mousePressEvent(event);
    }
}

void GraphicsWindowQt::mouseMoveEvent(QMouseEvent *event)
{
    if (_selecting) {
        const QRect box = QRect(_selectionStart, event->pos()).normalized();
        _selectionBand->setGeometry(box.intersected(rect()));
        _selectionBand->setVisible((event->pos() - _selectionStart).manhattanLength() >= QApplication::startDragDistance());
        event->accept();
        return;
    }
    if (!_pressedButtons) return;
    setKeyboardModifiers(event);
    _lastPointer = event->localPos();
    const QPointF pixel = _lastPointer * devicePixelRatioF();
    window->getEventQueue()->mouseMotion(pixel.x(), pixel.y());
    event->accept();
    update();
}

void GraphicsWindowQt::wheelEvent(QWheelEvent *event)
{
    if (_selecting) {
        event->accept();
        return;
    }
    const double steps = event->pixelDelta().isNull()
        ? event->angleDelta().y() / 120.0 : event->pixelDelta().y() / 48.0;
    if (steps == 0.0) {
        event->ignore();
        return;
    }
    setKeyboardModifiers(event);
    window->getEventQueue()->mouseScroll2D(0.0f, float(steps));
    event->accept();
    update();
}

void GraphicsWindowQt::focusOutEvent(QFocusEvent* event)
{
    cancelSelectionDrag();
    auto* queue = window->getEventQueue();
    queue->getCurrentEventState()->setModKeyMask(0);
    const QPointF pixel = _lastPointer * devicePixelRatioF();
    for (Qt::MouseButton button : {Qt::LeftButton, Qt::MiddleButton, Qt::RightButton})
        if (_pressedButtons.testFlag(button))
            queue->mouseButtonRelease(pixel.x(), pixel.y(), osgMouseButton(button));
    _pressedButtons = Qt::NoButton;
    QOpenGLWidget::focusOutEvent(event);
}

void GraphicsWindowQt::syncViewport()
{
    const QSize size = this->size() * devicePixelRatioF();
    const int w = std::max(1, size.width());
    const int h = std::max(1, size.height());
    if (_framebufferSize == QSize(w, h)) return;
    _framebufferSize = QSize(w, h);
    window->resized(0, 0, w, h);
    window->getEventQueue()->windowResize(0, 0, w, h);
    if (auto* camera = getCamera()) {
        camera->setViewport(0, 0, w, h);
        double fovy, aspect, nearPlane, farPlane;
        if (camera->getProjectionMatrixAsPerspective(fovy, aspect, nearPlane, farPlane))
            camera->setProjectionMatrixAsPerspective(fovy, double(w) / h, nearPlane, farPlane);
    }
}

void GraphicsWindowQt::resizeEvent(QResizeEvent *event)
{
    QOpenGLWidget::resizeEvent(event);
    syncViewport();
}

void GraphicsWindowQt::closeEvent(QCloseEvent *event)
{
	setDone(true);
}

void GraphicsWindowQt::timerEvent(QTimerEvent *)
{
    if (hasModel()) update();
}
 
void GraphicsWindowQt::paintGL()
{
    if (isVisible()) {
        syncViewport();
        window->setDefaultFboId(defaultFramebufferObject());
        if (!_profileClock.isValid()) _profileClock.start();
        double elapsedSeconds = 0.001 * _profileClock.elapsed();
        bool moving = false;
        bool reportStep = false;
        double stepScheduled = 0.0, stepAngle = 0.0;
        osg::Vec3d stepEye;
        if (_benchmarkEnabled && _benchmarkReady) {
            elapsedSeconds = 0.001 * _benchmarkClock.elapsed();
            // A fixed benchmark still reapplies the scripted camera each frame,
            // preventing incidental mouse input from invalidating A/B captures.
            static const bool fixedBenchmark = renderSetting("OSG_3DGS_BENCHMARK_STATIC", 0, 0, 1) != 0;
            double phase = fixedBenchmark ? 0.0 : std::fmod(elapsedSeconds, 20.0);
            moving = phase >= 10.0;
            double angle = moving ? 0.25 * std::sin((phase - 10.0) * (2.0 * M_PI / 10.0)) : 0.0;
            static const bool stepBenchmark = renderSetting("OSG_3DGS_BENCHMARK_STEP", 0, 0, 1) != 0;
            if (stepBenchmark && !fixedBenchmark) {
                // Hold distinct views after a short move. Returning to the
                // initial view after every motion hides new-view paging costs.
                const double starts[] = {20.0, 40.0, 60.0};
                const double angles[] = {0.0, 0.55, -0.45, 0.0};
                int stepPhase = 0;
                angle = 0.0;
                moving = false;
                for (int step = 0; step < 3; ++step) {
                    if (elapsedSeconds < starts[step]) break;
                    const double t = std::min(1.0, (elapsedSeconds - starts[step]) / 2.0);
                    const double smooth = t * t * (3.0 - 2.0 * t);
                    angle = angles[step] + (angles[step + 1] - angles[step]) * smooth;
                    moving = t < 1.0;
                    stepPhase = 1 + 2 * step + (moving ? 0 : 1);
                    stepScheduled = starts[step] + (moving ? 0.0 : 2.0);
                }
                reportStep = stepPhase != _benchmarkStepPhase;
                _benchmarkStepPhase = stepPhase;
                stepAngle = angle;
            }
            osg::Quat rotation(angle, _benchmarkUp);
            osg::Vec3d eye = _benchmarkCenter + rotation * (_benchmarkEye - _benchmarkCenter);
            stepEye = eye;
            auto* manipulator = static_cast<osgGA::TrackballManipulator*>(getCameraManipulator());
            manipulator->setTransformation(eye, _benchmarkCenter, _benchmarkUp);
        }

        SparkFrameMetrics::beginFrame();
        {
            SparkFrameMetrics::Scope frameTime(SparkFrameMetrics::Frame);
            frame();
        }
        if (reportStep && getCamera()) {
            // Capture the actual view after OSG updates the manipulator. The
            // timestamps bound recovery from the first frame at the new pose.
            std::ostringstream event;
            event.precision(17);
            event << "BENCHMARK_STEP,phase=" << _benchmarkStepPhase
                  << ",scheduled=" << stepScheduled << ",elapsed=" << elapsedSeconds
                  << ",moving=" << (moving ? 1 : 0) << ",angle=" << stepAngle
                  << ",eye=" << stepEye.x() << ":" << stepEye.y() << ":" << stepEye.z()
                  << ",center=" << _benchmarkCenter.x() << ":" << _benchmarkCenter.y() << ":" << _benchmarkCenter.z()
                  << ",up=" << _benchmarkUp.x() << ":" << _benchmarkUp.y() << ":" << _benchmarkUp.z()
                  << ",view=";
            const auto& view = getCamera()->getViewMatrix();
            for (int row = 0; row < 4; ++row)
                for (int column = 0; column < 4; ++column) {
                    if (row || column) event << ":";
                    event << view(row, column);
                }
            std::cout << event.str() << std::endl;
        }
        if (SparkFrameMetrics::enabled() && getCamera()) {
            const osg::Matrixd& view = getCamera()->getViewMatrix();
            if (!_benchmarkEnabled && _hasProfileView) {
                for (int row = 0; row < 4 && !moving; ++row)
                    for (int col = 0; col < 4; ++col)
                        if (std::abs(view(row, col) - _lastProfileView(row, col)) > 1.0e-6) {
                            moving = true;
                            break;
                        }
            }
            _lastProfileView = view;
            _hasProfileView = true;
        }
        SparkFrameMetrics::endFrame(elapsedSeconds, moving);
        if (!_selectedPoints.empty()) updateSelectionOverlay();
    }
}

void GraphicsWindowQt::resetBenchmark()
{
    _profileClock.restart();
    _hasProfileView = false;
    _benchmarkStepPhase = -1;
    if (!_benchmarkEnabled) return;
    auto* manipulator = dynamic_cast<osgGA::TrackballManipulator*>(getCameraManipulator());
    if (!manipulator) return;
    manipulator->setAllowThrow(false);
    manipulator->getTransformation(_benchmarkEye, _benchmarkCenter, _benchmarkUp);
    // An explicit pose is already the requested observation point. Do not
    // rescale it using the older auto-fit benchmark distance setting.
    const double distanceScale = _cameraPoseOverride ? 1.0
        : 0.01 * renderSetting("OSG_3DGS_BENCHMARK_DISTANCE_PERCENT", 100, 10, 400);
    _benchmarkEye = _benchmarkCenter + (_benchmarkEye - _benchmarkCenter) * distanceScale;
    _benchmarkUp.normalize();
    if (SparkFrameMetrics::enabled()) {
        std::cout << "BENCHMARK_CAMERA eye=" << _benchmarkEye.x() << "," << _benchmarkEye.y() << "," << _benchmarkEye.z()
            << " center=" << _benchmarkCenter.x() << "," << _benchmarkCenter.y() << "," << _benchmarkCenter.z()
            << " up=" << _benchmarkUp.x() << "," << _benchmarkUp.y() << "," << _benchmarkUp.z() << std::endl;
    }
    _benchmarkClock.restart();
    _benchmarkReady = true;
}

void GraphicsWindowQt::applyCameraOverrides()
{
    _cameraPoseOverride = false;
    osg::Camera* camera = getCamera();
    auto* manipulator = dynamic_cast<osgGA::TrackballManipulator*>(getCameraManipulator());
    if (!camera || !manipulator) return;

    double fovy, aspectRatio, zNear, zFar;
    camera->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
    fovy = cameraScalarSetting("OSG_3DGS_CAMERA_FOV", fovy, 10.0, 120.0);
    zNear = cameraScalarSetting("OSG_3DGS_CAMERA_NEAR", defaultNearPlane, 0.00001, std::min(100.0, zFar * 0.5));
    camera->setProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

    const bool requestedPose = !qgetenv("OSG_3DGS_CAMERA_EYE").isEmpty()
        || !qgetenv("OSG_3DGS_CAMERA_CENTER").isEmpty()
        || !qgetenv("OSG_3DGS_CAMERA_UP").isEmpty();
    if (requestedPose) {
        osg::Vec3d eye, center, up;
        bool valid = cameraVectorSetting("OSG_3DGS_CAMERA_EYE", eye)
            && cameraVectorSetting("OSG_3DGS_CAMERA_CENTER", center)
            && cameraVectorSetting("OSG_3DGS_CAMERA_UP", up);
        if (valid) {
            osg::Vec3d direction = center - eye;
            const double distance = direction.length();
            const double upLength = up.length();
            valid = std::isfinite(distance) && distance > 1.0e-9
                && std::isfinite(upLength) && upLength > 1.0e-9;
            if (valid) {
                direction /= distance;
                up /= upLength;
                valid = (direction ^ up).length2() > 1.0e-12;
            }
        }
        if (valid) {
            manipulator->setAllowThrow(false);
            manipulator->setHomePosition(eye, center, up, false);
            manipulator->setTransformation(eye, center, up);
            _cameraPoseOverride = true;
            std::cout << "CAMERA_OVERRIDE eye=" << eye.x() << "," << eye.y() << "," << eye.z()
                << " center=" << center.x() << "," << center.y() << "," << center.z()
                << " up=" << up.x() << "," << up.y() << "," << up.z()
                << " fovY=" << fovy << " near=" << zNear << std::endl;
        } else {
            std::cerr << "CAMERA_OVERRIDE ignored: EYE, CENTER and UP must each contain three finite comma-separated values; "
                << "eye and center must differ, and up must be nonzero and not parallel to the view direction" << std::endl;
        }
    } else if (!qgetenv("OSG_3DGS_CAMERA_FOV").isEmpty() || !qgetenv("OSG_3DGS_CAMERA_NEAR").isEmpty()) {
        std::cout << "CAMERA_OVERRIDE fovY=" << fovy << " near=" << zNear << std::endl;
    }
}

bool GraphicsWindowQt::loadModule(const std::string& file)
{
	// 逐级空指针检查，避免链式调用解引用 null
	osg::Camera* cam = getCamera();
	if (cam && cam->getGraphicsContext() && cam->getGraphicsContext()->getState())
		cam->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);

	osg::ref_ptr<osg::Node> pLoadedNode;
	std::unique_ptr<osg_3dgs::GaussianModel> next;
	if (osg_3dgs::GaussianModel::supportsPath(file))
	{
		next = osg_3dgs::GaussianModel::load(file, (QApplication::applicationDirPath() + "/shader").toStdString());
		if (next && cam) pLoadedNode = next->createSceneNode(*cam);
	}
	else
	{
		pLoadedNode = osgDB::readNodeFile(file);
	}

	if (!pLoadedNode) return false;
	cancelSelectionDrag();
	_selectionMode = false;
	unsetCursor();
	_selectedPoints.clear();
	updateSelectionOverlay();
	root->removeChildren(0, root->getNumChildren());
	pObj = std::move(next);

	root->addChild(pLoadedNode);
    updateModelBounds();

	syncViewport();
	applyCameraOverrides();
	if (!_cameraPoseOverride) frameModel();
	resetBenchmark();
	update();
	emit pointEditChanged();
	return true;
}


void GraphicsWindowQt::init3D()
{  
    if (!_bFirstFrame)return;
 
    setCamera(createCamera(0, 0, width(), height()));	 
 
    setCameraManipulator(_benchmarkEnabled
        ? static_cast<osgGA::TrackballManipulator*>(new BenchmarkManipulator) : new ViewerManipulator(getCamera()));
    auto* manipulator = static_cast<osgGA::TrackballManipulator*>(getCameraManipulator());
    manipulator->setAllowThrow(false);
    manipulator->setMinimumDistance(1.0e-6);
    addEventHandler(new osgViewer::StatsHandler);
    addEventHandler(new osgViewer::HelpHandler);
    addEventHandler(new osgGA::StateSetManipulator(this->getCamera()->getOrCreateStateSet()));
    setThreadingModel(osgViewer::Viewer::SingleThreaded);
 
    osg::ref_ptr<osg::Group> scene = new osg::Group;
    scene->addChild(root);
    _boundsGeometry = createBoundsGeometry();
    osg::ref_ptr<osg::Geode> boundsOverlay = new osg::Geode;
    boundsOverlay->addDrawable(_boundsGeometry);
    scene->addChild(boundsOverlay);
    setSceneData(scene);
    realize();
 
    startTimer(renderSetting("OSG_3DGS_FRAME_INTERVAL_MS", 16, 1, 1000), Qt::PreciseTimer);

	_bFirstFrame = false;
}
 
osg::ref_ptr<osg::Camera> GraphicsWindowQt::createCamera(int x, int y, int w, int h)
{
	QSurfaceFormat format;
	format.setRenderableType(QSurfaceFormat::OpenGL);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	format.setSamples(renderSetting("OSG_3DGS_MSAA_SAMPLES", 0, 0, 16));
	setFormat(format);

    window = new osgViewer::GraphicsWindowEmbedded(x, y, w, h);

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowDecoration = false;
    traits->x = x;
    traits->y = y;
    traits->width = std::max(1, w);
    traits->height = std::max(1, h);
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    traits->windowDecoration = false;
 
    traits->setInheritedWindowPixelFormat = true;
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
 
    camera->setGraphicsContext(window);
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    camera->setProjectionMatrixAsPerspective(
        90.0, double(traits->width) / double(traits->height), defaultNearPlane, 10000.0f);
 
	camera->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
    camera->setClearColor(osg::Vec4f(0.0,0.0, 0.0,1.0));  // 这里可以改背景色

	return camera;
}


bool GraphicsWindowQt::hasModel() const
{
    return root->getNumChildren() > 0;
}

namespace {
class SaveFileBuffer : public std::streambuf {
public:
    explicit SaveFileBuffer(QSaveFile& file) : _file(file) {}
private:
    std::streamsize xsputn(const char* data, std::streamsize count) override
    {
        return std::max<qint64>(0, _file.write(data, count));
    }
    QSaveFile& _file;
};
}

bool GraphicsWindowQt::canEditPoints() const
{
    return !_benchmarkEnabled && pObj && pObj->canEdit();
}

bool GraphicsWindowQt::canSaveModel() const
{
    return canEditPoints() && pObj->canSave();
}

bool GraphicsWindowQt::canUndoDeletion() const
{
    return canEditPoints() && pObj->canUndo();
}

bool GraphicsWindowQt::saveModel(const QString& path, QString& error) const
{
    error.clear();
    if (!canSaveModel()) {
        error = tr("There are no editable points to save.");
        return false;
    }
    QSaveFile output(path);
    const auto ioError = [&] { error = output.errorString(); return false; };
    if (!output.open(QIODevice::WriteOnly)) return ioError();
    SaveFileBuffer buffer(output);
    std::ostream stream(&buffer);
    if (!pObj->writePly(stream) || !output.commit()) return ioError();
    return true;
}

void GraphicsWindowQt::setSelectionMode(bool enabled)
{
    if (enabled && !canEditPoints()) return;
    if (_selectionMode == enabled) return;
    _selectionMode = enabled;
    cancelSelectionDrag();
    if (enabled) setCursor(Qt::CrossCursor);
    else unsetCursor();
    setFocus();
}

void GraphicsWindowQt::cancelSelectionDrag()
{
    _selecting = false;
    _selectionBand->hide();
}

void GraphicsWindowQt::finishSelection(const QPoint& end, Qt::KeyboardModifiers modifiers)
{
    if (!canEditPoints()) return;
    const auto clip = getCamera()->getViewMatrix() * getCamera()->getProjectionMatrix();
    const QRectF box = QRectF(_selectionStart, end).normalized().intersected(QRectF(rect()));
    const bool click = (end - _selectionStart).manhattanLength() < QApplication::startDragDistance();
    const osg::Vec2d viewport(width(), height());
    std::vector<int> hits;
    if (click) {
        const int picked = pObj->pickPoint(clip, viewport, {double(end.x()), double(end.y())});
        if (picked >= 0) hits.push_back(picked);
    } else if (!box.isEmpty()) {
        hits = pObj->pickRectangle(clip, viewport, {box.left(), box.top()}, {box.right(), box.bottom()});
    }
    // Editable indices and selections are kept in source order.
    if (modifiers.testFlag(Qt::ControlModifier) || modifiers.testFlag(Qt::ShiftModifier)) {
        std::vector<int> merged;
        if (modifiers.testFlag(Qt::ControlModifier))
            std::set_symmetric_difference(_selectedPoints.begin(), _selectedPoints.end(), hits.begin(), hits.end(), std::back_inserter(merged));
        else
            std::set_union(_selectedPoints.begin(), _selectedPoints.end(), hits.begin(), hits.end(), std::back_inserter(merged));
        hits.swap(merged);
    }
    _selectedPoints.swap(hits);
    updateSelectionOverlay();
    emit pointEditChanged();
}

void GraphicsWindowQt::updateSelectionOverlay()
{
    QPolygonF markers;
    if (!_selectedPoints.empty()) {
        const auto clip = getCamera()->getViewMatrix() * getCamera()->getProjectionMatrix();
        // Bound per-frame overlay work even when a box contains millions of splats.
        const size_t stride = (_selectedPoints.size() + 4095) / 4096;
        for (size_t i = 0; i < _selectedPoints.size(); i += stride) {
            osg::Vec2d pixel;
            if (pObj->projectPoint(_selectedPoints[i], clip, {double(width()), double(height())}, pixel))
                markers << QPointF(pixel.x(), pixel.y());
        }
        _selectionOverlay->setGeometry(rect());
    }
    _selectionOverlay->setMarkers(std::move(markers));
}

void GraphicsWindowQt::clearSelection()
{
    cancelSelectionDrag();
    if (_selectedPoints.empty()) return;
    _selectedPoints.clear();
    updateSelectionOverlay();
    emit pointEditChanged();
}

void GraphicsWindowQt::deleteSelectedPoints()
{
    if (!canEditPoints() || _selectedPoints.empty()) return;
    cancelSelectionDrag();
    pObj->deletePoints(std::move(_selectedPoints));
    _selectedPoints.clear();
    updateModelBounds();
    updateSelectionOverlay();
    update();
    emit pointEditChanged();
}

void GraphicsWindowQt::undoPointDeletion()
{
    if (!canUndoDeletion()) return;
    cancelSelectionDrag();
    _selectedPoints = pObj->undoDeletion();
    updateModelBounds();
    updateSelectionOverlay();
    update();
    emit pointEditChanged();
}

QString GraphicsWindowQt::modelStatus() const
{
    if (!pObj) return hasModel() ? tr("Model loaded") : tr("No model loaded");
    const QLocale locale;
    const auto stats = pObj->stats();
    QString status = tr("Drawn: %1  |  Source: %2")
        .arg(locale.toString(stats.drawnPoints), locale.toString(stats.sourcePoints));
    if (stats.paged) {
        status += tr("  |  Resident pages: %1/%2")
            .arg(locale.toString(stats.residentPages), locale.toString(stats.totalPages));
    } else if (canEditPoints()) {
        status += tr("  |  Selected: %1  |  Deleted: %2")
            .arg(locale.toString(qulonglong(_selectedPoints.size())),
                 locale.toString(stats.deletedPoints));
    }
    return status;
}

void GraphicsWindowQt::resetView()
{
    if (_benchmarkEnabled || !hasModel()) return;
    getCameraManipulator()->home(0.0);
    update();
}

void GraphicsWindowQt::fitModel()
{
    if (_benchmarkEnabled || !hasModel()) return;
    auto* manipulator = static_cast<osgGA::TrackballManipulator*>(getCameraManipulator());
    osg::Vec3d eye, center, up;
    manipulator->getHomePosition(eye, center, up);
    // Keep the initial (possibly explicit) home pose when framing a new view.
    if (frameModel()) {
        manipulator->setHomePosition(eye, center, up, false);
        update();
    }
}

osg::BoundingBox GraphicsWindowQt::modelFrameBounds() const
{
    if (pObj) {
        const auto focus = pObj->focusBounds();
        if (pObj->canEdit() || (pObj->stats().paged && focus.valid())) return focus;
    }
    osg::ComputeBoundsVisitor visitor;
    root->accept(visitor);
    return visitor.getBoundingBox();
}

bool GraphicsWindowQt::canShowModelBounds() const
{
    return pObj && !_benchmarkEnabled;
}

void GraphicsWindowQt::setShowModelBounds(bool show)
{
    _showModelBounds = show;
    updateModelBounds();
    update();
}

void GraphicsWindowQt::updateModelBounds()
{
    const auto bounds = _showModelBounds && canShowModelBounds() ? modelFrameBounds() : osg::BoundingBox();
    _boundsGeometry->setNodeMask(bounds.valid() ? ~0u : 0u);
    if (bounds.valid()) {
        osg::ref_ptr<osg::Vec3Array> corners = new osg::Vec3Array;
        for (unsigned i = 0; i < 8; ++i) corners->push_back(bounds.corner(i));
        _boundsGeometry->setVertexAttribArray(0, corners, osg::Array::BIND_PER_VERTEX);
        _boundsGeometry->setInitialBound(bounds);
    }
}

bool GraphicsWindowQt::frameModel()
{
    const auto bounds = modelFrameBounds();
    auto* manipulator = static_cast<osgGA::TrackballManipulator*>(getCameraManipulator());
    double fovy, aspect, nearPlane, farPlane;
    if (!bounds.valid() || !getCamera()->getProjectionMatrixAsPerspective(fovy, aspect, nearPlane, farPlane))
        return false;
    const double tanY = std::tan(fovy * M_PI / 360.0);
    const double tanX = tanY * aspect;
    if (!(tanX > 0.0 && tanY > 0.0)) return false;

    osg::Vec3d eye, center, up;
    manipulator->getTransformation(eye, center, up);
    osg::Vec3d back = eye - center;
    if (back.normalize() == 0.0) back.set(0.0, 0.0, 1.0);
    osg::Vec3d right = up ^ back;
    if (right.normalize() == 0.0) return false;
    up = back ^ right;
    center = bounds.center();

    // Every corner must fit both frustum axes and remain beyond the near plane.
    const double fill = 0.88;
    double distance = nearPlane * 1.1;
    for (unsigned int i = 0; i < 8; ++i) {
        const osg::Vec3d delta = osg::Vec3d(bounds.corner(i)) - center;
        const double depth = back * delta;
        distance = std::max(distance, depth + std::max({nearPlane * 1.1,
            std::abs(right * delta) / (fill * tanX), std::abs(up * delta) / (fill * tanY)}));
    }
    if (!std::isfinite(distance)) return false;
    manipulator->setHomePosition(center + back * distance, center, up, false);
    manipulator->home(0.0);
    return true;
}

osgGA::GUIEventAdapter::KeySymbol GraphicsWindowQt::getKey(int key, const QString& value)
{
	if (!value.isEmpty())
	{
		char code = value[0].toLatin1();
		if ((code >= 'a' && code <= 'z') || (code >= 'A' && code <= 'Z') ||
			(code >= '0' && code <= '9'))
			return (osgGA::GUIEventAdapter::KeySymbol)code;
	}

	switch (key)
	{
	case Qt::Key_Escape: return osgGA::GUIEventAdapter::KEY_Escape;
	case Qt::Key_Tab: return osgGA::GUIEventAdapter::KEY_Tab;
	case Qt::Key_Backspace: return osgGA::GUIEventAdapter::KEY_BackSpace;
	case Qt::Key_Return: return osgGA::GUIEventAdapter::KEY_Return;
	case Qt::Key_Enter: return osgGA::GUIEventAdapter::KEY_Return;
	case Qt::Key_Insert: return osgGA::GUIEventAdapter::KEY_Insert;
	case Qt::Key_Delete: return osgGA::GUIEventAdapter::KEY_Delete;
	case Qt::Key_Pause: return osgGA::GUIEventAdapter::KEY_Pause;
	case Qt::Key_Print: return osgGA::GUIEventAdapter::KEY_Print;
	case Qt::Key_SysReq: return osgGA::GUIEventAdapter::KEY_Sys_Req;
	case Qt::Key_Clear: return osgGA::GUIEventAdapter::KEY_Clear;
	case Qt::Key_Home: return osgGA::GUIEventAdapter::KEY_Home;
	case Qt::Key_End: return osgGA::GUIEventAdapter::KEY_End;
	case Qt::Key_Left: return osgGA::GUIEventAdapter::KEY_Left;
	case Qt::Key_Up: return osgGA::GUIEventAdapter::KEY_Up;
	case Qt::Key_Right: return osgGA::GUIEventAdapter::KEY_Right;
	case Qt::Key_Down: return osgGA::GUIEventAdapter::KEY_Down;
	case Qt::Key_PageUp: return osgGA::GUIEventAdapter::KEY_Page_Up;
	case Qt::Key_PageDown: return osgGA::GUIEventAdapter::KEY_Page_Down;
	case Qt::Key_CapsLock: return osgGA::GUIEventAdapter::KEY_Caps_Lock;
	case Qt::Key_NumLock: return osgGA::GUIEventAdapter::KEY_Num_Lock;
	case Qt::Key_ScrollLock: return osgGA::GUIEventAdapter::KEY_Scroll_Lock;
	case Qt::Key_F1: return osgGA::GUIEventAdapter::KEY_F1;
	case Qt::Key_F2: return osgGA::GUIEventAdapter::KEY_F2;
	case Qt::Key_F3: return osgGA::GUIEventAdapter::KEY_F3;
	case Qt::Key_F4: return osgGA::GUIEventAdapter::KEY_F4;
	case Qt::Key_F5: return osgGA::GUIEventAdapter::KEY_F5;
	case Qt::Key_F6: return osgGA::GUIEventAdapter::KEY_F6;
	case Qt::Key_F7: return osgGA::GUIEventAdapter::KEY_F7;
	case Qt::Key_F8: return osgGA::GUIEventAdapter::KEY_F8;
	case Qt::Key_F9: return osgGA::GUIEventAdapter::KEY_F9;
	case Qt::Key_F10: return osgGA::GUIEventAdapter::KEY_F10;
	case Qt::Key_F11: return osgGA::GUIEventAdapter::KEY_F11;
	case Qt::Key_F12: return osgGA::GUIEventAdapter::KEY_F12;
	case Qt::Key_Space: return osgGA::GUIEventAdapter::KEY_Space;
	case Qt::Key_Exclam: return osgGA::GUIEventAdapter::KEY_Exclaim;
	case Qt::Key_QuoteDbl: return osgGA::GUIEventAdapter::KEY_Quotedbl;
	case Qt::Key_NumberSign: return osgGA::GUIEventAdapter::KEY_Hash;
	case Qt::Key_Dollar: return osgGA::GUIEventAdapter::KEY_Dollar;
	case Qt::Key_Percent: return (osgGA::GUIEventAdapter::KeySymbol)0x25;  // '%'
	case Qt::Key_Ampersand: return osgGA::GUIEventAdapter::KEY_Ampersand;
	case Qt::Key_Apostrophe: return osgGA::GUIEventAdapter::KEY_Quote;
	case Qt::Key_ParenLeft: return osgGA::GUIEventAdapter::KEY_Leftparen;
	case Qt::Key_ParenRight: return osgGA::GUIEventAdapter::KEY_Rightparen;
	case Qt::Key_Asterisk: return osgGA::GUIEventAdapter::KEY_Asterisk;
	case Qt::Key_Plus: return osgGA::GUIEventAdapter::KEY_Plus;
	case Qt::Key_Comma: return osgGA::GUIEventAdapter::KEY_Comma;
	case Qt::Key_Minus: return osgGA::GUIEventAdapter::KEY_Minus;
	case Qt::Key_Period: return osgGA::GUIEventAdapter::KEY_Period;
	case Qt::Key_Slash: return osgGA::GUIEventAdapter::KEY_Slash;
	case Qt::Key_Colon: return osgGA::GUIEventAdapter::KEY_Colon;
	case Qt::Key_Semicolon: return osgGA::GUIEventAdapter::KEY_Semicolon;
	case Qt::Key_Less: return osgGA::GUIEventAdapter::KEY_Less;
	case Qt::Key_Equal: return osgGA::GUIEventAdapter::KEY_Equals;
	case Qt::Key_Greater: return osgGA::GUIEventAdapter::KEY_Greater;
	case Qt::Key_Question: return osgGA::GUIEventAdapter::KEY_Question;
	case Qt::Key_At: return osgGA::GUIEventAdapter::KEY_At;
	case Qt::Key_BracketLeft: return osgGA::GUIEventAdapter::KEY_Leftbracket;
	case Qt::Key_Backslash: return osgGA::GUIEventAdapter::KEY_Backslash;
	case Qt::Key_BracketRight: return osgGA::GUIEventAdapter::KEY_Rightbracket;
	case Qt::Key_AsciiCircum: return osgGA::GUIEventAdapter::KEY_Caret;
	case Qt::Key_Underscore: return osgGA::GUIEventAdapter::KEY_Underscore;
	case Qt::Key_QuoteLeft: return osgGA::GUIEventAdapter::KEY_Backquote;
	default: break;
	}
	return (osgGA::GUIEventAdapter::KeySymbol)0;
}
