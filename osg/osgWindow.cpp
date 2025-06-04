#include "osgWindow.h"

#include <QInputEvent>
#include <QApplication>
#include <QDebug>

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
#include <osg/Shape>
#include <osg/TextureBuffer>
#include "osg/Vec2ui"

#include "tools/tools.h"
#include "GaussianDrawObj.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif // !M_PI
 


GraphicsWindowQt::GraphicsWindowQt(QWidget *parent)
    : QOpenGLWidget(parent)
{
    init3D();
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}
 
GraphicsWindowQt::~GraphicsWindowQt(){ 
	delete pObj;
}
 
bool GraphicsWindowQt::event(QEvent *event)
{
	bool handled = QOpenGLWidget::event(event);
	switch (event->type())
	{
	case QEvent::KeyPress: 
	case QEvent::KeyRelease:
	case QEvent::MouseButtonDblClick:
	case QEvent::MouseButtonPress:
	case QEvent::MouseButtonRelease:
	case QEvent::MouseMove:
	case QEvent::Wheel:
		this->update(); break;
	default: break;
	}
	return handled;
}
 
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
    update();
}
 
void GraphicsWindowQt::keyPressEvent(QKeyEvent *event)
{
	osgGA::GUIEventAdapter::KeySymbol key = getKey(event->key(), event->text());
	if ((int)key == 0) return;  // not recognized

	if (event->text() == 'c' || event->text() == 'C')
		pObj->setDirty(true);

	if (event->modifiers() != Qt::NoModifier)
	{
		int modifiers = event->modifiers(); 
		_lastModifiers = modifiers;

		if (modifiers & Qt::ShiftModifier) 
			window->getEventQueue()->keyPress(
			osgGA::GUIEventAdapter::KEY_Shift_L, osgGA::GUIEventAdapter::KEY_Shift_L);

		if (modifiers & Qt::ControlModifier) 
			window->getEventQueue()->keyPress(
			osgGA::GUIEventAdapter::KEY_Control_L, osgGA::GUIEventAdapter::KEY_Control_L);
		if (modifiers & Qt::AltModifier) 
			window->getEventQueue()->keyPress(
			osgGA::GUIEventAdapter::KEY_Alt_L, osgGA::GUIEventAdapter::KEY_Alt_L);
	}
	window->getEventQueue()->keyPress(key, event->key());
 
}
 
void GraphicsWindowQt::keyReleaseEvent(QKeyEvent *event)
{
	osgGA::GUIEventAdapter::KeySymbol key = getKey(event->key(), event->text());
	if ((int)key == 0) return;  // not recognized

	if (_lastModifiers != Qt::NoModifier)
	{
		int modifiers = _lastModifiers; 
		_lastModifiers = 0;

		if (modifiers & Qt::ShiftModifier) 
			window->getEventQueue()->keyRelease(osgGA::GUIEventAdapter::KEY_Shift_L, osgGA::GUIEventAdapter::KEY_Shift_L);
		if (modifiers & Qt::ControlModifier) 
			window->getEventQueue()->keyRelease(osgGA::GUIEventAdapter::KEY_Control_L, osgGA::GUIEventAdapter::KEY_Control_L);
		if (modifiers & Qt::AltModifier) 
			window->getEventQueue()->keyRelease(osgGA::GUIEventAdapter::KEY_Alt_L, osgGA::GUIEventAdapter::KEY_Alt_L);
	}
	window->getEventQueue()->keyRelease(key, event->key());
}
 
void GraphicsWindowQt::mousePressEvent(QMouseEvent *event)
{
    int button = 0;
    switch (event->button()) {
    case Qt::LeftButton: button = 1; break;
    case Qt::MiddleButton: button = 2; break;
    case Qt::RightButton: button = 3; break;
    case Qt::NoButton: button = 0; break;
    default: button = 0; break;
    }
    setKeyboardModifiers(event);
    window->getEventQueue()->mouseButtonPress(event->x(), event->y(), button);
    update();
}
 
void GraphicsWindowQt::mouseReleaseEvent(QMouseEvent *event)
{
    int button = 0;
 
    switch (event->button()) {
    case Qt::LeftButton: button = 1; break;
    case Qt::MiddleButton: button = 2; break;
    case Qt::RightButton: button = 3; break;
    case Qt::NoButton: button = 0; break;
    default: button = 0; break;
    }
 
    setKeyboardModifiers(event);
    window->getEventQueue()->mouseButtonRelease(event->x(), event->y(), button);
 
    QOpenGLWidget::mouseReleaseEvent(event);
    update();
}
 
void GraphicsWindowQt::mouseDoubleClickEvent(QMouseEvent *event)
{
    int button = 0;
    switch (event->button()) {
    case Qt::LeftButton: button = 1; break;
    case Qt::MiddleButton: button = 2; break;
    case Qt::RightButton: button = 3; break;
    case Qt::NoButton: button = 0; break;
    default: button = 0; break;
    }

	if (button == 2) {
		fullScreen();
	}

    setKeyboardModifiers(event);
    window->getEventQueue()->mouseDoubleButtonPress(event->x(), event->y(), button);
 
    QOpenGLWidget::mouseDoubleClickEvent(event);
    update();
}
 
void GraphicsWindowQt::mouseMoveEvent(QMouseEvent *event)
{
    setKeyboardModifiers(event);
    window->getEventQueue()->mouseMotion(event->x(), event->y());
    QOpenGLWidget::mouseMoveEvent(event);
    update();
}
 
void GraphicsWindowQt::wheelEvent(QWheelEvent *event)
{
    setKeyboardModifiers(event);
#if(QT_VERSION < QT_VERSION_CHECK(6,0,0))
    window->getEventQueue()->mouseScroll(
        event->orientation() == Qt::Vertical ?
            (event->delta() > 0 ? osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN) :
            (event->delta() > 0 ? osgGA::GUIEventAdapter::SCROLL_LEFT : osgGA::GUIEventAdapter::SCROLL_RIGHT));
#else
    window->getEventQueue()->mouseScroll(
 
        event->angleDelta().y() != 0 ?
            (event->angleDelta().y() > 0 ? osgGA::GUIEventAdapter::SCROLL_UP : osgGA::GUIEventAdapter::SCROLL_DOWN) :
            (event->angleDelta().x() > 0 ? osgGA::GUIEventAdapter::SCROLL_LEFT : osgGA::GUIEventAdapter::SCROLL_RIGHT));
#endif
    QOpenGLWidget::wheelEvent(event);
    update();
}
 
void GraphicsWindowQt::resizeEvent(QResizeEvent *event)
{
    const QSize &size = event->size();
    window->resized(x(), y(), size.width(), size.height());
    window->getEventQueue()->windowResize(x(), y(), size.width(), size.height());
    window->requestRedraw();
    QOpenGLWidget::resizeEvent(event);
}
 
void GraphicsWindowQt::moveEvent(QMoveEvent *event)
{
    const QPoint &pos = event->pos();
    window->resized(pos.x(), pos.y(), width(), height());
    window->getEventQueue()->windowResize(pos.x(), pos.y(), width(), height());
 
    QOpenGLWidget::moveEvent(event);
}

void GraphicsWindowQt::closeEvent(QCloseEvent *event)
{
	setDone(true);
}

void GraphicsWindowQt::timerEvent(QTimerEvent *)
{
    update();
}
 
void GraphicsWindowQt::paintGL()
{
    if (isVisibleTo(QApplication::activeWindow())) {
        frame();
    }
}
void GraphicsWindowQt::clear()
{
	root->removeChildren(0, root->getNumChildren());
} 

void GraphicsWindowQt::loadModule(const std::string& file,bool bFirst)
{	
	getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);

	root->removeChildren(0, root->getNumChildren());

	osg::ref_ptr<osg::Node>pLoadedNode;
	if(bFirst)
		pLoadedNode = osgDB::readNodeFile(file); 
	else
	{
		setMainCamera(getCamera() );

		pObj = new GaussianDrawObj(file);
		if (pObj)
			pLoadedNode=  pObj->getNode();
	}

    if (!pLoadedNode) return ;

	root->addChild(pLoadedNode);
	fullScreen();

 
}


void GraphicsWindowQt::init3D()
{  
    if (!_bFirstFrame)return;
 
    setCamera(createCamera(0, 0, width(), height()));	 
 
    setCameraManipulator(new osgGA::TrackballManipulator);
    addEventHandler(new osgViewer::StatsHandler);
    addEventHandler(new osgViewer::ThreadingHandler());
    addEventHandler(new osgViewer::HelpHandler);
    addEventHandler(new osgGA::StateSetManipulator(this->getCamera()->getOrCreateStateSet()));
    setThreadingModel(osgViewer::Viewer::SingleThreaded);
 
    setSceneData(root);
    realize();
 
    startTimer(50);

	_bFirstFrame = false;
}
 
osg::ref_ptr<osg::Camera> GraphicsWindowQt::createCamera(int x, int y, int w, int h)
{
	QSurfaceFormat format;
	format.setRenderableType(QSurfaceFormat::OpenGL);
	format.setProfile(QSurfaceFormat::CompatibilityProfile);
	format.setSamples(4); setFormat(format);

    window = new osgViewer::GraphicsWindowEmbedded(x, y, w, h);

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowDecoration = false;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    traits->windowDecoration = false;
 
    traits->setInheritedWindowPixelFormat = true;
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
 
    camera->setGraphicsContext(window);
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    camera->setProjectionMatrixAsPerspective(
        90.0, double(traits->width) / double(traits->height), 0.20f, 1000);
 
	camera->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
    camera->setClearColor(osg::Vec4f(0.0,0.0, 0.0,1.0));  // 这里可以改背景色

	return camera;
}


float getNearDistance(osg::Camera* camera, const osg::BoundingBox& bb, float dist)
{
	float delta = dist * .02;
	float preDist = dist;

	int LOOP_COUNT = 0;
	while (true)
	{
		osg::Quat rot = camera->getViewMatrix().getRotate();
		osg::Matrix proj = camera->getProjectionMatrix();

		osg::Matrix view_m = osg::Matrixd::translate(-bb.center()) *
			osg::Matrixd::rotate(rot) *
			osg::Matrixd::translate(0, 0, -dist);

		float x, y, w, h;
		x = camera->getViewport()->x();
		y = camera->getViewport()->y();
		w = camera->getViewport()->width();
		h = camera->getViewport()->height();

		for (int i = 0; i <= 7; i++)
		{
			osg::Vec3 pp = bb.corner(i) * view_m * proj * camera->getViewport()->computeWindowMatrix();

			if (pp.x() < 0 || pp.x() > w + x || pp.y() < 0 || pp.y() > h + y)
			{
				dist += delta;
				return std::min(dist, preDist);
			}
		}

		dist -= delta;

		if (dist <= 0) return preDist;

		LOOP_COUNT++;
		if (LOOP_COUNT >= 100) return preDist;
	}

	return preDist;
}

void GraphicsWindowQt::fullScreen()
{
	osg::ComputeBoundsVisitor vi;
	root->accept(vi);

	const osg::BoundingBox& bb = vi.getBoundingBox();
 


		auto _cameraManipulator=  getCameraManipulator();

 		osg::Camera* camera =getCamera();

		double fovy, aspectRatio, zNear, zFar;
		camera->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

		osg::Vec3d eye, center, up;
		camera->getViewMatrixAsLookAt(eye, center, up);

		if (!eye.valid() || !center.valid() || !up.valid())
		{
			eye = osg::Vec3(0, 0, 2000);
			center = osg::Vec3();
			up = osg::Vec3(0, 1, 0);
		}

		osg::Vec3d dir = center - eye;
		dir.normalize();
		//double viewDistance = std::max((zFar - zNear) * 0.7, 1.0);

		double tanValue2 = tan(fovy*0.5*M_PI / 180);
		double	distance = bb.radius() / tanValue2;

		double dis = getNearDistance(camera, bb, distance);

		osg::Vec3d homeEye = bb.center() - dir * dis;
		_cameraManipulator->setHomePosition(homeEye, bb.center(), up);
  
	_cameraManipulator->home(0.0);
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
