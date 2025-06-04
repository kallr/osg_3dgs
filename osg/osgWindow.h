#ifndef GRAPHICSWINDOWQT_H
#define GRAPHICSWINDOWQT_H
 
#include <QOpenGLWidget>
#include <osgViewer/Viewer>
#include <osgViewer/GraphicsWindow>
 
class QInputEvent;
struct MI_TreeItem;
class GaussianDrawObj;


// qt +osg viewer 
class GraphicsWindowQt : public QOpenGLWidget, public osgViewer::Viewer
{
    Q_OBJECT 
public:
    GraphicsWindowQt(QWidget *parent = 0);
    ~GraphicsWindowQt();
 
    virtual void paintGL();
 	virtual void closeEvent(QCloseEvent *event);

    void setKeyboardModifiers(QInputEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

    void resizeEvent(QResizeEvent *event);
    void moveEvent(QMoveEvent *event);
    void timerEvent(QTimerEvent *);
	bool event(QEvent *e);
public:
    void loadModule(const std::string& file,bool bFirst);
	void fullScreen();
private:
	osg::ref_ptr<osg::Camera> createCamera(int x, int y, int w, int h);
	osgGA::GUIEventAdapter::KeySymbol getKey(int key, const QString& value);
	void clear();
	void init3D();
private:
	osg::ref_ptr<osg::Group> root = new osg::Group;
	osgViewer::GraphicsWindow *window = nullptr;
	bool _bFirstFrame = true;
	int _lastModifiers;  //键盘标志
	GaussianDrawObj* pObj = nullptr;//高斯泼溅对象
};
 
#endif // GRAPHICSWINDOWQT_H
