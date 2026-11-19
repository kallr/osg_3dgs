#ifndef GRAPHICSWINDOWQT_H
#define GRAPHICSWINDOWQT_H
 
#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <osgViewer/GraphicsWindow>
#include <memory>
#include <vector>
 
class QInputEvent;
namespace osg_3dgs { class GaussianModel; }
class QRubberBand;
class SelectionOverlay;


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
    void timerEvent(QTimerEvent *);
    void focusOutEvent(QFocusEvent* event) override;
public:
    bool loadModule(const std::string& file);
    void fitModel();
    bool canShowModelBounds() const;
    void setShowModelBounds(bool show);
    void resetView();
    bool hasModel() const;
    bool benchmarkEnabled() const { return _benchmarkEnabled; }
    QString modelStatus() const;
    bool canEditPoints() const;
    bool canSaveModel() const;
    bool saveModel(const QString& path, QString& error) const;
    bool selectionMode() const { return _selectionMode; }
    bool hasSelectedPoints() const { return !_selectedPoints.empty(); }
    bool canUndoDeletion() const;
    void setSelectionMode(bool enabled);
    void clearSelection();
    void deleteSelectedPoints();
    void undoPointDeletion();
signals:
    void pointEditChanged();
private:
    void finishSelection(const QPoint& end, Qt::KeyboardModifiers modifiers);
    void updateSelectionOverlay();
    void cancelSelectionDrag();
    osg::BoundingBox modelFrameBounds() const;
    void updateModelBounds();
	osg::ref_ptr<osg::Camera> createCamera(int x, int y, int w, int h);
	osgGA::GUIEventAdapter::KeySymbol getKey(int key, const QString& value);
	void syncViewport();
	bool frameModel();
	void init3D();
	void resetBenchmark();
	void applyCameraOverrides();
private:
	osg::ref_ptr<osg::Group> root = new osg::Group;
    osg::ref_ptr<osg::Geometry> _boundsGeometry;
    bool _showModelBounds = false;
	osgViewer::GraphicsWindow *window = nullptr;
	bool _bFirstFrame = true;
    Qt::MouseButtons _pressedButtons = Qt::NoButton;
    QPointF _lastPointer;
    QSize _framebufferSize;
    bool _selectionMode = false;
    bool _selecting = false;
    QPoint _selectionStart;
    QRubberBand* _selectionBand = nullptr;
    SelectionOverlay* _selectionOverlay = nullptr;
    std::vector<int> _selectedPoints;
	bool _benchmarkEnabled = false;
	bool _benchmarkReady = false;
	int _benchmarkStepPhase = -1;
	bool _cameraPoseOverride = false;
	bool _hasProfileView = false;
	QElapsedTimer _benchmarkClock;
	QElapsedTimer _profileClock;
	osg::Vec3d _benchmarkEye, _benchmarkCenter, _benchmarkUp;
	osg::Matrixd _lastProfileView;
	std::unique_ptr<osg_3dgs::GaussianModel> pObj;
};
 
#endif // GRAPHICSWINDOWQT_H
