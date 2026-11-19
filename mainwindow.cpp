#include "mainwindow.h"
#include "osg/osgWindow.h"
#include <QApplication>
#include <QDragEnterEvent>
#include <QFileInfo>
#include <QLabel>
#include <QLocale>
#include <QMenuBar>
#include <QMimeData>
#include <QShortcut>
#include <QStatusBar>
#include <QStyle>
#include <QTimer>
#include <QUrl>
#include <QToolBar>
#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QDebug>

 
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{  
    setWindowTitle(tr("3D Gaussian Viewer"));
    setAcceptDrops(true);
    pOSGWnd = new GraphicsWindowQt(this);
    setCentralWidget(pOSGWnd);
    createActions();
    _viewStatus = new QLabel(this);
    statusBar()->addPermanentWidget(_viewStatus);
    updateViewStatus();
    if (!pOSGWnd->benchmarkEnabled()) {
        auto* timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &MainWindow::updateViewStatus);
        timer->start(350);
    }
}
 
void MainWindow::createActions()
{
    auto* toolbar = addToolBar(tr("Viewer"));
    toolbar->setMovable(false);
    toolbar->setFloatable(false);
    toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    auto* file = menuBar()->addMenu(tr("&File"));
    auto* edit = menuBar()->addMenu(tr("&Edit"));
    auto* view = menuBar()->addMenu(tr("&View"));
    auto makeAction = [this](QStyle::StandardPixmap icon, const QString& text,
                             const QKeySequence& shortcut) {
        auto* action = new QAction(style()->standardIcon(icon), text, this);
        action->setShortcut(shortcut);
        action->setToolTip(QStringLiteral("%1 (%2)").arg(action->iconText(), shortcut.toString(QKeySequence::NativeText)));
        return action;
    };
    auto* open = makeAction(QStyle::SP_DialogOpenButton, tr("&Open"), QKeySequence::Open);
    _saveModelAction = makeAction(QStyle::SP_DialogSaveButton, tr("&Save Model As..."), QKeySequence::Save);
    _captureAction = makeAction(QStyle::SP_DialogSaveButton, tr("Save &Screenshot"), QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_S));
    _fitAction = makeAction(QStyle::SP_TitleBarMaxButton, tr("&Fit Model"), QKeySequence(Qt::Key_F));
    _boundsAction = makeAction(QStyle::SP_TitleBarNormalButton, tr("Show &Bounding Box"), QKeySequence(Qt::Key_B));
    _boundsAction->setCheckable(true);
    _resetAction = makeAction(QStyle::SP_ArrowBack, tr("&Reset View"), QKeySequence(Qt::Key_Home));
    _fullscreenAction = makeAction(QStyle::SP_TitleBarNormalButton, tr("&Full Screen"), QKeySequence(Qt::Key_F11));
    _fullscreenAction->setCheckable(true);
    _selectAction = makeAction(QStyle::SP_ArrowUp, tr("Select &Points"), QKeySequence(Qt::Key_V));
    _selectAction->setCheckable(true);
    _clearSelectionAction = makeAction(QStyle::SP_DialogCancelButton, tr("&Clear Selection"), QKeySequence(Qt::CTRL | Qt::Key_D));
    _deletePointsAction = makeAction(QStyle::SP_TrashIcon, tr("&Delete Points"), QKeySequence::Delete);
    _undoDeletionAction = makeAction(QStyle::SP_ArrowBack, tr("&Undo Deletion"), QKeySequence::Undo);
    file->addAction(open);
    file->addAction(_saveModelAction);
    file->addAction(_captureAction);
    edit->addAction(_selectAction);
    edit->addAction(_clearSelectionAction);
    edit->addSeparator();
    edit->addAction(_deletePointsAction);
    edit->addAction(_undoDeletionAction);
    view->addAction(_fitAction);
    view->addAction(_resetAction);
    view->addAction(_boundsAction);
    view->addSeparator();
    view->addAction(_fullscreenAction);
    toolbar->addAction(open);
    toolbar->addSeparator();
    toolbar->addAction(_fitAction);
    toolbar->addAction(_resetAction);
    toolbar->addSeparator();
    toolbar->addAction(_captureAction);
    toolbar->addAction(_fullscreenAction);
    toolbar->addSeparator();
    toolbar->addAction(_selectAction);
    toolbar->addAction(_deletePointsAction);

    connect(open, &QAction::triggered, this, &MainWindow::fileOpen);
    connect(_saveModelAction, &QAction::triggered, this, &MainWindow::saveModelAs);
    connect(_captureAction, &QAction::triggered, this, &MainWindow::saveScreenshot);
    connect(_fitAction, &QAction::triggered, pOSGWnd, &GraphicsWindowQt::fitModel);
    connect(_boundsAction, &QAction::toggled, pOSGWnd, &GraphicsWindowQt::setShowModelBounds);
    connect(_resetAction, &QAction::triggered, pOSGWnd, &GraphicsWindowQt::resetView);
    connect(_selectAction, &QAction::toggled, pOSGWnd, &GraphicsWindowQt::setSelectionMode);
    connect(_clearSelectionAction, &QAction::triggered, pOSGWnd, &GraphicsWindowQt::clearSelection);
    connect(_deletePointsAction, &QAction::triggered, pOSGWnd, &GraphicsWindowQt::deleteSelectedPoints);
    connect(_undoDeletionAction, &QAction::triggered, pOSGWnd, &GraphicsWindowQt::undoPointDeletion);
    connect(pOSGWnd, &GraphicsWindowQt::pointEditChanged, this, &MainWindow::updateViewStatus);
    connect(_fullscreenAction, &QAction::triggered, this, [this](bool enabled) {
        if (enabled) showFullScreen();
        else showNormal();
        pOSGWnd->setFocus();
    });
    auto* exitFullScreen = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    exitFullScreen->setEnabled(false);
    connect(exitFullScreen, &QShortcut::activated, this, &MainWindow::showNormal);
    connect(_fullscreenAction, &QAction::toggled, exitFullScreen, &QShortcut::setEnabled);
}


void MainWindow::loadFile(const QString& fileName)
{
	if (fileName.isEmpty())
		return;
    const QFileInfo file(fileName);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool loaded = file.isFile() && file.isReadable() &&
        pOSGWnd->loadModule(file.absoluteFilePath().toLocal8Bit().toStdString());
    QApplication::restoreOverrideCursor();
    if (!loaded) {
        statusBar()->showMessage(tr("Could not open %1").arg(file.fileName()), 8000);
        return;
    }
    _currentFile = file.absoluteFilePath();
    setWindowTitle(tr("%1 | 3D Gaussian Viewer").arg(file.fileName()));
    updateViewStatus();
    statusBar()->showMessage(tr("Opened %1").arg(file.fileName()), 3500);
    pOSGWnd->setFocus();
}

void MainWindow::updateViewStatus()
{
    const bool loaded = pOSGWnd->hasModel();
    _fitAction->setEnabled(loaded && !pOSGWnd->benchmarkEnabled());
    _boundsAction->setEnabled(pOSGWnd->canShowModelBounds());
    _resetAction->setEnabled(loaded && !pOSGWnd->benchmarkEnabled());
    _captureAction->setEnabled(loaded);
    _saveModelAction->setEnabled(pOSGWnd->canSaveModel());
    _selectAction->setEnabled(pOSGWnd->canEditPoints());
    _selectAction->setChecked(pOSGWnd->selectionMode());
    _clearSelectionAction->setEnabled(pOSGWnd->hasSelectedPoints());
    _deletePointsAction->setEnabled(pOSGWnd->hasSelectedPoints());
    _undoDeletionAction->setEnabled(pOSGWnd->canUndoDeletion());
    const QString status = pOSGWnd->modelStatus();
    if (_viewStatus->text() != status)
        _viewStatus->setText(status);
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event)
{
    const auto urls = event->mimeData()->urls();
    if (urls.size() == 1 && urls.front().isLocalFile() && QFileInfo(urls.front().toLocalFile()).isFile())
        event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent* event)
{
    const auto urls = event->mimeData()->urls();
    if (urls.size() == 1 && urls.front().isLocalFile()) {
        event->acceptProposedAction();
        loadFile(urls.front().toLocalFile());
    }
}

void MainWindow::changeEvent(QEvent* event)
{
    QMainWindow::changeEvent(event);
    if (event->type() == QEvent::WindowStateChange && _fullscreenAction)
        _fullscreenAction->setChecked(isFullScreen());
}

bool MainWindow::captureFramebuffer(const QString& fileName)
{
	if (!pOSGWnd || fileName.isEmpty())
		return false;
	QImage image = pOSGWnd->grabFramebuffer();
	if (image.isNull())
		return false;
	if (!image.save(fileName))
		return false;
	// Keep the exact camera alongside benchmark captures for reproducible
	// comparisons with other renderers. OSG row-major values can be loaded
	// directly into Three.js column-major Matrix4 storage.
	if (const osg::Camera* camera = pOSGWnd->getCamera()) {
		auto matrixJson = [](const osg::Matrixd& matrix) {
			QJsonArray values;
			for (int row = 0; row < 4; ++row)
				for (int col = 0; col < 4; ++col)
					values.append(matrix(row, col));
			return values;
		};
		QJsonObject metadata;
		metadata.insert("width", image.width());
		metadata.insert("height", image.height());
		metadata.insert("viewMatrix", matrixJson(camera->getViewMatrix()));
		metadata.insert("projectionMatrix", matrixJson(camera->getProjectionMatrix()));
		double fovY, aspect, nearPlane, farPlane;
		if (camera->getProjectionMatrixAsPerspective(fovY, aspect, nearPlane, farPlane)) {
			metadata.insert("fovY", fovY);
			metadata.insert("near", nearPlane);
			metadata.insert("far", farPlane);
		}
		QSaveFile cameraFile(fileName + ".camera.json");
		const QByteArray json = QJsonDocument(metadata).toJson();
		if (!cameraFile.open(QIODevice::WriteOnly) || cameraFile.write(json) != json.size() || !cameraFile.commit())
			qWarning() << "Could not save capture camera:" << cameraFile.fileName();
	}
	return true;
}

void MainWindow::fileOpen()
{
    const QString directory = _currentFile.isEmpty() ? QString() : QFileInfo(_currentFile).absolutePath();
    loadFile(QFileDialog::getOpenFileName(this, tr("Open Model"), directory,
        tr("Gaussian splats (*.ply *.splat *.spz *.ksplat *.rad);;All files (*)")));
}

void MainWindow::saveScreenshot()
{
    const QString path = QFileDialog::getSaveFileName(this, tr("Save Screenshot"),
        QFileInfo(_currentFile).completeBaseName() + QStringLiteral(".png"), tr("PNG image (*.png)"));
    if (path.isEmpty()) return;
    const bool saved = captureFramebuffer(path);
    statusBar()->showMessage(saved ? tr("Saved %1").arg(QFileInfo(path).fileName())
                                   : tr("Could not save screenshot"), 6000);
    pOSGWnd->setFocus();
}

void MainWindow::saveModelAs()
{
    if (!pOSGWnd->canSaveModel()) return;
    const QFileInfo source(_currentFile);
    QFileDialog dialog(this, tr("Save Model As"), source.absolutePath(), tr("Gaussian PLY (*.ply)"));
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setDefaultSuffix(QStringLiteral("ply"));
    dialog.selectFile(source.completeBaseName() + QStringLiteral("-edited.ply"));
    if (dialog.exec() != QDialog::Accepted) return;

    const QString path = dialog.selectedFiles().first();
    const QFileInfo target(path);
    QString error;
    if (target.suffix().compare(QStringLiteral("ply"), Qt::CaseInsensitive) != 0) {
        error = tr("Choose a file name ending in .ply.");
    } else if (target.exists() && target.canonicalFilePath().compare(source.canonicalFilePath(),
#ifdef Q_OS_WIN
                                                                   Qt::CaseInsensitive
#else
                                                                   Qt::CaseSensitive
#endif
                                                                   ) == 0) {
        error = tr("Choose a different file name to preserve the source model.");
    } else {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        const bool saved = pOSGWnd->saveModel(path, error);
        QApplication::restoreOverrideCursor();
        if (saved) statusBar()->showMessage(tr("Saved model: %1").arg(target.fileName()), 6000);
    }
    if (!error.isEmpty()) QMessageBox::warning(this, tr("Could Not Save Model"), error);
    pOSGWnd->setFocus();
}

MainWindow::~MainWindow() 
{
}
