#include "MainWindow.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QElapsedTimer>
#include <QtCore/QMutex>
#include <QtCore/QTimer>
#include <QtWidgets/QAction>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QToolBar>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace RecFusion;


MainWindow::MainWindow()
: m_timer(0)
, m_calibMessageBox(0)
, m_reconstruct(false)
, m_calibrate(false)
, m_rec(0)
, m_sensorManager(0)
{
// Output RecFusion SDK version
std::cout << "Using RecFusionSDK " << RecFusionSDK::majorVersion() << "." << RecFusionSDK::minorVersion() << "." << RecFusionSDK::buildVersion()
 << std::endl;

// Activate license
bool ok = RecFusionSDK::activate("JSVTI-NIOKA-PEXJO-GWUTE-HQPFQ");
if (!ok)
std::cout << "Invalid RecFusion license. Export will be disabled." << std::endl;

RecFusionSDK::init();

// Find available sensors
m_sensorManager = new SensorManager;
int numSensors = m_sensorManager->deviceCount();
std::cout << "Found " << numSensors << " sensors" << std::endl;
if (numSensors < 2)
{
QMessageBox::warning(this, "Initialization", "This sample requires at least two sensors to be connected. Exiting.");
QTimer::singleShot(0, this, SLOT(close()));
return;
}

// Create main window GUI
QGridLayout* l = new QGridLayout;
for (int i = 0; i < numSensors; i++)
{
m_imgLabel.push_back(new QLabel);
m_recLabel.push_back(new QLabel);

l->addWidget(m_imgLabel[i], 0, i);
l->addWidget(m_recLabel[i], 1, i);
}

QWidget* wt = new QWidget;
wt->setLayout(l);
setCentralWidget(wt);

resize(1024, 768);

// Resize all containers and initialize pointers to zero, and
m_colorImg.resize(numSensors, 0);
m_depthImg.resize(numSensors, 0);
m_sceneImg.resize(numSensors, 0);
m_calibImgColor.resize(numSensors, 0);
m_calibImgDepth.resize(numSensors, 0);
m_calibImgValid.resize(numSensors, false);
m_sensor.resize(numSensors, 0);
m_K.resize(numSensors);
m_colorK.resize(numSensors);
m_depthToColorT.resize(numSensors);
m_sensorT.resize(numSensors);

// Instantiate sensor objects
for (int i = 0; i < numSensors; i++)
m_sensor[i] = m_sensorManager->sensor(i);

// Open sensors
for (int i = 0; i < numSensors; i++)
{
ok = m_sensor[i]->open();
if (!ok)
{
QMessageBox::warning(this, "Initialization", "Couldn't open sensor #" + QString::number(i + 1) + ". Exiting.");
QTimer::singleShot(0, this, SLOT(close()));
}
else
{
// Get sensor properties
int cw = m_sensor[i]->colorWidth();
int ch = m_sensor[i]->colorHeight();
int dw = m_sensor[i]->depthWidth();
int dh = m_sensor[i]->depthHeight();
m_K[i] = m_sensor[i]->depthIntrinsics();
m_colorK[i] = m_sensor[i]->colorIntrinsics();
m_depthToColorT[i] = m_sensor[i]->depthToColorTransformation();

// Create color and depth images
m_colorImg[i] = new ColorImage(cw, ch);
m_depthImg[i] = new DepthImage(dw, dh);
m_sceneImg[i] = new ColorImage(dw, dh, 4);
m_calibImgColor[i] = new ColorImage(cw, ch);
m_calibImgDepth[i] = new DepthImage(dw, dh);

m_imgLabel[i]->resize(cw, ch);
}
}

// Set sensor transformation to identity
for (int r = 0; r < 4; ++r)
{
for (int c = 0; c < 4; ++c)
{
for (int i = 0; i < numSensors; i++)
m_sensorT[i](r, c) = (r == c) ? 1 : 0;
}
}

// Create message box for calibration dialog
m_calibMessageBox = new QMessageBox(this);
m_calibMessageBox->setIcon(QMessageBox::Information);
m_calibMessageBox->setWindowTitle("Calibration");
m_calibMessageBox->setText("Press OK to capture calibration frame");
m_calibMessageBox->setDefaultButton(QMessageBox::Ok);
connect(m_calibMessageBox, SIGNAL(accepted()), this, SLOT(performCalibration()));

QToolBar* toolbar = new QToolBar(this);
addToolBar(toolbar);

// Create actions for calibrating and reconstructing
QAction* a;
a = new QAction("Calibrate", this);
a->setShortcut(QKeySequence("F9"));
connect(a, SIGNAL(triggered()), this, SLOT(calibrate()));
addAction(a);
toolbar->addAction(a);

a = new QAction("Save Calibration", this);
a->setShortcut(QKeySequence("F10"));
connect(a, SIGNAL(triggered()), this, SLOT(saveCalibration()));
addAction(a);
toolbar->addAction(a);

a = new QAction("Load calibration", this);
a->setShortcut(QKeySequence("F11"));
connect(a, SIGNAL(triggered()), this, SLOT(loadCalibration()));
addAction(a);
toolbar->addAction(a);

a = new QAction("Start Reconstruction", this);
a->setShortcut(QKeySequence("F5"));
connect(a, SIGNAL(triggered()), this, SLOT(startReconstruction()));
addAction(a);
toolbar->addAction(a);

a = new QAction("Stop Reconstruction", this);
a->setShortcut(QKeySequence("F6"));
connect(a, SIGNAL(triggered()), this, SLOT(stopReconstruction()));
addAction(a);
toolbar->addAction(a);

m_timer = new QTimer(this);
connect(m_timer, SIGNAL(timeout()), this, SLOT(processFrames()));
m_timer->start(50);
}


MainWindow::~MainWindow()
{
int numSensors = (int)m_sensor.size();
for (int i = 0; i < numSensors; i++)
{
// Close sensors
m_sensor[i]->close();

// Delete all allocated data
delete m_colorImg[i];
delete m_depthImg[i];
delete m_sceneImg[i];
delete m_calibImgColor[i];
delete m_calibImgDepth[i];
}

delete m_sensorManager;

delete m_timer;

delete m_rec;

RecFusionSDK::deinit();
}


void MainWindow::calibrate()
{
// Show message box to let user choose correct frame before starting calibration
m_calibrate = false;

m_calibMessageBox->setText("Press OK to capture calibration frames.");
m_calibMessageBox->show();
}


void MainWindow::performCalibration()
{
int numSensors = (int)m_sensor.size();
int numSensorPairs = numSensors - 1;

// Set sensor transformation to identity
for (int r = 0; r < 4; ++r)
{
for (int c = 0; c < 4; ++c)
{
for (int i = 0; i < numSensors; i++)
m_sensorT[i](r, c) = (r == c) ? 1 : 0;
}
}

bool okForAll = true;
std::vector<RecFusion::Mat4> T(numSensorPairs);
for (int sp = 0; sp < numSensorPairs; ++sp)
{
QMessageBox::information(
this, tr("Calibration"), tr("Show the calibration marker to sensors %1 and %2, then press OK").arg(sp + 1).arg(sp + 2));

// Create calibration object for two sensors
Calibration calib;
calib.init(numSensors);

// Single-sided calibration
calib.setMarker(100, 180.1);

//// Two-sided calibration with a marker board of thickness 12 mm. The calibration coordinate system origin is in the middle of the board.

//// Transformation from the first marker coordinate system to the calibration coordinate system.
//// The z-Axis is always perpendicular to the marker surface pointing toward the viewer.
//// When placing the marker such that the marker id printed on the lower left of the marker
//// the x-axis extends from the center of the marker towards the bottom (where the marker id is printed),
//// the y-axis extends to the right, the origin of the marker coordinate system is at the center of the marker.
//
//// Transformation from first marker coordinate system to calibration coordinate system (move 6 mm in z-direction to the center of the
///board). Matrix is column-major-order.
// double T1_[] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, -6, 1 };
// Mat4 T1(T1_);
// calib.setMarker(400, 190, &T1);

//// Transformation from second marker coordinate system to calibration coordinate system (rotate 180?around x-axis and move 6 mm in
///z-direction to the center of the board). Matrix is column-major-order.
// double T2_[] = { 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, -6, 1 };
// Mat4 T2(T2_);
// calib.setMarker(500, 190, &T2);

bool ok = false;

// Try to run calibration until it succeeds but at most 10 times
for (int i = 0; i < 10; ++i)
{
// Reset valid flag for capturing calibration images
m_calibImgValid[sp] = m_calibImgValid[sp + 1] = false;

// Setting m_calibrate to true, instructs the capture loop to capture calibration images
m_calibrate = true;

// Wait until calibration images for both sensors have been captured
while (!m_calibImgValid[sp] && !m_calibImgValid[sp + 1])
QCoreApplication::processEvents();

// Stop calibration frame capturing
m_calibrate = false;

// Pass captured images to calibration
calib.setImage(0, *m_calibImgDepth[sp + 0], *m_calibImgColor[sp + 0], m_K[sp + 0], m_colorK[sp + 0], &m_depthToColorT[sp + 0]);
calib.setImage(1, *m_calibImgDepth[sp + 1], *m_calibImgColor[sp + 1], m_K[sp + 1], m_colorK[sp + 1], &m_depthToColorT[sp + 1]);

// Run calibration
ok = calib.calibrate();

if (ok)
break;
}

if (ok)
{
// Retrieve sensor transformation if calibration succeeded
calib.getTransformation(1, T[sp]);
QMessageBox::information(
this, "Calibration", "Calibration between sensors " + QString::number(sp + 1) + " and " + QString::number(sp + 2) + " succeeded");
}
else
{
QMessageBox::information(
this, "Calibration", "Calibration between sensors " + QString::number(sp + 1) + " and " + QString::number(sp + 2) + " failed");
}

okForAll &= ok;
if (!okForAll)
break;
}

if (okForAll)
{
for (int sp = 0; sp < numSensorPairs; sp++)
m_sensorT[sp + 1] = m_sensorT[sp] * T[sp];
QMessageBox::information(this, "Calibration", "Calibration succeeded");
}
else
{
QMessageBox::information(this, "Calibration", "Calibration failed");
}
}


void MainWindow::saveCalibration()
{
QString filename = QFileDialog::getSaveFileName(this, "Save calibration");
if (filename.isEmpty())
return;

// Save calibrations to file as 4x4 matrices in row-major order
std::ofstream out(filename.toStdString());
for (int i = 0; i < m_sensorT.size(); ++i)
{
for (int r = 0; r < 4; ++r)
{
for (int c = 0; c < 4; ++c)
out << m_sensorT[i](r, c) << " ";
out << std::endl;
}
}
out.close();
}


void MainWindow::loadCalibration()
{
QString filename = QFileDialog::getOpenFileName(this, "Load calibration");
if (filename.isEmpty())
return;
std::ifstream in(filename.toStdString());
if (!in.is_open() || !in.good())
{
QMessageBox::information(this, "Load calibration", "Couldn't open calibration file");
return;
}

// Load calibration from file
std::vector<RecFusion::Mat4> tmp(m_sensor.size());
for (int i = 0; i < m_sensor.size(); ++i)
{
for (int r = 0; r < 4; ++r)
for (int c = 0; c < 4; ++c)
in >> tmp[i](r, c);
}
if (in.fail())
{
QMessageBox::information(this, "Load calibration", "Error reading calibration file");
return;
}
in.close();

m_sensorT = tmp;
}


void MainWindow::startReconstruction()
{
m_reconstruct = false;

// Delete reconstruction object if there is one
delete m_rec;
m_rec = 0;

// Set reconstruction parameters for sensors
ReconstructionParams params((int)m_sensor.size());

// Set per-sensor parameters
for (int i = 0; i < m_sensor.size(); ++i)
{
params.setImageSize(m_colorImg[i]->width(), m_colorImg[i]->height(), m_depthImg[i]->width(), m_depthImg[i]->height(), i);
params.setIntrinsics(m_K[i], i);
params.setColorIntrinsics(m_colorK[i], i);
params.setDepthToColorTransformation(m_depthToColorT[i], i);
}

// params.setVolumePosition(Vec3(0, 200, 1650));
// params.setVolumeResolution(Vec3i(204, 512, 204));
// params.setVolumeSize(Vec3(800, 2000, 800));

// Set volume parameters
params.setVolumePosition(Vec3(230, 0, 1000));
params.setVolumeResolution(Vec3i(360, 512, 360));
params.setVolumeSize(Vec3(700, 1000, 700));

// Create reconstruction object
m_rec = new Reconstruction(params);

// Start reconstruction
m_reconstruct = true;
}


void MainWindow::stopReconstruction()
{
// Stop reconstruction
m_reconstruct = false;
if (!m_rec)
return;

// Get reconstructed mesh
Mesh mesh;
bool ok = m_rec->getMesh(&mesh);

// Delete reconstruction object
delete m_rec;
m_rec = 0;
if (!ok)
{
std::cout << "Couldn't retrieve mesh" << std::endl;
return;
}

std::cout << "Reconstructed mesh (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;

// Define rotation matrix which give 180 rotation by X axsis
Mat3 R;
for (int i = 0; i < 3; i++) {
for (int j = 0; j < 3; j++) {
R(i, j) = 0;
}
}
R(0, 0) = 1;
R(1, 1) = -1;
R(2, 2) = -1;

// Define translation vector if necessary (for repositioning)
Vec3 t(0, 0, 0); // No translation in this example
mesh.transform(R, t);

// Save mesh to file
ok = mesh.save("mesh.ply", Mesh::PLY);
if (ok)
std::cout << "Saved mesh as PLY (" << mesh.vertexCount() << " vertices, " << mesh.triangleCount() << " triangles)" << std::endl;

#ifndef _DEBUG
// Show mesh in viewer
MeshViewer viewer;
viewer.showMesh(&mesh);
#endif
}


void MainWindow::processFrames()
{
for (int i = 0; i < m_sensor.size(); i++)
{
if (!m_depthImg[i] || !m_colorImg[i])
return;
}

// Grab images from sensor
std::vector<bool> ok(m_sensor.size());
for (int i = 0; i < m_sensor.size(); i++)
ok[i] = m_sensor[i]->readImage(*m_depthImg[i], *m_colorImg[i], 40);

// Process images
for (int i = 0; i < m_sensor.size(); ++i)
{
if (!ok[i])
continue;

// Get image size
int cw = m_colorImg[i]->width();
int ch = m_colorImg[i]->height();
int dw = m_depthImg[i]->width();
int dh = m_depthImg[i]->height();

if (m_reconstruct && m_rec)
{
// Add frame to reconstruction
bool status;
bool ret = m_rec->addFrame(i, *m_depthImg[i], *m_colorImg[i], &status, m_sceneImg[i], 0, &m_sensorT[i]);
if (ret && status)
{
// Display rendering of current reconstruction when tracking succeeded
QImage image(m_sceneImg[i]->data(), dw, dh, QImage::Format_RGBA8888);
m_recLabel[i]->setPixmap(QPixmap::fromImage(image).scaled(dw, dh, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
}
}
else if (m_calibrate)
{
// Save calibration frame
memcpy(m_calibImgColor[i]->data(), m_colorImg[i]->data(), cw * ch * 3);
memcpy(m_calibImgDepth[i]->data(), m_depthImg[i]->data(), dw * dh * sizeof(float));
m_calibImgValid[i] = true;
}

// Display captured images in GUI
QImage image(m_colorImg[i]->data(), cw, ch, QImage::Format_RGB888);
m_imgLabel[i]->setPixmap(QPixmap::fromImage(image).scaled(cw, ch, Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
}

// Update GUI
update();
}
