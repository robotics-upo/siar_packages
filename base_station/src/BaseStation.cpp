/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2014  sinosuke <email>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "BaseStation.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QRadioButton>
#include <boost/lexical_cast.hpp>
#include <QStatusBar>
#include <qwt/qwt_dial_needle.h>
#include <qwt/qwt_dial.h>
#include <QVBoxLayout>
#define BUFFER_LENGTH 100000
#include <QUrl>

using namespace std;
using namespace functions;
using boost::lexical_cast;
using boost::bad_lexical_cast;

BaseStation::BaseStation(int argc, char **argv, QWidget* parent, Qt::WindowFlags flags): 
QMainWindow(parent, flags), argc(argc), argv(argv), init_log_time(), curves(), curves_z(), distance_log(), distance_log_z(), position_log(),
t_log(), xy_dist(1.2), z_dist(0.3), node(NULL), uavs(), pos_log(), time_step(0.1), timer(NULL), count(0), started(false)
{
  setupUi(this);
  QwtDialSimpleNeedle *nd = new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, Qt::white, Qt::red);
  
  QwtDial &d = *Dial_speed; 
  
  d.setNeedle(nd);
  d.setMode(QwtDial::RotateNeedle);
  d.setRange( 0.0, 1.0,  0.2);
  d.setScale(135,45, 0.2);
  d.setScaleArc(135,-135);
  
  
  // Start ROS comms
  startComms();
  
  // RViz stuff
  configureRVizDisplay(manager_, render_panel_, "base_link", mdiArea);
  configurePointCloud(point_cloud_1,"/front/points");
  configurePointCloud(point_cloud_2,"/front_left/points");
  configurePointCloud(point_cloud_3,"/front_right/points");
  
  configureRVizDisplay(manager_2, render_panel_2, "/map", tab->layout());
  configureMap();
  // End of RVIZ stuff
  
//   web_view = new QWebView(this);
//   horizontalLayout_2->addWidget(web_view);
//   mdiArea->;
//   QMdiArea &a = *mdiArea;
//   a.addSubWindow(web_view);
//   a.
  
//   QUrl u("http://192.168.168.11:8080/stream_viewer?topic=/front_web/rgb/image_raw");
//   web_view->setUrl(u);
  
  // Make Qt connections
  connect(emergencyButton, SIGNAL(released()), node, SLOT(setEmergencyStop())); 
  connect(node, SIGNAL(siarStatusChanged(const siar_driver::SiarStatus &)), this, SLOT(updateSiarStatus(const siar_driver::SiarStatus &)));
  connect(node, SIGNAL(newRSSI(const rssi_get::Nvip_status&)), this, SLOT(updateRSSIStatus(const rssi_get::Nvip_status &)));
}

BaseStation::BaseStation(const QMainWindow& ): QMainWindow()
{

}

void BaseStation::configureRVizDisplay(rviz::VisualizationManager*& manager, rviz::RenderPanel*& panel, 
                                       const string &frame, QLayout *parent)
{
  // Configure RVIZ
  // From RVIZ tutorial (http://docs.ros.org/lunar/api/librviz_tutorial/html/index.html) (thanks)
  // First add render panel
  panel = new rviz::RenderPanel();
  if (parent != NULL) {
    parent->addWidget(panel);
  }
  
  // Then the manager
  manager = new rviz::VisualizationManager( panel );
  panel->initialize( manager->getSceneManager(), manager );
  manager->initialize();
  manager->startUpdate();
  manager->setFixedFrame(QString::fromStdString(frame));
}

void BaseStation::configureRVizDisplay(rviz::VisualizationManager*& manager, rviz::RenderPanel*& panel, 
                                       const string &frame, QMdiArea *parent)
{
  // Configure RVIZ
  // From RVIZ tutorial (http://docs.ros.org/lunar/api/librviz_tutorial/html/index.html) (thanks)
  // First add render panel
  panel = new rviz::RenderPanel();
  if (parent != NULL) {
    parent->addSubWindow(panel);
  }
  
  // Then the manager
  manager = new rviz::VisualizationManager( panel );
  panel->initialize( manager->getSceneManager(), manager );
  manager->initialize();
  manager->startUpdate();
  manager->setFixedFrame(QString::fromStdString(frame));
}

void BaseStation::configurePointCloud(rviz::Display *&pc_display, const std::string &topic)
{
  pc_display = manager_->createDisplay("rviz/PointCloud", "sensors_msgs::PointCloud",true);
  pc_display->setTopic(QString::fromStdString("topic"), "sensor_msgs/PointCloud");
  pc_display->setProperty("Color Transformer", "Axis");
//   pc_display->initialize(manager_);
}

void BaseStation::configureCameraDisplay() {
  camera_display = manager_->createDisplay("rviz/Camera", "Front camera",true);
  camera_display->setTopic("/front_web/rgb/image_raw", "sensor_msgs/Image");
//   mdiArea->addSubWindow(camera_display);
//   image_display = manager_->createDisplay("rviz/Image", "Front image", true);
//   image_display->setTopic("/front_web/rgb/image_raw", "sensor_msgs/Image");
}

void BaseStation::configureMap() {
  sat_view = manager_2->createDisplay("rviz_plugins/AerialMapDisplay","Localization display",true);
  if (sat_view != NULL) {
    sat_view->setProperty("Zoom",18);
    sat_view->setProperty("Object URI", "http://a.tiles.mapbox.com/v4/mapbox.streets/{z}/{x}/{y}.jpg?access_token=pk.eyJ1IjoiY2h1cnIiLCJhIjoiY2l6dHQzZWJyMDFnZjMzbnA1cDR4MWV3cCJ9.r-YuBsl8JXSBQ_UXTOeSYA");
    sat_view->setProperty("Topic", "/gps/fix");
  }
  render_panel_2->setMaximumHeight(200);
}


BaseStation::~BaseStation()
{
  stopComms();
  
  // Erase plots
//   clearLogs();
}

bool BaseStation::startComms()
{
  // First configure the ROS connections
  if (node == NULL) {
    // First create the node
    node = new Comms(argc, argv);
    started = true;
  }
  
  return true;
}

void BaseStation::stopComms()
{
  if (started) {
    started = false;
    node->shutdownComms();
  }
  delete timer;
  timer = NULL;
}

void BaseStation::updateValues()
{
  FormattedTime t;
  t.getTime();
  
//   ostringstream os;
//   os << "Getting Values " << count;
//   statusBar()->showMessage(QString(os.str().c_str()));
   
  t_log.push_back(t - init_log_time);
  count++;
}

void BaseStation::updateSiarStatus(const siar_driver::SiarStatus& state)
{
  progressBar_4_elec_bat->setValue(  state.elec_battery.percentage);
  progressBar_5_motor_bat->setValue(  state.motor_battery.percentage);
  horizontalSlider_width_indicator->setValue(state.width);
  if (!state.reverse) {
    radioButton_forward->setChecked(true);
    
  } else {
    radioButton_reverse->setChecked(true);
  }
  if (state.slow) {
    radioButton_slow->setChecked(true);
  } else {
    radioButton_normal->setChecked(true);
  }
  
  QwtDial &d = *Dial_speed;
  d.setValue(fabs(state.speed));
}

void BaseStation::updateRSSIStatus(const rssi_get::Nvip_status& status)
{
  progressBar_rssi->setValue(status.rssi_perc);
}


// QColor BaseStation::selectColor(unsigned int i)
// {
//   QColor ret = Qt::blue;
//   switch (i%8) {
//     case 0:
//     ret = Qt::blue;
//     break;
//     case 1:
//     ret = Qt::red;
//     break;
//     case 2:
//     ret = Qt::green;
//     break;
//     case 3:
//     ret = Qt::cyan;
//     break;
//     case 4:
//     ret = Qt::magenta;
//     break;
//     case 5:
//     ret = Qt::yellow;
//     break;
//     case 6:
//     ret = Qt::black;
//     break;
//     case 7:
//     default:
//     ret = Qt::gray;
//     break;
//   }
//   return ret;
// }



