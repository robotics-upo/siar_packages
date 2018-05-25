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
// #include <QUrl>
#include <QMdiArea>
#include <QMdiSubWindow>
#include "rviz/default_plugin/camera_display.h"
#include "rviz/default_plugin/image_display.h"

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
  
  window_1 = configureRVizDisplay(manager_, render_panel_, "base_link", mdiArea);
  configurePointCloud(point_cloud_1,"/front/points");
  configurePointCloud(point_cloud_2,"/front_left/points");
  configurePointCloud(point_cloud_3,"/front_right/points");
//   window_1->resize(640, 480);
  window_1->setWindowTitle("PointClouds");
  
  
  
  window_2 = configureCameraDisplay();
  
  
  window_3 = configureRVizDisplay(manager_2, render_panel_2, "map", mdiArea);
  window_3->setWindowTitle("Map");
  configureMap();
  
  
  
  // End of RVIZ stuff
  resizeWindows();
 
  QMdiArea &q = *mdiArea;
//   q.tileSubWindows();
//   q.cascadeSubWindows();
 
  
  
  // Make Qt connections
  connect(emergencyButton, SIGNAL(released()), node, SLOT(setEmergencyStop())); 
  connect(node, SIGNAL(siarStatusChanged(const siar_driver::SiarStatus &)), this, SLOT(updateSiarStatus(const siar_driver::SiarStatus &)));
  connect(node, SIGNAL(newRSSI(const rssi_get::Nvip_status&)), this, SLOT(updateRSSIStatus(const rssi_get::Nvip_status &)));
  connect(horizontalSlider_width_indicator_2, SIGNAL(valueChanged(int)), node, SLOT(setElecX(int)));
}

void BaseStation::resizeWindows()
{
  QSize size_ = tab_2->size();
  
  window_1->setMinimumWidth(size_.width()*0.66);
  window_1->setMaximumWidth(size_.width()*1.1);
  window_1->setMaximumHeight(size_.height()*1.2);
  window_1->setMinimumHeight(size_.height()*0.75);
  window_2->setMinimumWidth(size_.width()*0.62);
  window_2->setMaximumWidth(size_.width()*1.0);
  window_2->setMaximumHeight(size_.height()*1.2);
  window_2->setMinimumHeight(size_.height()*0.75);
  window_3->setMinimumWidth(size_.width());
  window_3->setMaximumWidth(size_.width()*2);
  window_3->setMaximumHeight(size_.height()*0.8);
  window_3->setMinimumHeight(size_.height()*0.2);
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

QMdiSubWindow *BaseStation::configureRVizDisplay(rviz::VisualizationManager*& manager, rviz::RenderPanel*& panel, 
                                       const string &frame, QMdiArea *parent)
{
  // Configure RVIZ
  // From RVIZ tutorial (http://docs.ros.org/lunar/api/librviz_tutorial/html/index.html) (thanks)
  // First add render panel
  QMdiSubWindow *ret_val = NULL;
  
  if (parent != NULL) {
    panel = new rviz::RenderPanel();
    ret_val = parent->addSubWindow(panel);
    
    // Then the manager
    manager = new rviz::VisualizationManager( panel );
    panel->initialize( manager->getSceneManager(), manager );
    manager->initialize();
    manager->startUpdate();
    manager->setFixedFrame(QString::fromStdString(frame));
    
    
  }
  
  return ret_val;
}

void BaseStation::configurePointCloud(rviz::Display *&pc_display, const std::string &topic)
{
  pc_display = manager_->createDisplay("rviz/PointCloud2", "sensors_msgs::PointCloud2",true);
  pc_display->setTopic(QString::fromStdString(topic), "sensor_msgs/PointCloud2");
  pc_display->subProp("Color Transformer")->setValue("AxisColor");
//   pc_display->initialize(manager_);
}

QMdiSubWindow *BaseStation::configureCameraDisplay() {
  QMdiSubWindow *ret_val = NULL;
//   camera_display = manager_->createDisplay("rviz/Camera", "Front camera",true);
  camera_display = manager_->createDisplay("rviz/Image", "Front camera",true);
  camera_display->setTopic("/front_web/rgb/image_raw", "sensor_msgs/Image");
  camera_display->subProp("Transport Hint")->setValue("compressed");
//   camera_display->subProp("Overlay Alpha")->setValue(0.9);
//   camera_display->subProp("Image Rendering")->setValue("background");
  
//   rviz::CameraDisplay *a = dynamic_cast<rviz::CameraDisplay*>(camera_display);
  rviz::ImageDisplay *a = dynamic_cast<rviz::ImageDisplay*>(camera_display);
  if (a!=NULL) {
//     std::cout << "Here Camera\n";
    
    ret_val = mdiArea->addSubWindow(a->getAssociatedWidget());
//     a->set
  }
  
  
  // Create a Grid display.
  grid_display = manager_->createDisplay( "rviz/Grid", "grid", true );
//   ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_display->subProp( "Line Style" )->setValue( "Lines" );
  grid_display->subProp( "Cell Size" )->setValue(0.2);
  grid_display->subProp( "Plane Cell Count" )->setValue(50);
//   grid_display->subProp( "Color" )->setValue( Qt::yellow );
  
  // Create a robot model display
  robot_model_display = manager_->createDisplay("rviz/RobotModel", "robot model", true);
  
  
  
  return ret_val;
//   image_display = manager_->createDisplay("rviz/Image", "Front image", true);
//   image_display->setTopic("/front_web/rgb/image_raw", "sensor_msgs/Image");
}

void BaseStation::configureMap() {
  sat_view = manager_2->createDisplay("rviz_plugins/AerialMapDisplay","Localization display",true);
  if (sat_view != NULL) {
    std::cout << "Here\nHere\nhere\n";
    sat_view->subProp("Zoom")->setValue(18);
    sat_view->subProp("Object URI")->setValue("http://a.tiles.mapbox.com/v4/mapbox.streets/{z}/{x}/{y}.jpg?access_token=pk.eyJ1IjoiY2h1cnIiLCJhIjoiY2l6dHQzZWJyMDFnZjMzbnA1cDR4MWV3cCJ9.r-YuBsl8JXSBQ_UXTOeSYA");
    sat_view->setTopic("/gps/fix","sensor_msgs/NavSatFix");
    sat_view->subProp("Robot Frame")->setValue("map");
  } else {
    
    std::cerr << "COuld not create satellite view\n";
  }
  render_panel_2->setMaximumHeight(300);
  
  // Create a Grid display.
  grid_display2 = manager_2->createDisplay( "rviz/Grid", "grid", true );
//   ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_display2->subProp( "Line Style" )->setValue( "Lines" );
  grid_display2->subProp( "Cell Size" )->setValue(5.0);
  
  // Sewer graph marker
  marker_1 = manager_2->createDisplay("rviz/Marker", "marker_1", true);
  marker_1->setTopic("/amcl_sewer_node/sewer_graph","visualization_msgs/Marker");
//   marker_1->
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
  if (state.operation_mode == 0) {
    radioButton_manual->setChecked(true);
  } else {
    radioButton_automatic->setChecked(true);
  }
  checkBox_front->setChecked( state.front_light!=0);
  checkBox_rear->setChecked( state.rear_light!=0);
  
  label_width->setText(QString::number(state.width));
  
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



