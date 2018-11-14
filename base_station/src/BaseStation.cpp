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

#include <QStatusBar>
#include <QVBoxLayout>
#define BUFFER_LENGTH 100000
// #include <QUrl>
#include <QMdiArea>
#include <QMdiSubWindow>

#include <qwt/qwt_dial_needle.h>

#ifndef Q_MOC_RUN
#include "rviz/default_plugin/camera_display.h"
#include "rviz/default_plugin/image_display.h"
#include "rviz/view_controller.h"
#include "rviz/default_plugin/view_controllers/xy_orbit_view_controller.h"
#include <boost/lexical_cast.hpp>
#endif

#include "OGRE/OgreCamera.h"

using namespace std;
using namespace functions;
using boost::lexical_cast;
using boost::bad_lexical_cast;

BaseStation::BaseStation(int argc, char **argv, QWidget* parent, Qt::WindowFlags flags): 
QMainWindow(parent, flags), argc(argc), argv(argv), init_log_time(), node(NULL), started(false)
{
  setupUi(this);
  QwtDialSimpleNeedle *nd = new QwtDialSimpleNeedle(QwtDialSimpleNeedle::Arrow, Qt::white, Qt::red);
  
  QwtDial &d = *Dial_speed; 
  
  d.setNeedle(nd);
  d.setMode(QwtDial::RotateNeedle);
//   d.setScaleArc(  0.0, 1.0);
  d.setScale( 0.0, 1.0);
  d.setScaleArc(315,35);
  
  
  // Start ROS comms
  startComms();
  
  
  mdiArea->setFocusPolicy(Qt::ClickFocus);
  // RViz stuff
  
  window_1 = configureRVizDisplay(manager_, render_panel_, "base_link", mdiArea);
  configurePointCloud(point_cloud_1,"/front/points", manager_);
  configurePointCloud(point_cloud_2,"/front_left/points", manager_);
  configurePointCloud(point_cloud_3,"/front_right/points", manager_);
//   window_1->resize(640, 480);
  window_1->setWindowTitle("PointClouds");
  
  window_2 = configureImageDisplay("/inspection1_cam/image_raw", "Inpection1");
  window_img_2 = configureImageDisplay("/inspection2_cam/image_raw", "Inpection2");
  window_img_3 = configureImageDisplay("/flip_image", "Thermal");
  
  // Create a Grid display. 
  grid_display = manager_->createDisplay( "rviz/Grid", "grid", true );
//   ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_display->subProp( "Line Style" )->setValue( "Lines" );
  grid_display->subProp( "Cell Size" )->setValue(0.2); 
  grid_display->subProp( "Plane Cell Count" )->setValue(50);
//   grid_display->subProp( "Color" )->setValue( Qt::yellow );
  
  // Create a robot model display
  robot_model_display = manager_->createDisplay("rviz/MarkerArray", "marker", true);
  robot_model_display->setTopic("/siar_model", "visualization_msgs/MarkerArray");
  
  
  window_3 = configureRVizDisplay(manager_2, render_panel_2, "map", mdiArea);
  window_3->setWindowTitle("Map");
  configureMap();
  
  CameraSettings cloud_cam;
  cloud_cam.getSettings("cloud");
  cloud_camera_ = cloud_cam;
  CameraSettings map_cam("map", 300, M_PI, 1.53, 50, -100, -40);
  map_cam.getSettings("map");
  map_camera_ = map_cam;
  
  ROS_INFO("Camera cloud settings: %s" , cloud_camera_.toString().c_str());
  ROS_INFO("Map camera settings: %s" , map_camera_.toString().c_str());
  
  // Third display:
  window_unused = configureRVizDisplay(manager_3, render_panel_3, "base_link", mdiArea);
  window_unused->showMinimized();
  window_unused->setWindowTitle("Processed Points");
  window_cam = configureCameraDisplay();
  configurePointCloud(point_cloud_defects, "/serviceabilityProblems", manager_3);
  point_cloud_defects->subProp("Size (m)")->setValue(0.05);
  configurePointCloud(p_c_curb, "/curbCloud", manager_3);
  p_c_curb->subProp("Size (m)")->setValue(0.05);
  configurePointCloud(p_c_gutter, "/gutterCloud", manager_3);
  p_c_curb->subProp("Color Transformer")->setValue("FlatColor");
  p_c_curb->subProp("Color")->setValue("#0000FF");
  p_c_gutter->subProp("Size (m)")->setValue(0.05);
  window_cam->setWindowTitle("Front Camera");
  p_c_gutter->subProp("Color Transformer")->setValue("FlatColor");
  p_c_gutter->subProp("Color")->setValue("#777777");
  // End of RVIZ stuff
  
  // Tree widget!!
  tree_widget = new QTreeWidget(this);
  window_4 = mdiArea->addSubWindow(tree_widget);
  window_4->showMinimized();
  window_4->setWindowTitle("Database");
  
  
  window_1->setFocusPolicy(Qt::ClickFocus);
  window_2->setFocusPolicy(Qt::ClickFocus);
  window_img_2->setFocusPolicy(Qt::ClickFocus);
  window_3->setFocusPolicy(Qt::ClickFocus);
  window_4->setFocusPolicy(Qt::ClickFocus);
  window_cam->setFocusPolicy(Qt::ClickFocus);
  window_unused->setFocusPolicy(Qt::ClickFocus);
  
  setExploreView();
  
  setRvizExplorationView(false);
  last_status.reverse = false;
 
  QMdiArea &q = *mdiArea;
//   q.tileSubWindows();
//   q.cascadeSubWindows();
  
  
  
  // Make Qt connections
  connect(actionExploration, SIGNAL(triggered()), this, SLOT(setExploreView()));
  connect(actionAlert, SIGNAL(triggered()), this, SLOT(setMissionView()));
  connect(actionServiceability, SIGNAL(triggered()), this, SLOT(setServiceabilityView()));
  connect(actionInspection, SIGNAL(triggered()), this, SLOT(setInspectionView()));
  connect(emergencyButton, SIGNAL(clicked()), node, SLOT(setEmergencyStop())); 
  connect(node, SIGNAL(siarStatusChanged(const siar_driver::SiarStatus &)), this, SLOT(updateSiarStatus(const siar_driver::SiarStatus &)));
  connect(node, SIGNAL(newRSSI(const rssi_get::Nvip_status&)), this, SLOT(updateRSSIStatus(const rssi_get::Nvip_status &)));
  connect(node, SIGNAL(alertDBReceived(const std::string&)), this, SLOT(updateTreeContent(const std::string&)));
  connect(horizontalSlider_width_indicator_2, SIGNAL(valueChanged(int)), node, SLOT(setElecX(int)));
  connect(checkBox_Curb, SIGNAL(toggled(bool)), p_c_curb, SLOT(setEnabled(bool)));
  connect(checkBox_Gutter, SIGNAL(toggled(bool)), p_c_gutter, SLOT(setEnabled(bool)));
  connect(checkBox_Defects, SIGNAL(toggled(bool)), point_cloud_defects, SLOT(setEnabled(bool)));
  connect(checkBox, SIGNAL(toggled(bool)), marker_section, SLOT(setEnabled(bool)));
  
  // Setviews:
  // Set the view
  sleep(1);
  rviz::ViewController *v = render_panel_2->getViewController();
  map_camera_.setSettings(v);
  
  v = render_panel_->getViewController();
  cloud_camera_.setSettings(v);
  
}

void BaseStation::setExploreView()
{
  QSize size_ = this->size();
  
  window_1->showNormal();
  window_2->showMinimized();
  window_3->showNormal();
  window_4->showMinimized();
  window_cam->showNormal();
  window_img_2->showMinimized();
  window_img_3->showMinimized();
  
  window_1->setMinimumWidth(size_.width()*0.2);
  window_1->resize(size_.width()*0.4, size_.height()*0.55);
  window_1->setMaximumWidth(size_.width()*2.1);
  window_1->setMaximumHeight(size_.height()*2.2);
  window_1->setMinimumHeight(size_.height()*0.2);
  window_cam->setMinimumWidth(size_.width()*0.1);
  window_cam->setMaximumWidth(size_.width()*2.0);
  window_cam->resize(size_.width()*0.4, size_.height()*0.55);
  window_cam->setMaximumHeight(size_.height()*2.2);
  window_cam->setMinimumHeight(size_.height()*0.1);
  window_3->setMinimumWidth(size_.width()*0.2);
  window_3->setMaximumWidth(size_.width()*3);
  window_3->setMaximumHeight(size_.height()*1.2);
  window_3->setMinimumHeight(size_.height()*0.1);
  window_3->resize(size_.width()*0.8, size_.height()*0.4);
  
  window_1->move(0,0);
  window_cam->move(size_.width()*0.4,0);
  window_3->move(0, size_.height()*0.54);
}

void BaseStation::setInspectionView() {
  QSize size_ = this->size();
  
  window_1->showNormal();
  window_2->showNormal();
  window_3->showMinimized();
  window_4->showMinimized();
  window_cam->showNormal();
  window_img_2->showNormal();
  window_img_3->showNormal();
  
  window_2->setMinimumWidth(size_.width()*0.2);
  window_2->resize(size_.width()*0.4, size_.height()*0.5);
  window_2->setMaximumWidth(size_.width()*2.1);
  window_2->setMaximumHeight(size_.height()*2.2);
  window_2->setMinimumHeight(size_.height()*0.2);
  
  window_img_2->setMinimumWidth(size_.width()*0.2);
  window_img_2->resize(size_.width()*0.4, size_.height()*0.5);
  window_img_2->setMaximumWidth(size_.width()*2.1);
  window_img_2->setMaximumHeight(size_.height()*2.2);
  window_img_2->setMinimumHeight(size_.height()*0.2);
  
  window_cam->setMinimumWidth(size_.width()*0.1);
  window_cam->setMaximumWidth(size_.width()*2.0);
  window_cam->resize(size_.width()*0.4, size_.height()*0.5);
  window_cam->setMaximumHeight(size_.height()*2.2);
  window_cam->setMinimumHeight(size_.height()*0.1);
  window_img_3->setMinimumWidth(size_.width()*0.1);
  window_img_3->setMaximumWidth(size_.width()*2);
  window_img_3->setMaximumHeight(size_.height()*2.2);
  window_img_3->setMinimumHeight(size_.height()*0.01);
  window_img_3->resize(size_.width()*0.2, size_.height()*0.5);
  
  window_1->setMinimumWidth(size_.width()*0.1);
  window_1->resize(size_.width()*0.2, size_.height()*0.5);
  window_1->setMaximumWidth(size_.width()*2.1);
  window_1->setMaximumHeight(size_.height()*2.2);
  window_1->setMinimumHeight(size_.height()*0.2);
  
  window_2->move(0,0);
  window_img_2->move(0,size_.height()*0.5);
  window_cam->move(size_.width()*0.4,0);
  window_img_3->move(size_.width()*0.4, size_.height()*0.5);
  window_1->move(size_.width()*0.6, size_.height()*0.5);
  
}

void BaseStation::setMissionView()
{
  QSize size_ = this->size();
  
  window_1->showMinimized();
  window_2->showMinimized();
  window_3->showNormal();
  window_4->showNormal();
  window_cam->showMinimized();
  
  window_3->setMinimumWidth(size_.width()*0.2);
  window_3->setMaximumWidth(size_.width()*3);
  window_3->setMaximumHeight(size_.height()*2.2);
  window_3->setMinimumHeight(size_.height()*0.1);
  window_3->resize(size_.width()*0.6, size_.height()*0.9);
  window_3->move(0, 0);
  
  window_4->setMinimumWidth(size_.width()*0.2);
  window_4->setMaximumWidth(size_.width()*3);
  window_4->setMaximumHeight(size_.height()*2.2);
  window_4->setMinimumHeight(size_.height()*0.1);
  window_4->resize(size_.width()*0.2, size_.height()*0.9);
  
  window_4->move(size_.width()*0.6, 0);
}

void BaseStation::setServiceabilityView()
{
  QSize size_ = this->size();
  
  window_1->showNormal();
  window_2->showMinimized();
  window_3->showNormal();
  window_4->showMinimized();
  window_cam->showNormal();
  
  
  window_1->setMinimumWidth(size_.width()*0.2);
  window_1->resize(size_.width()*0.4, size_.height()*0.55);
  window_1->setMaximumWidth(size_.width()*2.1);
  window_1->setMaximumHeight(size_.height()*2.2);
  window_1->setMinimumHeight(size_.height()*0.2);
  window_1->move(0, size_.height() * 0.4);
  
  window_3->setMinimumWidth(size_.width()*0.2);
  window_3->setMaximumWidth(size_.width()*3);
  window_3->setMaximumHeight(size_.height()*2.2);
  window_3->setMinimumHeight(size_.height()*0.1);
  window_3->resize(size_.width()*0.85, size_.height()*0.2);
  window_3->move(0, 0);
  
  window_cam->setMinimumWidth(size_.width()*0.2);
  window_cam->setMaximumWidth(size_.width()*3);
  window_cam->setMaximumHeight(size_.height()*2.2);
  window_cam->setMinimumHeight(size_.height()*0.1);
  window_cam->resize(size_.width()*0.5, size_.height()*0.9);
  
  window_cam->move(size_.width()*0.4, 0);
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

void BaseStation::configurePointCloud(rviz::Display *&pc_display, const std::string &topic, rviz::VisualizationManager *manager)
{
  pc_display = manager->createDisplay("rviz/PointCloud2", "sensors_msgs::PointCloud2",true);
  pc_display->setTopic(QString::fromStdString(topic), "sensor_msgs/PointCloud2");
  pc_display->subProp("Color Transformer")->setValue("AxisColor");
//   pc_display->initialize(manager_);
}

QMdiSubWindow *BaseStation::configureCameraDisplay() {
  QMdiSubWindow *ret_val = NULL;
  camera_display = manager_3->createDisplay("rviz/Camera", "Front camera",true);
  camera_display->setTopic("/front_web/rgb/image_raw", "sensor_msgs/Image");
  camera_display->subProp("Transport Hint")->setValue("compressed");
  camera_display->subProp("Overlay Alpha")->setValue(0.9);
  camera_display->subProp("Image Rendering")->setValue("background");
  
  ret_val = mdiArea->addSubWindow(camera_display->getAssociatedWidget());
  
  marker_section = manager_3->createDisplay("rviz/Marker", "Marker Sec Typ", true);
  marker_section->setTopic("/section_type", "visualization_msgs/Marker");
  
  
  
  
//   grid_display->subProp( "Color" )->setValue( Qt::yellow );
  return ret_val;
}


void BaseStation::configureGridDisplay(rviz::VisualizationManager *vis) {
  // Create a Grid display. 
  grid_display = vis->createDisplay( "rviz/Grid", "grid", true );
  ROS_ASSERT( grid_display != NULL );

  // Configure the GridDisplay the way we like it.
  grid_display->subProp( "Line Style" )->setValue( "Lines" );
  grid_display->subProp( "Cell Size" )->setValue(0.2); 
  grid_display->subProp( "Plane Cell Count" )->setValue(50);
}

QMdiSubWindow *BaseStation::configureImageDisplay(const std::string &topic, const std::string &window_name) {
  QMdiSubWindow *ret_val = NULL;
//   camera_display = manager_->createDisplay("rviz/Camera", "Front camera",true);
  image_display = manager_->createDisplay("rviz/Image", QString::fromStdString(window_name),true);
  image_display->setTopic(QString::fromStdString(topic), "sensor_msgs/Image");
  image_display->subProp("Transport Hint")->setValue("compressed");
  
  ret_val = mdiArea->addSubWindow(image_display->getAssociatedWidget());
  
  
  
  return ret_val;
}

void BaseStation::configureMap() {
  sat_view = manager_2->createDisplay("rviz_plugins/AerialMapDisplay","Localization display",true);
  if (sat_view != NULL) {
//     sat_view->setTopic("/gps/fix","Topic");
    sat_view->subProp("Topic")->setValue("/gps/fix");
    sat_view->subProp("Zoom")->setValue(18);
    sat_view->subProp("Object URI")->setValue("http://a.tiles.mapbox.com/v4/mapbox.streets/{z}/{x}/{y}.jpg?access_token=pk.eyJ1IjoiY2h1cnIiLCJhIjoiY2l6dHQzZWJyMDFnZjMzbnA1cDR4MWV3cCJ9.r-YuBsl8JXSBQ_UXTOeSYA");
    sat_view->subProp("Robot frame")->setValue("map");
    sat_view->subProp("Blocks")->setValue(8);
    
    // Set the view
    rviz::ViewController *v = render_panel_2->getViewController();
    map_camera_.setSettings(v);
  } else {
    std::cerr << "Could not create satellite view\n";
  }
  
  // Create a Grid display.
  grid_display2 = manager_2->createDisplay( "rviz/Grid", "grid", true );
//   ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_display2->subProp( "Line Style" )->setValue( "Lines" );
  grid_display2->subProp( "Cell Size" )->setValue(5.0);
  
  // Sewer graph marker
  marker_1 = manager_2->createDisplay("rviz/Marker", "marker_1", true);
  marker_1->setTopic("/amcl_sewer_node/sewer_graph","visualization_msgs/Marker");
  
  marker_2 = manager_2->createDisplay("rviz/Marker", "marker_1", true);
  marker_2->setTopic("/alerts","visualization_msgs/Marker");
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
}

void BaseStation::updateValues()
{
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
  
  if (last_status.reverse != state.reverse) {
    // The mode has changed --> change the view automatically
    setRvizExplorationView(state.reverse);
  }
 
  last_status = state;
}

void BaseStation::updateRSSIStatus(const rssi_get::Nvip_status& status)
{
  progressBar_rssi->setValue(status.rssi_perc);
}

void BaseStation::updateTreeContent(const string& string)
{
  tree_widget->clear();
  // Get the Lines
  QString text(QString::fromStdString(string));
  QStringList lines = text.split("\n");
  
  
  QList<QTreeWidgetItem*> parents;
  QList<int> indentations;
  parents << tree_widget->invisibleRootItem();
  indentations << 0;

  int number = 0;
  
//   Adapted from Qt Simple Tree Model Tutorial
  while (number < lines.count()) 
  {
    int position = 0;
    while (position < lines[number].length()) {
      if (lines[number].at(position) != ' ')
        break;
      position++;
    }

    QString lineData = lines[number].mid(position).trimmed();

    if (!lineData.isEmpty()) {
      // Read the column data from the rest of the line.
      QStringList columnStrings = lineData.split("\t", QString::SkipEmptyParts);
      QList<QVariant> columnData;
      for (int column = 0; column < columnStrings.count(); ++column)
        columnData << columnStrings[column];

      if (position > indentations.last()) {
          // The last child of the current parent is now the new parent
          // unless the current parent has no children.

          if (parents.last()->childCount() > 0) {
              parents << parents.last()->child(parents.last()->childCount()-1);
              indentations << position;
          }
      } else {
          while (position < indentations.last() && parents.count() > 0) {
              parents.pop_back();
              indentations.pop_back();
          }
      }

      // Append a new item to the current parent's list of children.
      QTreeWidgetItem *new_item = new QTreeWidgetItem(parents.last(), columnStrings);
    }

    ++number;
  }
}

void BaseStation::setRvizExplorationView(bool reverse)
{
  ROS_INFO("Changing view. Reverse = %d", reverse);
  rviz::ViewController *v = render_panel_->getViewController();
  if (!reverse) {
    if (cloud_camera_.f_x > 0.0) {
      cloud_camera_.f_x *= -1.0;
      cloud_camera_.yaw += M_PI;
      cloud_camera_.setSettings(v);
    }
  } else {
    if (cloud_camera_.f_x < 0.0) {
      cloud_camera_.f_x *= -1.0;
      cloud_camera_.yaw += M_PI;
      cloud_camera_.setSettings(v);
    }
  }
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



