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


#ifndef BASE_STATION_H____
#define BASE_STATION_H____

#ifndef Q_MOC_RUN
#include <functions/functions.h>
#include <functions/Point3D.h>
#include <functions/FormattedTime.h>
#endif

#include "Comms.h"

// Qt includes
#include <QTimer>
#include <QMainWindow>
#include <base_station/ui_siar_gui.h>

// #include <qwt/qwt_dial.h>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/yaml_config_reader.h"

#include <QMdiArea>

#include <QTreeWidget>
#include <QStandardItemModel>
#include <QStandardItem>

#include "CameraSettings.h"

class BaseStation:public QMainWindow, Ui::MainWindow
{
  Q_OBJECT
public:
    BaseStation(int argc, char **argv, QWidget* parent = 0, Qt::WindowFlags flags = 0);
    virtual ~BaseStation();
    BaseStation(const QMainWindow& );
    
    
public slots:
  void updateSiarStatus(const siar_driver::SiarStatus &state);
  void updateRSSIStatus(const rssi_get::Nvip_status &status);
  void updateTreeContent(const std::string &string);
  void setExploreView();
  void setInspectionView();
  void setMissionView();
  void setServiceabilityView();
  
private slots:
  void updateValues();
  
private:
  //! @brief Clears the data contained in the logs
  void clearLogs();
  //! @brief Stop the ROS communication (in Comms.h)
  void stopComms();
  bool startComms();
  
  //! Rviz views and related parameters
  void setRvizExplorationView(bool reverse);
  CameraSettings cloud_camera_;
  CameraSettings map_camera_;
  
  int argc;
  char **argv;
  functions::FormattedTime init_log_time;
  
  // Curves
  
  // Communication stuff
  Comms *node;
  
  // Last messages
  siar_driver::SiarStatus last_status;
  
  // Flags
  bool started;
  
  // RViz stuff
  rviz::VisualizationManager* manager_,*manager_2, *manager_3;
  rviz::RenderPanel* render_panel_,*render_panel_2, *render_panel_3;
  rviz::Display* sat_view;
  rviz::Display* point_cloud_1, *point_cloud_2, *point_cloud_3;
  rviz::Display* robot_model_display;
  rviz::Display* axes_display, *grid_display, *grid_display2;
  rviz::Display* camera_display, *image_display, *image_display_2, *image_display_3;
  rviz::Display* marker_1, *marker_2, *marker_3, *marker_alerts;
  QMdiSubWindow *window_1, *window_2, *window_3, *window_4, *window_cam, *window_unused, *window_img_2, *window_img_3;
  rviz::Display* marker_section, *point_cloud_defects, *p_c_curb, *p_c_gutter;
  
  
//   rviz::RenderPanel* 
  //! @brief Old version when the panel is located in a Layout (deprecated)
  void configureRVizDisplay(rviz::VisualizationManager *&manager, rviz::RenderPanel *&panel, 
                            const std::string &frame="/base_link", QLayout *parent = NULL);
  
  //! @brief Configures the display when the panel is located in a mdiArea
  QMdiSubWindow * configureRVizDisplay(rviz::VisualizationManager*& manager, rviz::RenderPanel*& panel, 
                                       const std::string &frame, QMdiArea *parent);
  void configurePointCloud(rviz::Display *&pc_display, const std::string &topic, rviz::VisualizationManager *manager);
  QMdiSubWindow *configureImageDisplay(const std::string& topic, const std::string &window_name);
  QMdiSubWindow * configureCameraDisplay();
  void configureGridDisplay(rviz::VisualizationManager *vis);
  void configureMap();
  
  // Tree widget
  QTreeWidget *tree_widget;
  
  // qwt_dial
//   QwtDial *Dial_speed;
};

#endif // BASE_STATION_H____
