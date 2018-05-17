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

#include <functions/functions.h>
#include <functions/Point3D.h>
#include <functions/FormattedTime.h>

#include "Comms.h"

// Qt includes
#include <QTimer>
#include <QMainWindow>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_symbol.h>
#include <base_station/ui_siar_gui.h>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/yaml_config_reader.h"

#include <QWebView>
#include <QMdiArea>


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
  
private slots:
  void updateValues();
  
private:
  

  
  //! @brief Clears the data contained in the logs
  void clearLogs();
  //! @brief Stop the ROS communication (in Comms.h)
  void stopComms();
  bool startComms();
  
  int argc;
  char **argv;
  functions::FormattedTime init_log_time;
  
  // Curves
  std::vector<QwtPlotCurve *> curves;
  std::vector<QwtPlotCurve *> curves_z;
  std::vector<std::vector <double> > distance_log;
  std::vector<std::vector <double> > distance_log_z;
  std::vector<std::vector<functions::Point3D> > position_log;
  std::vector<double> t_log;
  
  // Automatic emergency stop!!
  double xy_dist, z_dist;
  
  // Communication stuff
  Comms *node;
  std::vector<uint> uavs;
  std::vector<std::vector<functions::Point3D> > pos_log;
  
  // Configuration
  double time_step;
  QTimer *timer;
  unsigned int count;
  
  // Flags
  bool started;
  
  // RViz stuff
  rviz::VisualizationManager* manager_,*manager_2;
  rviz::RenderPanel* render_panel_,*render_panel_2;
  rviz::Display* sat_view;
  rviz::Display* point_cloud_1, *point_cloud_2, *point_cloud_3;
  rviz::Display* robot_model_display;
  rviz::Display* axes_display, *grid_display, *grid_display2;
  rviz::Display* camera_display, *image_display;
  rviz::Display* marker_1, *marker_2;
  QMdiSubWindow *window_1, *window_2, *window_3;
//   rviz::RenderPanel* 
  void configureRVizDisplay(rviz::VisualizationManager *&manager, rviz::RenderPanel *&panel, 
                            const std::string &frame="/base_link", QLayout *parent = NULL);
  QMdiSubWindow * configureRVizDisplay(rviz::VisualizationManager*& manager, rviz::RenderPanel*& panel, 
                                       const std::string &frame, QMdiArea *parent);
  void configurePointCloud(rviz::Display *&pc_display, const std::string &topic);
  QMdiSubWindow * configureCameraDisplay();
  void configureMap();
  void resizeWindows();
  
  // Web kit
  QWebView *web_view;
};

#endif // BASE_STATION_H____
