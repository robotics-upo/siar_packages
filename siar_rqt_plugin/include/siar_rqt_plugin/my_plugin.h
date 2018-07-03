
#ifndef SIAR_RQT_PLUGIN_MY_PLUGIN_H
#define SIAR_RQT_PLUGIN_MY_PLUGIN_H

#include <QWidget>

#ifndef Q_MOC_RUN
#include <rqt_gui_cpp/plugin.h>
#include <siar_rqt_plugin/ui_my_plugin.h>
#include <ros/ros.h>
#include "siar_driver/SiarStatus.h"
#endif

namespace siar_rqt_plugin
{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  void ros_data_callback(const siar_driver::SiarStatus msg);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
  
public slots:
  void click();
  void cleanAlert();
  
private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
  ros::Subscriber my_subscriber;
  ros::ServiceClient alert_service;
};
}  // namespace siar_rqt_plugin
#endif  // SIAR_RQT_PLUGIN_MY_PLUGIN_H
