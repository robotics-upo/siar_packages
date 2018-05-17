
#include <siar_rqt_plugin/my_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QLabel>
#include <QCheckBox>
#include <QString>
#include <QTextEdit>
#include <QMessageBox>
#include <QComboBox>
#include <QSpinBox>
#include "siar_driver/SiarStatus.h"
#include <ros/ros.h>
#include "alert_db/GenerateAlert.h"

#include <sstream>
#include <string>

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

namespace siar_rqt_plugin
{

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
  //setWindowIcon(QIcon("~/catkin_ws/src/siar_packages/siar_rqt_plugin/icon.png"));
 // widget_->setWindowIcon(QIcon("~/catkin_ws/src/siar_packages/siar_rqt_plugin/icon.png"));
  
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  
  my_subscriber = getNodeHandle().subscribe("/siar_status", 1, &MyPlugin::ros_data_callback, this);
  
  alert_service = getNodeHandle().serviceClient<alert_db::GenerateAlert>("/generate_alert");
  
  connect(ui_.pushButton, SIGNAL(pressed()), SLOT(click()));
  connect(ui_.pushButton, SIGNAL(pressed()), SLOT(cleanAlert()));
  
}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

void MyPlugin::ros_data_callback(const siar_driver::SiarStatus msg)
{
  std::stringstream sstm;
  sstm << "Mode: " << int(msg.operation_mode);
  ui_.operationMode->setText(QString::fromStdString(sstm.str()));
  ui_.operationMode_2->setText(QString::fromStdString(sstm.str()));
  sstm.str("");
  sstm << "Speed: " << float(msg.speed);
  ui_.speed->setText(QString::fromStdString(sstm.str()));  
  ui_.speed_2->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "Width: " << float(msg.width);
  ui_.width->setText(QString::fromStdString(sstm.str()));  
  ui_.width_2->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "Electronics_x: " << float(msg.electronics_x);
  ui_.electronicsX->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "Lin Motor pot: " << int(msg.lin_motor_pot);
  ui_.linMotorPot->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "Lin Motor pos: " << int(msg.lin_motor_pos);
  ui_.linMotorPos->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "Position:{ x: " << float(msg.x)<<"; y: "<< float(msg.x)<< "}";
  ui_.xy->setText(QString::fromStdString(sstm.str()));  
  
  sstm.str("");
  sstm << "Electric battery: { Percentage:"<< float(msg.elec_battery.percentage)<<"; Voltage: "<< float(msg.elec_battery.voltage) << "V; Current :" << float(msg.elec_battery.current)<< "A }";
  ui_.elecBattery->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "Motor battery: { Percentage:"<< float(msg.motor_battery.percentage)<<"; Voltage: "<< float(msg.motor_battery.voltage) << "V; Current :" << float(msg.motor_battery.current)<< "A }";
  ui_.motorBattery->setText(QString::fromStdString(sstm.str()));  
 // sstm.str("");
 // sstm << "Power supply:"<< float(msg.x);
 // ui_.powerSupply->setText(QString::fromStdString(sstm.str()));  
  
  
  
  ui_.slow->setChecked(msg.slow); 
  ui_.slow_2->setChecked(msg.slow); 
  ui_.reverse->setChecked(msg.reverse); 
  ui_.reverse_2->setChecked(msg.reverse); 
  ui_.frontLight->setChecked(msg.front_light); 
  ui_.frontLight_2->setChecked(msg.front_light); 
  ui_.rearLight->setChecked(msg.rear_light); 
  ui_.rearLight_2->setChecked(msg.rear_light);  
  ui_.armPanic->setChecked(msg.arm_panic); 
  ui_.armPanic2->setChecked(msg.arm_panic); 
  ui_.fan->setChecked(msg.fan); 
  ui_.hardStop->setChecked(msg.hard_stop);
  
  
  sstm.str("");
  sstm << "1: " << int(msg.herculex_position[0]);
  ui_.herculexPosition1->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "2: " << int(msg.herculex_position[1]);
  ui_.herculexPosition2->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "3: " << int(msg.herculex_position[2]);
  ui_.herculexPosition3->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "4: " << int(msg.herculex_position[3]);
  ui_.herculexPosition4->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "5: " << int(msg.herculex_position[4]);
  ui_.herculexPosition5->setText(QString::fromStdString(sstm.str())); 
  
  sstm.str("");
  sstm << "1: " << int(msg.herculex_temperature[0]);
  ui_.herculexTemperature1->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "2: " << int(msg.herculex_temperature[1]);
  ui_.herculexTemperature2->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "3: " << int(msg.herculex_temperature[2]);
  ui_.herculexTemperature3->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "4: " << int(msg.herculex_temperature[3]);
  ui_.herculexTemperature4->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "5: " << int(msg.herculex_temperature[4]);
  ui_.herculexTemperature5->setText(QString::fromStdString(sstm.str())); 
  
  sstm.str("");
  sstm << "1: " << int(msg.herculex_torque[0]);
  ui_.herculexTorque1->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "2: " << int(msg.herculex_torque[1]);
  ui_.herculexTorque2->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "3: " << int(msg.herculex_torque[2]);
  ui_.herculexTorque3->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "4: " << int(msg.herculex_torque[3]);
  ui_.herculexTorque4->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "5: " << int(msg.herculex_torque[4]);
  ui_.herculexTorque5->setText(QString::fromStdString(sstm.str()));   

  sstm.str("");
  sstm << "1: " << int(msg.herculex_status[0]);
  ui_.herculexStatus1->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "2: " << int(msg.herculex_status[1]);
  ui_.herculexStatus2->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "3: " << int(msg.herculex_status[2]);
  ui_.herculexStatus3->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "4: " << int(msg.herculex_status[3]);
  ui_.herculexStatus4->setText(QString::fromStdString(sstm.str()));  
  sstm.str("");
  sstm << "5: " << int(msg.herculex_status[4]);
  ui_.herculexStatus5->setText(QString::fromStdString(sstm.str()));   

  
}

void MyPlugin::click() {
  alert_db::GenerateAlert::Request req;
  alert_db::GenerateAlert::Response res;
  req.head.stamp = ros::Time::now();
  req.description = ui_.textEdit->toPlainText().toStdString();
  req.position = ui_.spinBox->value(); // TODO: Add position according to clock hours
  req.attach_images = ui_.checkBox->checkState();
  
  QComboBox &cb = *ui_.alertTypeCombo;
  int index = cb.currentIndex();
  if (index > 0)
    index++;
  req.type = index;
  
  ROS_INFO("Calling generate alert service");
  alert_service.call(req, res);
  if (res.result == 0) {
    // Show error
    QMessageBox::critical(widget_, "Service error", "Could not create the alert");
  } else {
    QMessageBox::information(widget_, "Alert OK", "Alert generated successfully");
  }
}

void MyPlugin::cleanAlert() {
  
  ui_.textEdit->setText("");
  ui_.checkBox->setCheckState(Qt::CheckState(false));
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}  // namespace rqt_example_cpp
PLUGINLIB_DECLARE_CLASS(siar_rqt_plugin, MyPlugin, siar_rqt_plugin::MyPlugin, rqt_gui_cpp::Plugin)
