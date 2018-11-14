
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <boost/lexical_cast.hpp>

#include <sstream>
#include <fstream>
#include "functions/FormattedTime.h"

#include <QtGui/QGuiApplication>
#include <QApplication>
#include "BaseStation.h"

#ifndef PI
#define PI 3.14159265
#endif

using namespace std;

// Initial time (time when the first message arrived
bool init_time = false;
functions::FormattedTime t;

int main(int argc, char** argv)
{
  
  QApplication app(argc, argv);
  BaseStation foo(argc, argv);
  foo.show();
  int ret_val = app.exec();
//   ros::waitForShutdown();
  
  return ret_val;
}

