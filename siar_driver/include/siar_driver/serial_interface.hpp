/***********************************************************************/
/**                                                                    */
/** serial_interface.hpp                                                 */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */ 
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** David Alejo Teissière (maintainer)                                 */
/** Ignacio Perez-Hurtado                                              */
/** Noe Perez                                                          */
/** Rafael Ramon                                                       */
/** Fernando Caballero                                                 */
/** Jesus Capitan                                                      */
/** Luis Merino                                                        */
/**                                                                    */   
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/


#ifndef __SERIAL_IFACE_CXX__
#define __SERIAL_IFACE_CXX__

#ifndef _SIAR_SERIAL_DEBUG_
// #define _SIAR_SERIAL_DEBUG_
#endif

#include <sstream>
#include <string>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdint.h>
#include <iostream>
#include <ctime>
#include <cstdio>
#include <iomanip>
#include <unistd.h>

//-- HEADERS ----------------------------------------------------------------------------

/*********************************************************************************/
/**                                                                              */
/** class SerialInterface                                                        */
/**                                                                              */
/** An abstract class implementing general methods for reading/writing the RS232 */            
/** serial interface                                                             */
/**                                                                              */
/*********************************************************************************/
class SerialInterface
{
  
  public:
  //! \brief Constructor 
  //! \param devicename (IN) -- The device name, as "/dev/ttyUSB0"
  //! \param hardware_flow_control (IN) -- If true, flow control by hardware 
  //! The serial port will be opened when the program calls the open method.
  SerialInterface(const std::string& devicename, bool hardware_flow_control);
  
  //! \brief Destructor
  //!  Purpose: free the memory and close the serial port if it's open 
  virtual ~SerialInterface();
   
  //! \brief Opens the serial link
  //! Purpose:  Open the serial port and configure it at the desired configuration
  //!           The flow control by hardware will be activated if
  //!          hardware_flow_control has been passed to the constructor
  //! \param baudrate 	The baudrate
  //! \param mode 	Open mode: read, write...
  //! \retval  true 	The serial port has been opened successfully
  //! \retval false 	Some error has occurred
  virtual bool open(uint32_t baudrate = 115200, uint8_t mode = static_cast<uint8_t>(O_RDWR | O_NOCTTY | O_NDELAY));
  
  //! \brief Closes the serial port
  //! \retval true The serial port has been closed successfully
  //! \retval false Some error has occurred
  virtual bool close();
  
  //! \brief Checks if serial port is open
  //! \retval true The serial port is open
  //! \retval false The serial port is closed
  bool isOpen();
  
  //! \brief Get a description of the last error if some function returns false 
  //! \return A string describing the last error
  const std::string& getLastError();
  
  //! @brief Writes buffer to serial
  //! @param buf Buffer to write
  //! @param size Number of bytes to write
  //! @retval true Write successful
  //! @retval false Errors found
  bool write(const unsigned char *buf, unsigned int size);
  
  int read_stream(unsigned char* buffer, int buffer_size);
  
  bool incomingBytes(int& bytes);
  
  void flush() {
    tcflush(fd, TCIOFLUSH);
  }
  
  protected:
  bool setRTS(bool rts);
  bool getDSR(bool& dsr);	
  bool getRTS(bool& rts);
  
  
  
  virtual void closeNow();
  
  protected:
  std::string devicename; 
  bool hardware_flow_control;
  int fd;
  std::string last_error;
  int _max_retries;
};

//-- END OF HEADERS ----------------------------------------------------------------------------

/***********************************/
/** SerialInterface implementation */
/***********************************/

inline SerialInterface::SerialInterface(const std::string& devicename, bool hardware_flow_control) : 
devicename(devicename), 
hardware_flow_control(hardware_flow_control),
fd(-1),
last_error(""),
_max_retries(3)
{}

inline SerialInterface::~SerialInterface()
{
  closeNow();
}

inline bool SerialInterface::open(uint32_t baudrate, uint8_t mode )
{
  if (fd!=-1) {
    last_error = std::string("Cannot open ") + devicename + std::string(" because it's already open.");
    return false;	
  }
  
  struct termios attr;
  bool success =  ((fd = ::open(devicename.c_str(), mode)) != -1) && 
		  (fcntl(fd, F_SETFL, 0) != -1) &&
		  (tcgetattr(fd, &attr) != -1) &&
		  (cfsetospeed (&attr, baudrate) != -1) &&
		  (cfsetispeed (&attr, baudrate) != -1) &&
		  (tcflush(fd, TCIOFLUSH) != -1);	

  if (success) {
    attr.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    attr.c_cflag |= (CS8 | CLOCAL | CREAD);
    if (hardware_flow_control) {
      attr.c_cflag |= CRTSCTS;
    }
    else {
      attr.c_cflag &= ~CRTSCTS;
    }
    attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    attr.c_iflag = 0;
    attr.c_oflag = 0;
    attr.c_cc[VMIN]  = 0;
    attr.c_cc[VTIME] = 1;
    success = (tcsetattr(fd, TCSANOW, &attr) != -1);
  }

  if (!success) {
    last_error = std::string(strerror(errno));
    closeNow();
  }

  return success;
}

inline bool SerialInterface::close()
{
  if (fd==-1) {
    last_error = std::string("Cannot close ") + devicename + std::string(" isn't open.");
    return false;	
  }

  bool success = (::close(fd) != -1);
  
  if (success) {
    fd = -1;
  } else {
    last_error = std::string(strerror(errno));	
  }
  
  return success;
}

inline bool SerialInterface::isOpen()
{
	return fd != -1;
}

inline const std::string& SerialInterface::getLastError()
{
	return last_error;
}

inline bool SerialInterface::setRTS(bool rts)
{
	int status;
	bool success = (ioctl(fd, TIOCMGET, &status) != -1);	
	if (success) {
		if (rts) {
			status |= TIOCM_RTS;	
		}
		else {
			status &= ~TIOCM_RTS;
		}
		success = (ioctl(fd, TIOCMSET, &status) != -1);
	}
	if (!success) {
		last_error = std::string(strerror(errno));
	}
	return success;
}


inline bool SerialInterface::getDSR(bool& dsr)
{
	int s;
	bool success = (ioctl(fd,TIOCMGET,&s) != -1);
	if (!success) {
		last_error = std::string(strerror(errno));
	}
	else {
		dsr = (s & TIOCM_DSR) != 0;
	}
	return success;
}

inline bool SerialInterface::getRTS(bool& rts)
{
	int s;
	bool success = (ioctl(fd,TIOCMGET,&s) != -1);
	if (!success) {
		last_error = std::string(strerror(errno));
	}
	else {
		rts = (s & TIOCM_RTS) != 0;
	}
	return success;
}

inline bool SerialInterface::incomingBytes(int& bytes)
{
  bool success= (ioctl(fd, FIONREAD, &bytes)!=-1);
  if (!success) {
    last_error = std::string(strerror(errno));
  }
  return success;
}

inline bool SerialInterface::write(const unsigned char *buf, unsigned int size)
{
  bool success =  (::write (fd, buf, size) == size);
  if (!success) {
    last_error = std::string(strerror(errno));
  }
  return success;
}	

inline int SerialInterface::read_stream(unsigned char* buffer, int buffer_size)
{
  int bytes = 0;
  bool error = false;
  
#ifdef _SIAR_SERIAL_DEBUG_
  std::cout << "Inside read_stream.\n";
#endif
  int timeout = 100;
  while (bytes < buffer_size && bytes >= 0 && timeout > 0) {
    if (!incomingBytes(bytes)) {
      bytes = -1;
    }
    if (bytes < buffer_size) {
      usleep(100);
    }
    timeout --;
  }
  if (timeout < 0) {
    std::cerr << "COuld not read a stream.\n";
  }
  
  if (bytes > 0) {
    bytes = ::read(fd,buffer,buffer_size);
    #ifdef _SIAR_SERIAL_DEBUG_
    std::cout << "Read " << bytes << " bytes. Expected: " <<buffer_size <<  std::endl;
#endif
  } else {
#ifdef _SIAR_SERIAL_DEBUG_
    std::cout << "Error in read stream." << std::endl;
#endif
  }
  
  return bytes;
}

inline void SerialInterface::closeNow()
{
  if (fd!=-1) {
    ::close(fd);
    fd = -1;
  }	
}
//-- END OF INLINE FUNCTIONS ---------------------------------------
#endif