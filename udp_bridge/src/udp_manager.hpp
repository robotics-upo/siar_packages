#ifndef __STREAM_MANAGER_HPP__
#define __STREAM_MANAGER_HPP__

#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <vector>

#include <boost/asio.hpp>
using boost::asio::ip::udp;

class UDPManager
{
public:

  //!Intialize the serial port to the given values
  UDPManager():max_udp_length(1400),header("UDPHEAD"),max_length(200000L),running(false)
  {
    ROS_INFO("In UDPManager()");
  }
  
  //!Default destructor
  ~UDPManager()
  {
    endSession();
  }
  
  //!Intialize the serial port to the given values
  virtual bool startSession() = 0;

  //!Intialize the port to the given values
  bool init()
  {
    ROS_INFO("About to start session");
    startSession();
    running = true;
    readThreadHandler = boost::thread(boost::bind(&UDPManager::readThread, this));
    
    ROS_INFO("Thread created");
    
    return true;
  }  
  
  //!Finish the serial communication 
  bool endSession()
  {
    // Stop threads
    if(running)
    {
      running = false;
      readThreadHandler.join();
    }
    
    return true;
  }
  
  bool isRunning() const
  {
    return running;
  }

protected:

  //!Read (blocking) a complete message from the stream port
  int readMessage(std::string &topic, std::vector<uint8_t> &msg, uint32_t max_topic_length = 256)
  {
    int i;
    int err;
    int cont = 0;
    uint16_t crc;
    uint32_t size = 1;
    
    std::vector <uint8_t> msg_chunk;
    msg_chunk.resize(max_udp_length);
    udp::endpoint sender_endpoint;
    
    topic.clear();
    
    // Read UDP datagrams until a header is found
    bool header_found = false;
    try 
    {
      
      for(int cont_ant = cont;cont < size; cont_ant = cont) 
      {
        size_t len = socket_ptr->receive_from(boost::asio::buffer(msg_chunk.data(), max_udp_length), sender_endpoint);
        if (len > header.size()) 
        {
          std::string possible_header((const char *)msg_chunk.data(), header.size());
          
          if (header == possible_header) 
          {
            header_found = true;
            topic.clear();
            // Get the important info
            i = header.size();
            char c = msg_chunk.at(i);
            for (; c != ',' && i < max_topic_length ;i++, c = msg_chunk.at(i)) 
            {
              topic += c;
            }
            if (i >= max_topic_length) {
              return -1;
            }
            
            size = *(uint32_t* ) (msg_chunk.data() + i + 1);
            ROS_INFO ("Got a message: size = %u. Topic: %s", size, topic.c_str());
            
            if (size > max_length) {
              ROS_INFO("Max message length exceeded. Dropping");
              return -1;
            }
            msg.resize(size);
            
            // Get the crc16
            crc = *(uint16_t *)(msg_chunk.data() + i + 5);
            
            // Get the data
            msg.resize(size);
            for (i = i + 7; i < msg_chunk.size(); i++, cont++) {
              msg[cont] = msg_chunk[i];
            }
          } 
          else if (header_found)
	  {
	    // We have now a non-header block, try to append it to the existing info
	    cont += len;
	    for (i = 0;cont_ant < cont && cont_ant < size; cont_ant++, i++) 
	    {
	      msg[cont_ant] = msg_chunk[i];
	    }
	  }
        } 
        
      }
      
      if (crc16(msg.data(), msg.size()) != crc) 
      {
        ROS_INFO("Bad CRC");
        return -1;
      }
      ROS_INFO("CRC ok!");
    } 
    catch (std::exception &e) 
    {
      ROS_INFO("Exception catched while reading message. Content %s", e.what());
    }
    
    return 0;
  }
  
  //!Write a complete message. If congestion is detected --> blocks transmission (if not secure)
  int writeMessage(const std::string &topic, const std::vector<uint8_t> &msg)
  {
    boost::system::error_code ignored_error;
    blocking_write(msg.data(), msg.size());
    return msg.size();
  }
  
  //! @brief Sends a message, breaks it into pieces if necessary (max_size)
  int blocking_write(const uint8_t* buf, uint32_t size) {
    int ret_val = size;
    int cont = 0;
    boost::system::error_code ignored_error;
    try {
      for (;cont < size; cont += max_udp_length) {
        socket_ptr->send_to( boost::asio::buffer(buf + cont, max_udp_length), remote_endpoint, 0, ignored_error);
      }
    } 
    catch (std::exception &e)
    {
      ret_val = -1;
    }
    return ret_val;
  }

  //! CRC computation
  uint16_t crc16(const uint8_t *buffer, uint32_t length)
  {
    uint16_t crc = 0xFFFF;

    while(length--)
    {
      uint8_t x = crc >> 8 ^ *buffer++;
      x ^= x>>4;
      crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    
    return crc;
  }
  
  //! get the UDP data and 
  template<typename T>
  T deserializePublish(uint8_t * buffer, uint32_t size, ros::Publisher &pub)
  {
    T msg;
    try 
    {
      ros::serialization::IStream stream(buffer, size);
      ros::serialization::Serializer<T>::read(stream, msg);
      pub.publish(msg);
    } 
    catch (std::exception &e) 
    {
      ROS_INFO("Error while serializing: %s", e.what());
    }
    return msg;
  }
  
  //! get the UDP data
  template<typename T>
  T deserialize(uint8_t * buffer, uint32_t size)
  {
    T msg;
    try 
    {
      ros::serialization::IStream stream(buffer, size);
      ros::serialization::Serializer<T>::read(stream, msg);
    } 
    catch (std::exception &e) 
    {
      ROS_INFO("Error while serializing: %s", e.what());
    }
    return msg;
  }
  
  //! Serialize and get a uint8_t buffer of data to be sent over UDP
  template<typename T>
  int serializeWrite(const std::string &topic, T msg)
  {
    int ret_val = -1;
    try
    {
      // Size of the buffer: msg, header, topicname, 4 of size, 2 of CRC and 1 of end of topic name
      uint32_t size = ros::serialization::Serializer<T>::serializedLength(msg);
      std::vector<uint8_t> buffer(size + topic.size() + header.size() + 7); 
      
      // Insert header and topic name
      memcpy(buffer.data(), header.c_str(), header.size());
      memcpy(buffer.data() + header.size(), topic.c_str(), topic.size());
      *(buffer.data() + header.size() + topic.size()) = ',';
      memcpy(buffer.data() + header.size() + topic.size() + 1, &size, 4);
      
      // Insert data (at the end of the message
      ros::serialization::OStream stream(buffer.data() + topic.size() + header.size() + 7, size);
      ros::serialization::Serializer<T>::write(stream, msg);
      
      // Insert CRC 
      uint16_t crc = crc16(buffer.data() + topic.size() + header.size() + 7, size);
      memcpy(buffer.data() + topic.size() + header.size() + 5, &crc, 2);
      ret_val = writeMessage(topic, buffer);
    } 
    catch (std::exception &e) 
    {
      ROS_INFO("Error while serializing: %s", e.what());
    }
    return ret_val;
  }
  
  //! Thread to read from serial port
  virtual void readThread() = 0;
  
  //! UDP stuff
  int max_udp_length;
  int port;
  boost::shared_ptr<udp::socket> socket_ptr;
  boost::asio::io_service io_service;
  udp::endpoint remote_endpoint;
  
  //! Reading thread handler
  boost::thread readThreadHandler;
  boost::mutex time_mutex;
  
  //! Header 
  std::string header;
  
  uint32_t max_length;
  
  //! Running flag
  bool running;
};


#endif
