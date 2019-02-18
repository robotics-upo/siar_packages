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
#include <list>
#include <cmath>

#include <boost/asio.hpp>
using boost::asio::ip::udp;

struct Message_ 
{
  std::vector <uint8_t> vec;
  uint32_t id;
  uint32_t size;
  std::string topic;
  uint32_t received; // Number of bytes received
  uint16_t crc;
};

class UDPManager
{
public:

  //!Intialize the serial port to the given values
  UDPManager():max_udp_length(1460),max_topic_length(256), max_msgs(5),msg_sent(0),header("UDPHEAD"),start("UDPSTART"),max_length(200000L),max_time(5),running(false) 
  {
//     ROS_INFO("In UDPManager()");
    buf_s.resize(max_udp_length);
    timer_.reset(new boost::asio::deadline_timer(io_service));
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
  std::list<Message_> msg_list; // List with the message to be processed
  

  //!Reads a chunk and returns a message if a message has been completed
  int getChunk(std::string &topic, std::vector <uint8_t> &msg)
  {
    int i;
    int err;
    int cont = 0;
    uint16_t crc;
    uint32_t size = 1;
    uint32_t id;
    
    bool msg_complete = false;
    
    std::vector <uint8_t> msg_chunk;
    msg_chunk.resize(max_udp_length);
    udp::endpoint sender_endpoint;
    
    topic.clear();
    
    // Read UDP datagrams until a header is found
    bool header_found = false;
    try 
    {
      timeout = false;
      bad_read = true;
      received = false;
      socket_ptr->async_receive_from(
      boost::asio::buffer(msg_chunk.data(), max_udp_length), sender_endpoint,
      boost::bind(&UDPManager::handle_receive_from, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));

      timer_->expires_from_now(boost::posix_time::seconds(max_time));
      timer_->async_wait(boost::bind(&UDPManager::handle_wait, this, boost::asio::placeholders::error));
      
       io_service.reset(); 
       while (io_service.run_one() && !timeout && !received)
       {
        if (timeout)
          socket_ptr->cancel();
        else if (received)
          timer_->cancel();
       }
      
      
      if ( timeout) {
        return -3; // Cont expired or error in transmission
      }
      
      if (bad_read && received) {
        return -4;
      }
      
      bool is_start = false;
      
      if (len == start.size()) {
        std::string possible_start((const char *)msg_chunk.data(), start.size());
        if (start == possible_start) {
          ROS_INFO("Ignoring start message");
          is_start = true;
        }
      } 
      if (len > header.size() && !is_start) 
      { 
        // Check if a new message has arrived
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
          i++; 
          size = *(uint32_t* ) (msg_chunk.data() + i);
          i += 4;
          id = *(uint32_t* ) (msg_chunk.data() + i);
          i += 4;
//           ROS_INFO ("Got a message: len = %d, size = %u. id = %d, Topic: %s", (int)len, size, (int)id, topic.c_str());
            
          if (size > max_length) {
//             ROS_INFO("Max message length exceeded. Dropping");
            return -1;
          }
          msg.resize(size);
            
          // Get the crc16
          crc = *(uint16_t *)(msg_chunk.data() + i);
          i += 2;
            
          // Get the data
          msg.resize(size);
          for (; i < len; i++, cont++) {
            msg[cont] = msg_chunk[i];
          }
//           ROS_INFO("Cont: %d \t Size: %d", (int)cont, (int)size);
          if (std::abs(static_cast<long>(cont - size)) < 2) {
            // All the data has been retrieved --> return a non-negative number
            ROS_INFO("Got a message composed by one datagram. Topic = %s. Size = %d", topic.c_str(), (int)size);
            msg_complete = true;
          } else {
            Message_ m;
            m.vec = msg;
            m.id = id;
            m.topic = topic;
            m.size = size;
            m.crc = crc;
            m.received = len - header.size() - topic.size() - 11;
            
            msg_list.push_back(m);
            
            ROS_INFO("First chunk of message received. Received bytes: %d", (int)m.received);
            
            if (msg_list.size() > max_msgs) 
              msg_list.erase(msg_list.begin()); // If the buffer size is surpassed --> erase the oldest msg
       
          }
        }
      }
      
      if (!header_found && !is_start)  // Not a header --> the message is not the first of one topic
      {
        // Find the existing message (if any)
        // We have now a non-header block, try to append it to the existing info
        id = *(uint32_t* ) (msg_chunk.data());
//         ROS_INFO("Received a message that is not a header. ID : %d", (int)id);
        std::list<Message_>::iterator it = msg_list.begin();
      
        for (;it != msg_list.end();it++) {
          if (it->id == id) { 
            // Found
            uint32_t cont_ant = *(uint32_t* ) (msg_chunk.data() + 4); // Get the position
            
            memcpy(it->vec.data() + cont_ant, msg_chunk.data() + 8, len - 8);
            it->received += len - 8;
            if (it->received == it->size) {
              // All the data has been received --> mark it
              ROS_INFO("Completed a message. Topic %s. Size = %d", it->topic.c_str(), (int)it->size);
              msg = it->vec;
              msg_complete = true;
              topic = it->topic;
              size = it->size;
              crc = it->crc;
              msg_list.erase(it);
              break;
            } else {
//               ROS_INFO("Message not completed. Topic %s. Cont_ant = %d. Received = %d", it->topic.c_str(), (int)cont_ant, (int)it->received);
            }
          }
          
        }
      }
      if (msg_complete) {
        if (crc16(msg.data(), msg.size()) != crc && msg_complete) 
        {
          ROS_INFO("Bad CRC");
          return -1;
        }
      }
      
      if (!msg_complete) {
        return -2;
      }
      
      if (msg_complete) {
        ROS_INFO("Message OK");
        return msg.size();
        
      }
    } 
    catch (std::exception &e) 
    {
      ROS_INFO("Exception catched while reading message. Content %s", e.what());
    }
    
    return 0;
  }
  
  void handle_receive_from(const boost::system::error_code& err,
      size_t length)
  {
    
    received = true;
    if (err)
    {
      bad_read = true;
    }
    else
    {
      bad_read = false;
      len = length;
      ROS_INFO("Received a message of length: %d", (int)len);
    }
//     timer_ -> cancel();
  }
  
  void handle_wait(const boost::system::error_code& error) // Result of operation
  {
    if (!error) {
    
    
      timeout = true;
      ROS_INFO("UDPBRIDGE --> Timeout");
//     socket_ptr->cancel();
    }
  }
  
  //!Write a complete message. If congestion is detected --> blocks transmission (if not secure)
  int writeMessage(const std::string &topic, const std::vector<uint8_t> &msg)
  {
    boost::system::error_code ignored_error;
    blocking_write(msg.data(), msg.size(), topic.size());
    return msg.size();
  }
  
  size_t simple_write(const uint8_t* buf, uint32_t size) {
    boost::system::error_code ignored_error;
    return socket_ptr->send_to(boost::asio::buffer(buf, size), remote_endpoint, 0, ignored_error);
  }
  
  //! @brief Sends a message, breaks it into pieces if necessary (max_size)
  int blocking_write(const uint8_t* buf, uint32_t size, uint32_t topic_size) {
    int ret_val = size;
    uint32_t id = msg_sent;
    uint32_t cont = 0; // Counts bytes of real info (without headers)
    boost::system::error_code ignored_error;
    
    int total_header_size = header.size() + 11 + topic_size;
    
    memcpy(buf_s.data(), &id, 4);
    try {
      // Send the first chunk
      int sending_bytes = max_udp_length; // Bytes sent
      if (sending_bytes > size) {
        sending_bytes = size;
      }
      socket_ptr->send_to( boost::asio::buffer(buf, sending_bytes), remote_endpoint, 0, ignored_error);
      cont += sending_bytes - total_header_size; // We have to discount the header size
      
      sending_bytes = max_udp_length - 8;
      for (;cont < size - total_header_size; cont += max_udp_length - 8) {
        if ( size - total_header_size - cont < sending_bytes) {
          sending_bytes = size - total_header_size - cont;
        }
        memcpy(buf_s.data() + 4, &cont, 4);
        memcpy(buf_s.data() + 8, buf + cont + total_header_size, sending_bytes);
        
        socket_ptr->send_to( boost::asio::buffer(buf_s.data(), sending_bytes + 8), remote_endpoint, 0, ignored_error);
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
      std::vector<uint8_t> buffer(size + topic.size() + header.size() + 11); 
      
      // Insert header and topic name
      memcpy(buffer.data(), header.c_str(), header.size());
      memcpy(buffer.data() + header.size(), topic.c_str(), topic.size());
      *(buffer.data() + header.size() + topic.size()) = ',';
      memcpy(buffer.data() + header.size() + topic.size() + 1, &size, 4);
      memcpy(buffer.data() + header.size() + topic.size() + 5, &msg_sent, 4);
      
      // Insert data (at the end of the message
      ros::serialization::OStream stream(buffer.data() + topic.size() + header.size() + 11, size);
      ros::serialization::Serializer<T>::write(stream, msg);
      
      // Insert CRC 
      uint16_t crc = crc16(buffer.data() + topic.size() + header.size() + 11, size);
      memcpy(buffer.data() + topic.size() + header.size() + 9, &crc, 2);
      ret_val = writeMessage(topic, buffer);
      msg_sent++;
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
  const int max_udp_length, max_topic_length;
  const int max_msgs; // Max messages in msg_list
  int port;
  uint32_t msg_sent;
  boost::shared_ptr<udp::socket> socket_ptr;
  boost::asio::io_service io_service;
  udp::endpoint remote_endpoint;
  std::vector<uint8_t> buf_s;
  
  //! Reading thread handler
  boost::thread readThreadHandler;
  boost::mutex time_mutex;
  
  //! Header and start strings
  const std::string header, start;
  
  const uint32_t max_length;
  const int max_time;
  
  //! Running flag
  bool running;
  bool bad_read, timeout, received;
  size_t len;
  
  boost::shared_ptr<boost::asio::deadline_timer> timer_;
};


#endif
