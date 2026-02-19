#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <fstream>
#include <memory>

// https://answers.ros.org/question/11289/read-or-write-message-fromto-file/
// Updated for ROS 2 using rclcpp serialization

namespace wr_msg {

template <class MSG, class FILENAME>
static void writeMsg(const MSG& msg, const FILENAME& name) {
  std::ofstream ofs(name, std::ios::out | std::ios::binary);

  rclcpp::Serialization<MSG> serializer;
  rclcpp::SerializedMessage serialized_msg;
  
  // Serialize the message
  serializer.serialize_message(&msg, &serialized_msg);

  // Write the raw buffer to file
  // serialized_msg.get_rcl_serialized_message() gives access to the underlying C-struct
  // which contains buffer (uint8_t*) and buffer_length (size_t)
  auto rcl_msg = serialized_msg.get_rcl_serialized_message();
  
  if (rcl_msg.buffer && rcl_msg.buffer_length > 0) {
    ofs.write(reinterpret_cast<char*>(rcl_msg.buffer), rcl_msg.buffer_length);
  }
  
  ofs.close();
}

template <class MSG, class FILENAME>
static void readMsg(MSG& msg, const FILENAME& name) {
  std::ifstream ifs(name, std::ios::in | std::ios::binary);
  if (!ifs.is_open()) {
    // Handle error or throw exception? For now, mimicking original behavior (silent failure or crash later)
    return; 
  }

  ifs.seekg(0, std::ios::end);
  std::streampos end = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  std::streampos begin = ifs.tellg();

  size_t file_size = end - begin;

  // Create a SerializedMessage with enough capacity
  rclcpp::SerializedMessage serialized_msg(file_size);
  
  // Read file content directly into the SerializedMessage buffer
  auto rcl_msg = serialized_msg.get_rcl_serialized_message();
  
  // Manually read into buffer and set the length
  ifs.read(reinterpret_cast<char*>(rcl_msg.buffer), file_size);
  serialized_msg.get_rcl_serialized_message().buffer_length = file_size;

  // Deserialize
  rclcpp::Serialization<MSG> serializer;
  serializer.deserialize_message(&serialized_msg, &msg);

  ifs.close();
}

}  // namespace wr_msg
