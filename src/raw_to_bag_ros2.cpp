// -*-c++-*--------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <event_array_msgs/decode.h>
#include <event_array_msgs/encode.h>
#include <unistd.h>

#include <chrono>
#include <event_array_msgs/msg/event_array.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "evt_3_definitions.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "raw_to_bag -b name_of_bag_file -i name_of_raw_file -t topic -f "
               "frame_id -w width -h height"
            << std::endl;
}

namespace metavision_ros_tools
{
using event_array_msgs::mono::bytes_per_event;
using event_array_msgs::msg::EventArray;

class EventBagWriter
{
public:
  EventBagWriter(const std::string & name, const std::string & topic)
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(name);
    writer_->create_topic(
      {topic, "event_array_msgs/msg/EventArray", rmw_get_serialization_format(), ""});
    topic_ = topic;
  }
  void write(const EventArray & m)
  {
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<EventArray> serialization;
    serialization.serialize_message(&m, &serialized_msg);
    writer_->write(
      serialized_msg, topic_, "event_array_msgs/msg/EventArray", rclcpp::Time(m.header.stamp));
  }

private:
  std::string topic_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

void skip_header(std::fstream & in)
{
  int c;
  while ((c = in.peek()) == '%') {
    std::string line;
    std::getline(in, line);
  }
}

static bool is_big_endian()
{
  const union {
    uint32_t i;
    char c[4];
  } combined_int = {0x01020304};  // from stackoverflow
  return (combined_int.c[0] == 1);
}

bool add_event_to_msg(EventArray * msg, uint64_t ts, uint16_t ex, uint16_t ey, uint8_t p)
{
  // nanoseconds since start of message
  uint32_t dt = ts - msg->time_base;  // should not overflow
  constexpr size_t kBufSize = bytes_per_event;
  uint8_t buffer[kBufSize];
  event_array_msgs::mono::encode(
    reinterpret_cast<uint64_t *>(buffer), static_cast<bool>(p), ex, ey, dt);
  msg->events.insert(msg->events.end(), buffer, buffer + bytes_per_event);
  const size_t MAX_MESSAGE_SIZE(5000);

  // keep MAX_MESSAGE_TIME small enough to not overflow the uint32_t!
  const uint32_t MAX_MESSAGE_TIME(100000000);  // in nanoseconds = 0.1ms
  if (msg->events.size() >= MAX_MESSAGE_SIZE || dt > MAX_MESSAGE_TIME) {
    return (true);
  }
  return (false);
}

size_t process_raw(
  const std::string & inFile, const std::string & outFile, const std::string & topic,
  const std::string & frame_id, uint32_t width, uint32_t height)
{
  std::fstream in;
  in.open(inFile, std::ios::in | std::ios::binary);
  skip_header(in);
  const size_t BUF_SIZE(100);
  EVT3::Event buffer[BUF_SIZE];
  EventBagWriter bagWriter(outFile, topic);

  EventArray msg;
  msg.header.frame_id = frame_id;
  msg.width = width;
  msg.height = height;
  msg.encoding = "mono";
  msg.is_bigendian = is_big_endian();
  msg.seq = 0;

  size_t numEvents(0);
  uint16_t ey = 0;
  uint64_t ts(0);  // sensor time stamp accumulated from LOW and HIGH messages
  uint64_t t0_sensor(0);

  rclcpp::Time t0_ros = rclcpp::Clock().now();  // defaults to system time
  rclcpp::Time lastRosTime = t0_ros;
  while (in.read(reinterpret_cast<char *>(buffer), BUF_SIZE * sizeof(EVT3::Event))) {
    size_t numRead = in.gcount() / sizeof(EVT3::Event);
    for (size_t i = 0; i < numRead; i++) {
      switch (buffer[i].code) {
        case EVT3::Code::ADDR_X: {
          const EVT3::AddrX * e = reinterpret_cast<const EVT3::AddrX *>(&buffer[i]);
          if (t0_sensor == 0) {  // this is the first full event ever received
            t0_sensor = ts;
          }
          const uint64_t ts_ros = t0_ros.nanoseconds() + (ts - t0_sensor) * 1000LL;
          if (msg.events.empty()) {  // starting new message
            msg.header.stamp = rclcpp::Time(ts_ros, RCL_SYSTEM_TIME);
            msg.time_base = ts_ros;
          }
          if (add_event_to_msg(&msg, ts_ros, e->x, ey, e->polarity)) {
            bagWriter.write(msg);
            msg.events.clear();
            msg.seq++;
          }
        } break;
        case EVT3::Code::ADDR_Y: {
          const EVT3::AddrY * e = reinterpret_cast<const EVT3::AddrY *>(&buffer[i]);
          ey = e->y;  // save for later
        } break;
        case EVT3::Code::TIME_LOW: {
          const EVT3::TimeLow * e = reinterpret_cast<const EVT3::TimeLow *>(&buffer[i]);
          ts = (ts & 0xFFFFFFFFFFFFF000) | (e->t & 0xFFF);
        } break;
        case EVT3::Code::TIME_HIGH: {
          const EVT3::TimeHigh * e = reinterpret_cast<const EVT3::TimeHigh *>(&buffer[i]);
          const uint64_t prev_ts = ts;
          ts = (e->t << 12);
          if (ts < (prev_ts & 0xFFFFFFFFFFFFF000)) {  // must have experienced wrap-around
            t0_sensor = t0_sensor - (prev_ts - ts);
          }
        } break;
        case EVT3::Code::OTHERS: {
          const EVT3::Others * e = reinterpret_cast<const EVT3::Others *>(&buffer[i]);
          const EVT3::SubType subtype = static_cast<EVT3::SubType>(e->subtype);
          if (subtype != EVT3::SubType::MASTER_END_OF_FRAME) {
            std::cout << "ignoring OTHERS code: " << EVT3::toString(subtype) << std::endl;
          }
        } break;
          // ------- the CONTINUED codes are used in conjunction with OTHERS, so ignore as well
        case EVT3::Code::CONTINUED_4:
        case EVT3::Code::CONTINUED_12: {
        } break;
        default:
          // ------- all the vector codes are not generated by the Gen3 sensor I have....
          std::cout << "got unsupported code: " << static_cast<int>(buffer[i].code) << std::endl;
          throw std::runtime_error("got unsupported code!");
          break;
      }
      numEvents++;
    }
  }
  return (numEvents);
}
}  // namespace metavision_ros_tools

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/events");
  std::string frameId("event_camera");
  int height(480);
  int width(640);
  while ((opt = getopt(argc, argv, "b:i:t:f:h:w:")) != -1) {
    switch (opt) {
      case 'b':
        outFile = optarg;
        break;
      case 'i':
        inFile = optarg;
        break;
      case 't':
        topic = optarg;
        break;
      case 'f':
        frameId = optarg;
        break;
      case 'w':
        width = atoi(optarg);
        break;
      case 'h':
        height = atoi(optarg);
        break;
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (inFile.empty() || outFile.empty()) {
    std::cout << "missing input or output file name!" << std::endl;
    usage();
    return (-1);
  }
  auto start = std::chrono::high_resolution_clock::now();
  size_t numEvents(0);
  numEvents += metavision_ros_tools::process_raw(inFile, outFile, topic, frameId, width, height);
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "number of events read: " << numEvents * 1e-6 << " Mev in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mevs"
            << std::endl;
  return (0);
}
