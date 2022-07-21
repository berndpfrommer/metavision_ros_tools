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
#include <unistd.h>

#include <chrono>
#include <event_array_msgs/msg/event_array.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

#include "evt_3_definitions.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_raw -b name_of_bag_file -o name_of_raw_file -t topic " << std::endl;
}

namespace metavision_ros_tools
{
using event_array_msgs::mono::bytes_per_event;
using event_array_msgs::mono::decode_t_x_y_p;
using event_array_msgs::msg::EventArray;

// you may have to modify the header string to match your system
const char * header =
  "% date 2022-07-20 11:20:09\n"
  "% evt 3.0\n"
  "% firmware_version 4.1.1\n"
  "% integrator_name CenturyArks\n"
  "% plugin_name evc3a_plugin_gen31\n"
  "% serial_number 00000198\n"
  "% subsystem_ID 0\n"
  "% system_ID 40\n";

static size_t write_evt_3(std::fstream & out, const EventArray & s, uint32_t * last_evt_stamp)
{
  if (s.encoding != "mono") {
    std::cerr << "unknown event encoding: " << s.encoding << std::endl;
    throw std::runtime_error("unknown encoding!");
  }

  const uint64_t time_base = s.time_base;  // this implies using sensor time
  uint8_t const * p = &(s.events[0]);

  uint32_t last_etu_us(0);
  for (size_t i = 0; i < s.events.size(); i += bytes_per_event, p += bytes_per_event) {
    uint16_t ex;
    uint16_t ey;
    uint64_t etu;
    bool pol = decode_t_x_y_p(p, time_base, &etu, &ex, &ey);
    const uint64_t sensor_us = etu / 1000;  // total sensor time in usec
    const uint32_t etu_us = sensor_us & 0x0000000000FFFFFF;
    if (*last_evt_stamp == 0 || (*last_evt_stamp != (etu_us & 0xFFFFF000))) {
      EVT3::TimeHigh e3th(etu_us);
      *last_evt_stamp = e3th.write(out);
    }
    // write last digits of time event if first event in ROS message,
    // or if the event time stamp has changed,
    if (i == 0 || last_etu_us != etu_us) {
      EVT3::TimeLow e3tl(etu_us);
      e3tl.write(out);
      last_etu_us = etu_us;
    }
    EVT3::AddrY e3y(ey, 0);
    e3y.write(out);
    EVT3::AddrX e3x(ex, static_cast<uint8_t>(pol));
    e3x.write(out);
  }
  const size_t numEvents = s.events.size() / bytes_per_event;
  return (numEvents);
}

static size_t process_bag(
  const std::string & inFile, const std::string & outFile, const std::string & topic)
{
  std::fstream out;
  out.open(outFile, std::ios::out | std::ios::binary);
  out.write(header, strlen(header));
  size_t numEvents(0);
  {
    rosbag2_cpp::Reader reader;
    reader.open(inFile);
    rclcpp::Serialization<EventArray> serialization;
    uint32_t last_evt_stamp(0);
    while (reader.has_next()) {
      auto msg = reader.read_next();
      if (msg->topic_name == topic) {
        rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
        EventArray m;
        serialization.deserialize_message(&serializedMsg, &m);
        numEvents += write_evt_3(out, m, &last_evt_stamp);
      }
    }
  }  // close reader when out of scope
  return (numEvents);
}
}  // namespace metavision_ros_tools

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/events");
  while ((opt = getopt(argc, argv, "b:o:t:")) != -1) {
    switch (opt) {
      case 'b':
        inFile = optarg;
        break;
      case 'o':
        outFile = optarg;
        break;
      case 't':
        topic = optarg;
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
  const size_t numEvents = metavision_ros_tools::process_bag(inFile, outFile, topic);
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "number of events read: " << numEvents * 1e-6 << " Mev in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "event rate: " << static_cast<double>(numEvents) / total_duration.count() << " Mevs"
            << std::endl;
  return (0);
}
