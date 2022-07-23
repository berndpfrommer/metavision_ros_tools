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
#include <event_array_msgs/EventArray.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>


#include "evt_3_utils.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_raw -b name_of_bag_file -o name_of_raw_file -t topic "
            << std::endl;
}

namespace metavision_ros_tools
{
using event_array_msgs::mono::bytes_per_event;
using event_array_msgs::mono::decode_t_x_y_p;
using event_array_msgs::EventArray;

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

static size_t process_bag(
  const std::string & inFile, const std::string & outFile,
  const std::string & topic)
{
  std::fstream out;
  out.open(outFile, std::ios::out | std::ios::binary);
  out.write(header, strlen(header));
  size_t numEvents(0);
  size_t numMessages(0);
  {
    rosbag::Bag bag;
    std::cout << "reading from bag: " << inFile << std::endl;
    bag.open(inFile, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery({topic}));
    uint32_t last_evt_stamp(0);
    for (const rosbag::MessageInstance & m : view) {
      if (m.getTopic() == topic) {
        EventArray::ConstPtr ea = m.instantiate<EventArray>();
        if (ea) {
          numEvents += evt_3_utils::write(
            out, &ea->events[0], ea->events.size(), ea->time_base, ea->encoding,
            &last_evt_stamp);
          numMessages++;
        }
      }
    }
    bag.close();
  }
  std::cout << "read " << numMessages << " messages with "
            << numEvents / numMessages << " events/msg" << std::endl;
  return (numEvents);
}
}  // namespace metavision_ros_tools

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string outFile;
  std::string topic("/event_camera/events");
  while ((opt = getopt(argc, argv, "b:o:t:h")) != -1) {
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
      case 'h':
        usage();
        return (-1);
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
  const size_t numEvents =
    metavision_ros_tools::process_bag(inFile, outFile, topic);
  auto final = std::chrono::high_resolution_clock::now();
  auto total_duration =
    std::chrono::duration_cast<std::chrono::microseconds>(final - start);

  std::cout << "number of events read: " << numEvents * 1e-6 << " Mev in "
            << total_duration.count() * 1e-6 << " seconds" << std::endl;
  std::cout << "event rate: "
            << static_cast<double>(numEvents) / total_duration.count()
            << " Mevs" << std::endl;

  return (0);
}
