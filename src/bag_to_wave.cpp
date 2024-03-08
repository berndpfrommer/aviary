// -*-c++-*--------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <audio_common_msgs/msg/audio_data_stamped.hpp>

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_wave -b input_bag -t topic [-T timestamp_file] [-s start_time] [-e end_time]"
            << std::endl;
}

using audio_common_msgs::msg::AudioDataStamped;
using Path = std::filesystem::path;
using rclcpp::Time;

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("bag_to_wave"));
}

void processBag(const std::string &bag, const std::string &topic, const std::string &time_stamp_file, double start_time, double end_time)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rclcpp::Serialization<AudioDataStamped> serialization;
  std::ofstream ts_file(time_stamp_file);

  size_t sample_number{0};
  size_t packet_number{0};
  while (reader.has_next())
  {
    auto msg = reader.read_next();
    if (topic != msg->topic_name)
    {
      continue;
    }
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    AudioDataStamped::SharedPtr m(new AudioDataStamped());
    serialization.deserialize_message(&serialized_msg, m.get());
    const uint64_t t = Time(m->header.stamp).nanoseconds();
    ts_file << t << " " << packet_number << std::endl;
    packet_number++;
    sample_number += 1;
  }
}

int main(int argc, char **argv)
{
  int opt;

  std::string bag;
  std::string topic = "/audio/audio_stamped";
  std::string time_stamp_file = "timestamps.txt";
  double start_time(0), end_time(std::numeric_limits<double>::max());
  while ((opt = getopt(argc, argv, "b:t:o:s:e:h")) != -1)
  {
    switch (opt)
    {
    case 'b':
      bag = optarg;
      break;
    case 'o':
      time_stamp_file = optarg;
      break;
    case 't':
      topic = optarg;
      break;
    case 's':
      start_time = atof(optarg);
      break;
    case 'e':
      end_time = atof(optarg);
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
  if (bag.empty())
  {
    std::cout << "missing bag file argument!" << std::endl;
    usage();
    return (-1);
  }

  const auto start = std::chrono::high_resolution_clock::now();
  processBag(bag, topic, time_stamp_file, start_time, end_time);
  const auto stop = std::chrono::high_resolution_clock::now();
  auto total_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "total time for processing: " << total_duration.count() * 1e-6
            << std::endl;
  return (0);
}
