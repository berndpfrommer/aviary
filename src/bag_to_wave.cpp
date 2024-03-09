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

#include "logging.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
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
  std::cout << "bag_to_wave -b input_bag [-t topic] [-o out_file] "
            << "[-T timestamp_file] [-s start_time] [-e end_time] "
            << "[-E encoding] [-c channels] [-r rate]"
            << std::endl;
}

using audio_common_msgs::msg::AudioDataStamped;
using Path = std::filesystem::path;
using rclcpp::Time;

static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("bag_to_wave"));
}

static void convertToWavpack(const std::string &raw, const std::string &wavpak,
                             const std::string &enc, const std::string &out_enc,
                             double rate, int channels) {
      std::stringstream ss;
      ss << "ffmpeg -y -channel_layout " << channels << " -f " << enc << " -ar "
         << rate << " -ac " << channels
         << " -i " << raw << " -c:a " << out_enc << " " << wavpak;
      int rc = std::system(ss.str().c_str());
      if (rc == -1) {
        LOG_ERROR(" error command: "<< ss.str());
      } else {
        LOG_INFO("successfully ran command: "<< ss.str());
      }

      rc = std::system(("rm " +raw).c_str());
      if (rc == -1) {
        LOG_ERROR(" error removing file: " << raw);
      }
}



void processBag(const std::string &out_file, const std::string &bag,
                const std::string &topic, const std::string &time_stamp_file,
                double start_time, double end_time)
{
  LOG_INFO("opening bag: " << bag << " topic: " << topic);
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rclcpp::Serialization<AudioDataStamped> serialization;
  std::ofstream ts_file(time_stamp_file);
  std::fstream raw_file;
  raw_file.open(out_file, std::ios::app | std::ios::binary);
  LOG_INFO("opening bag done.");

  size_t sample_number{0};
  size_t packet_number{0};
  while (reader.has_next())
  {
    auto msg = reader.read_next();
    if (!msg || topic != msg->topic_name)
    {
      continue;
    }
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    AudioDataStamped::SharedPtr m(new AudioDataStamped());
    serialization.deserialize_message(&serialized_msg, m.get());
    raw_file.write(reinterpret_cast<char*>(m->audio.data.data()), m->audio.data.size());
    ts_file << packet_number << " " << Time(m->header.stamp).nanoseconds()
            << " " << Time(msg->time_stamp).nanoseconds() << std::endl;
    
    packet_number++;
    sample_number += 1;
  }
}

#define USE_WAVEPAK

int main(int argc, char **argv)
{
  int opt;
  double rate(48000);
  std::string bag;
  std::string enc = "s24le";
#ifdef USE_WAVEPAK
  std::string out_enc = "wavpack";
  std::string out_file = "audio.wv";
#else
  std::string out_enc = "pcm_" + enc;
  std::string out_file = "audio.wav";
#endif  
  std::string topic = "/audio/audio_stamped";
  std::string time_stamp_file = "timestamps.txt";

  double start_time(0);
  double end_time(std::numeric_limits<double>::max());
  int channels(24);
  while ((opt = getopt(argc, argv, "b:c:e:E:o:O::s:t:T:h")) != -1)
  {
    switch (opt)
    {
    case 'b':
      bag = optarg;
      break;
    case 'c':
      channels = atoi(optarg);
      break;
    case 'e':
      end_time = atof(optarg);
      break;
    case 'E':
      enc = optarg;
      break;
    case 'o':
      out_file = atof(optarg);
      break;
    case 'O':
      out_enc = optarg;
      break;
    case 's':
      start_time = atof(optarg);
      break;
    case 'r':
      rate = atof(optarg);
      break;
    case 't':
      topic = optarg;
      break;
    case 'T':
      time_stamp_file = optarg;
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
  const std::string raw_file = out_file + ".tmp";
  processBag(raw_file, bag, topic, time_stamp_file, start_time, end_time);
  convertToWavpack(raw_file, out_file, enc, out_enc, rate, channels);
  const auto stop = std::chrono::high_resolution_clock::now();
  auto total_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "total time for processing: " << total_duration.count() * 1e-6
            << std::endl;
  return (0);
}
