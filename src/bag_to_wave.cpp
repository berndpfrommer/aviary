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

#include <audio_common_msgs/msg/audio_data_stamped.hpp>
#include <aviary/bag_processor.hpp>
#include <aviary/message_processor.hpp>
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
#include <sstream>

#include "logging.hpp"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "bag_to_wave -b input_bag [-t topic] [-o out_file] "
            << "[-T timestamp_file] [-s start_time] [-e end_time] "
            << "[-E encoding] [-c channels] [-r rate]" << std::endl;
}

using audio_common_msgs::msg::AudioDataStamped;
using rclcpp::Time;
using bag_time_t = rcutils_time_point_value_t;

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

class FileWriter : public aviary::MessageProcessor<AudioDataStamped>
{
public:
  FileWriter(const std::string & raw_file, const std::string & ts_file)
  {
    std::cout << "writing to raw file: " << raw_file << std::endl;
    raw_file_ = std::ofstream(raw_file, std::ios::out | std::ios::binary);
    ts_file_.open(ts_file);
  }

  void process(uint64_t t, const AudioDataStamped::ConstSharedPtr & m) final
  {
    const auto t_header = Time(m->header.stamp).nanoseconds();
    raw_file_.write(reinterpret_cast<const char *>(m->audio.data.data()), m->audio.data.size());
    ts_file_ << packet_number_++ << " "
             << " " << t_header << " " << t << std::endl;
  }

private:
  std::ofstream raw_file_;
  std::ofstream ts_file_;
  size_t packet_number_{0};
};

#define USE_WAVEPAK

int main(int argc, char ** argv)
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

  bag_time_t start_time = std::numeric_limits<bag_time_t>::min();
  bag_time_t end_time = std::numeric_limits<bag_time_t>::max();
  int channels(24);
  while ((opt = getopt(argc, argv, "b:c:e:E:o:O::s:t:T:h")) != -1) {
    switch (opt) {
      case 'b':
        bag = optarg;
        break;
      case 'c':
        channels = atoi(optarg);
        break;
      case 'e':
        end_time = static_cast<bag_time_t>(atof(optarg) * 1e9);
        if (end_time < 0) {
          std::cout << "end time out of range, must be in seconds since start of epoch"
                    << std::endl;
          usage();
          return (-1);
        }
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
        start_time = static_cast<bag_time_t>(atof(optarg) * 1e9);
        if (start_time < 0) {
          std::cout << "start time out of range, must be in seconds since start of epoch"
                    << std::endl;
          usage();
          return (-1);
        }
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
  if (bag.empty()) {
    std::cout << "missing bag file argument!" << std::endl;
    usage();
    return (-1);
  }

  const std::string raw_file = out_file + ".tmp";
  const std::string topic_type = "audio_common_msgs/msg/AudioDataStamped";
  aviary::BagProcessor<AudioDataStamped> bproc(bag, topic, topic_type, start_time, end_time);
  FileWriter fw(raw_file, time_stamp_file);
  bproc.process(&fw);
  convertToWavpack(raw_file, out_file, enc, out_enc, rate, channels);
  return (0);
}
