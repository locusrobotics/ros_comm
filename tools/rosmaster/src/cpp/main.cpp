// Software License Agreement (BSD License)
//
// Copyright (c) 2025, Locus Robotics
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Locus Robotics nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/// @file main.cpp
/// @brief Entry point for the C++ ROS Master (rosmaster)

#include "rosmaster/master.h"
#include "rosmaster/master_handler.h"

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

namespace
{
volatile std::sig_atomic_t g_shutdown_requested = 0;
rosmaster::Master* g_master = nullptr;

void safe_tolower(std::string& str)
{
  for (auto& c : str)
  {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
}

void signalHandler(int /*signal*/)
{
  g_shutdown_requested = 1;
}

void printUsage(const char* prog)
{
  std::cout << "Usage: " << prog << " [options]\n"
            << "Options:\n"
            << "  --core                Run as core\n"
            << "  -p, --port PORT       Override port (default: 11311)\n"
            << "  -w, --numworkers N    Number of worker threads (default: 3)\n"
            << "  -t, --timeout SECS    Socket connection timeout\n"
            << "  --master-logger-level LEVEL  Set log level (debug, info, warn, error, fatal)\n"
            << "  -h, --help            Show this help\n";
}

/// Read the rosmaster log level from ROS_PYTHON_LOG_CONFIG_FILE if it exists.
/// Returns true and sets level if successful.
bool readLogLevelFromConfig(rosmaster::LogLevel& level)
{
  const char* config_file = std::getenv("ROS_PYTHON_LOG_CONFIG_FILE");
  if (!config_file || !config_file[0])
  {
    return false;
  }

  try
  {
    YAML::Node config = YAML::LoadFile(config_file);
    YAML::Node rosmaster_logger = config["loggers"]["rosmaster"];
    if (!rosmaster_logger || !rosmaster_logger["level"])
    {
      return false;
    }

    std::string level_str = rosmaster_logger["level"].as<std::string>();
    safe_tolower(level_str);

    if (level_str == "debug")
      level = rosmaster::LogLevel::DEBUG;
    else if (level_str == "info")
      level = rosmaster::LogLevel::INFO;
    else if (level_str == "warning" || level_str == "warn")
      level = rosmaster::LogLevel::WARN;
    else if (level_str == "error")
      level = rosmaster::LogLevel::ERROR;
    else if (level_str == "critical" || level_str == "fatal")
      level = rosmaster::LogLevel::FATAL;
    else
      return false;

    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "[WARN] [rosmaster]: Failed to read log config from "
              << config_file << ": " << e.what() << std::endl;
    return false;
  }
}

}  // anonymous namespace

int main(int argc, char** argv)
{
  int port = rosmaster::DEFAULT_MASTER_PORT;
  int num_workers = 3;
  bool is_core = false;

  // Read log level from ROS_PYTHON_LOG_CONFIG_FILE (can be overridden by --master-logger-level)
  rosmaster::LogLevel config_level;
  if (readLogLevelFromConfig(config_level))
  {
    rosmaster::setLogLevel(config_level);
  }

  // Simple argument parsing
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--core")
    {
      is_core = true;
    }
    else if ((arg == "-p" || arg == "--port") && i + 1 < argc)
    {
      try
      {
        port = std::stoi(argv[++i]);
      }
      catch (const std::exception&)
      {
        std::cerr << "Invalid port value: " << argv[i] << std::endl;
        printUsage(argv[0]);
        return 1;
      }
    }
    else if ((arg == "-w" || arg == "--numworkers") && i + 1 < argc)
    {
      try
      {
        num_workers = std::stoi(argv[++i]);
      }
      catch (const std::exception&)
      {
        std::cerr << "Invalid numworkers value: " << argv[i] << std::endl;
        printUsage(argv[0]);
        return 1;
      }
    }
    else if ((arg == "-t" || arg == "--timeout") && i + 1 < argc)
    {
      // Timeout is noted but not critical for the C++ implementation
      ++i;
    }
    else if (arg == "--master-logger-level" && i + 1 < argc)
    {
      std::string level_str = argv[++i];
      safe_tolower(level_str);

      if (level_str == "debug")
        rosmaster::setLogLevel(rosmaster::LogLevel::DEBUG);
      else if (level_str == "info")
        rosmaster::setLogLevel(rosmaster::LogLevel::INFO);
      else if (level_str == "warn")
        rosmaster::setLogLevel(rosmaster::LogLevel::WARN);
      else if (level_str == "error")
        rosmaster::setLogLevel(rosmaster::LogLevel::ERROR);
      else if (level_str == "critical" || level_str == "fatal")
        rosmaster::setLogLevel(rosmaster::LogLevel::FATAL);
       else
         std::cerr << "[WARN] [rosmaster]: --master-logger-level received unknown option '" << level_str << "'\n";
    }
    else if (arg == "-h" || arg == "--help")
    {
      printUsage(argv[0]);
      return 0;
    }
    else if (arg.find("__log:=") == 0)
    {
      // ROS log remapping - ignore
    }
    else
    {
      std::cerr << "Unrecognized argument: " << arg << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  if (!is_core)
  {
    std::cout << "\n\nACHTUNG WARNING ACHTUNG WARNING ACHTUNG\n"
              << "WARNING ACHTUNG WARNING ACHTUNG WARNING\n\n\n"
              << "Standalone zenmaster has been deprecated, please use 'roscore' instead\n\n\n"
              << "ACHTUNG WARNING ACHTUNG WARNING ACHTUNG\n"
              << "WARNING ACHTUNG WARNING ACHTUNG WARNING\n\n"
              << std::endl;
  }

  // Set up signal handling
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  try
  {
    rosmaster::Master master(port, num_workers);
    master.start();
    g_master = &master;

    while (master.ok() && !g_shutdown_requested)
    {
      master.spinOnce(100.0);
    }

    g_master = nullptr;
    master.stop();
  }
  catch (const std::exception& e)
  {
    std::cerr << "[FATAL] [rosmaster]: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
