// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <stdio.h>
#include <memory>
#include <functional>
#include <thread>
#include <string.h>
#include <chrono>
#include "tclap/CmdLine.h"
#include <cmath>

#define EXIT_NAN 2

using namespace TCLAP;

int main(int argc, char * argv[]) try
{
    // Parse command line arguments
    CmdLine cmd("librealsense rs-record tool", ' ');
    ValueArg<int>    time("t", "Time", "Amount of time to record (in seconds)", false, 10, "");
    ValueArg<std::string> out_file("f", "FullFilePath", "File that the recording will be saved to", false, "test.bag", "");
    SwitchArg disable_tracking("d", "DisableTracking", "Disables the tracking algorithm (only streams raw data)");
    cmd.add(time);
    cmd.add(out_file);
    cmd.add(disable_tracking);
    cmd.parse(argc, argv);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_record_to_file(out_file.getValue());
    if (disable_tracking.getValue()) {
        //cfg.disable_stream(RS2_STREAM_POSE);
        cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
        cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
        cfg.enable_stream(RS2_STREAM_ACCEL, 0, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO, 0, RS2_FORMAT_MOTION_XYZ32F);
        // won't save extrinsics/ pose frame /device_0/sensor_0/Pose_0/tf/0
    }

    std::mutex m;
    bool nan = false;
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(m);

        auto t = std::chrono::system_clock::now();
        static auto tk = t;
        static auto t0 = t;
        if (t - tk >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(3) << std::fixed
                      << "Recording t = "  << std::chrono::duration_cast<std::chrono::seconds>(t-t0).count() << "s" << std::flush;
            tk = t;
        }

        if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
            rs2_pose pose_data = fp.get_pose_data();
            if (std::isnan(pose_data.translation.x)) {
                nan = true; // exit callback before pipe.stop
            }
        }
    };

    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    auto t = std::chrono::system_clock::now();
    auto t0 = t;
    while(t - t0 <= std::chrono::seconds(time.getValue())) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        t = std::chrono::system_clock::now();
    }
    
    pipe.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for clean-up
    std::cout << "\nSaved to " << out_file.getValue() << std::endl;
    std::lock_guard<std::mutex> lock(m);
    return nan ? EXIT_NAN : EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
