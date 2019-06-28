// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max
#include <fstream>              // std::ifstream
#include <iomanip>

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char * argv[]) try
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Tracking and Depth Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    double ts_depth;
    // store pose and timestamp
    rs2::pose_frame pose_frame(nullptr);

    rs2::context                ctx;            // Create librealsense context for managing devices
    std::vector<rs2::pipeline>  pipelines;

    // Start a streaming pipe per each connected device
    for (auto&& dev : ctx.query_devices())
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
    }

    // extrinsics
    // depth w.r.t. tracking (column-major)
    float H_t265_d400[16] =  {1, 0, 0, 0,
                              0,-1, 0, 0,
                              0, 0,-1, 0,
                              0, 0, 0, 1};
    std::string fn = "H_t265_d400.txt";
    std::ifstream ifs(fn);
    if (!ifs.is_open())
        std::cout << "Couldn't open ./" << fn << std::endl;
    else {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                ifs >> H_t265_d400[i+4*j];  // row-major to column-major
            }
        }
    }

    uint64_t pose_counter = 0;
    uint64_t frame_counter = 0;
    uint64_t rendering_counter = 0;
    auto last_print = std::chrono::system_clock::now();
    while (app) // Application still alive?
    {
        for (auto &&pipe : pipelines) // loop over pipelines
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();


            auto color = frames.get_color_frame();

            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            if (!color)
                color = frames.get_infrared_frame();

            // Tell pointcloud object to map to this color frame
            if (color)
                pc.map_to(color);

            auto depth = frames.get_depth_frame();

            // Generate the pointcloud and texture mappings
            if (depth) {
                points = pc.calculate(depth);
                ts_depth = depth.get_timestamp();
                frame_counter++;
            }

            // Upload the color frame to OpenGL
            if (color)
                app_state.tex.upload(color);


            // pose
            auto pose = frames.get_pose_frame();
            if (pose) {
                pose_frame = pose;
                pose_counter++;
            }
        }

        // Draw the pointcloud
        if (points && pose_frame) {
            //timing
            double ts_pose = pose_frame.get_timestamp();
            std::cout << "ts_{depth,pose}_ms: " << std::setprecision(16) << ts_depth << ", " << ts_pose << " dt_ms (depth wrt pose) = " << ts_depth - ts_pose << std::endl;
            rs2_pose pose =  pose_frame.get_pose_data();
            draw_pointcloud_wrt_world(app.width(), app.height(), app_state, points, pose, H_t265_d400);
            rendering_counter++;
        }

        // Print the approximate pose and image rates once per second
        auto now = std::chrono::system_clock::now();
        if (now - last_print >= std::chrono::seconds(1)) {
            std::cout << std::setprecision(0) << std::fixed
                      << "Pose rate: "  << pose_counter << " "
                      << "Image rate: "  << frame_counter << " "
                      << "Rendering rate: " << rendering_counter << std::endl;
            pose_counter = 0;
            frame_counter = 0;
            rendering_counter = 0;
            last_print = now;
        }
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
