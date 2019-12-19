// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout

#include <ros/ros.h>
#include <realsense_pack/Depth_real.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <sstream>

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char * argv[]) try
{

    // Init ROS node
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<realsense_pack::Depth_real>("depth", 1000);
    ros::Rate loop_rate(10);

    // Define Depth object
    realsense_pack::Depth_real depth_values;

    // Set depth frame resolution and frame rate
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start(cfg);

    // Initialize left, right, and center metric depth variables
    float left_dist = 0.0f;
    float right_dist = 0.0f;
    float center_dist = 0.0f;

    while(ros::ok())
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();

        /*
        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);

        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
        */
        //* 
        // Query the distance from the camera to the object in the center of the image
        
        for (auto i=1; i< 160; i++)
        {
            left_dist = left_dist + depth.get_distance(i, (height / 2) - 30);
            right_dist = right_dist + depth.get_distance(640-i, (height / 2) - 30);
        }

		for(auto i=160; i < 480; i++)
		{
			center_dist = center_dist + depth.get_distance(i,(height/2) - 30);
		}

        depth_values.left = left_dist/160;
        depth_values.right = right_dist/160;
        depth_values.center = center_dist/320;
        
        left_dist = 0.0f;
        right_dist = 0.0f;
        center_dist = 0.0f;
        //depth_values.left = depth.get_distance(0, (height / 2));
        //depth_values.right = depth.get_distance(320, (height / 2));
        //depth_values.center = depth.get_distance(639, (height / 2));

        // Print the distance
        //std::cout << "Left: " << depth_values.left << "Center: " << depth_values.center << "Right: " << depth_values.right << "\r";
        std::cout << "width: " << width << "height: " << height << "\r";
        //*/

        // Publish depth and color messages
        chatter_pub.publish(depth_values);
        ros::spinOnce();

        loop_rate.sleep();
    }
    

    return EXIT_SUCCESS;
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
