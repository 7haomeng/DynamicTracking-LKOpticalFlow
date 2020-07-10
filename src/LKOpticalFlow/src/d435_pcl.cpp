// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
//#pragma comment(lib, "ws2_32.lib")
//#pragma comment(lib, "mswsock.lib")
//#pragma comment(lib, "advapi32.lib")
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// #include "librealsense2/hpp/rs_frame.hpp"
// #include "example.hpp"          // Include short list of convenience functions for rendering
// #include <algorithm>            // std::min, std::max

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>


#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
#include <chrono>

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;

// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();

	// convert normals [u v] to basic coords [x y]
	int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color) {


	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Config of PCL Cloud object
	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto tex_coords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	// and RGB values
	for (int i = 0; i < points.size(); ++i)
	{
		cloud->points[i].x = vertices[i].x;
		cloud->points[i].y = vertices[i].y;
		cloud->points[i].z = vertices[i].z;

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);
		//current_color= std::tuple<uint8_t, uint8_t, uint8_t>(150, 0, 0);

		cloud->points[i].r = std::get<0>(current_color);
		cloud->points[i].g = std::get<1>(current_color);
		cloud->points[i].b = std::get<2>(current_color);

	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points) {


	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Config of PCL Cloud object
	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto tex_coords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	// and RGB values
	
	for (int i = 0; i < points.size(); ++i)
	{
		if ((vertices[i].x*vertices[i].x + vertices[i].y*vertices[i].y + vertices[i].z*vertices[i].z) < 4) {
			cloud->points[i].x = vertices[i].x;
			cloud->points[i].y = vertices[i].y;
			cloud->points[i].z = vertices[i].z;
		}
		
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudt(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.01, 0.01, 0.01);
	sor.filter(*cloudt);

	Eigen::Matrix4f Trans = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf( 180/180 * M_PI, Eigen::Vector3f::UnitX())* Eigen::AngleAxisf(0 / 180 * M_PI, Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(0 / 180 * M_PI, Eigen::Vector3f::UnitZ());
	Trans.block<3, 3>(0, 0) = m;
	pcl::transformPointCloud(*cloudt, *cloud, Trans);
	return cloud;
}

void voxelgrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
	pcl::PCLPointCloud2::Ptr cloud_infiltered(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*cloud_in, *cloud_infiltered);
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_infiltered);
	sor.setLeafSize(0.1, 0.1, 0.1);
	sor.filter(*cloud_filtered);
	//std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height	<< " data points (" << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
	pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_out);

}

// Helper functions
// void register_glfw_callbacks(window& app, glfw_state& app_state);

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	// window app(1280, 720, "RealSense Pointcloud Example");
	// Construct an object to manage view state
	// glfw_state app_state;
	// register callbacks to allow manipulation of the pointcloud
	// register_glfw_callbacks(app, app_state);

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start();
	auto i = 0;
	/*
	Eigen::Matrix4f Trans = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitX())* Eigen::AngleAxisf(0 / 180 * M_PI, Eigen::Vector3f::UnitY())* Eigen::AngleAxisf(0 / 180 * M_PI, Eigen::Vector3f::UnitZ());
	Trans.block<3, 3>(0, 0) = m;
	std::cerr << m << std::endl << Trans << std::endl;
	*/
	while (1) // Application still alive?
	{
		i++;
		// Wait for the next set of frames from the camera
		auto frames = pipe.wait_for_frames();//rs2::frameset

		auto color = frames.get_color_frame();
		
		// For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
		if (!color)
			color = frames.get_infrared_frame();

		// Tell pointcloud object to map to this color frame
		pc.map_to(color);

		auto depth = frames.get_depth_frame();

		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);

		if (i > 1) {
			// Actual calling of conversion and saving XYZRGB cloud to file
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = points_to_pcl(points, color);
			auto start = std::chrono::high_resolution_clock::now();

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = points_to_pcl(points);
			auto elapsed = std::chrono::high_resolution_clock::now() - start;
			long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
			std::cout<<microseconds*1e-6<<std::endl;
			std::string name = "cloud_test_" + std::to_string(i % 10) + ".pcd";
			// pcl::io::savePCDFileASCII(name, *cloud);
			/*
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter;
			//voxelgrid(cloud, cloud_filter);
			
			pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
			pcl::PCLPointCloud2::Ptr cloud_2f(new pcl::PCLPointCloud2());
			//pcl::transformPointCloud(*cloud, *cloud_filter, Trans);
			pcl::toPCLPointCloud2(*cloud, *cloud2);
			pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
			sor.setInputCloud(cloud2);
			sor.setLeafSize(0.01, 0.01, 0.01);
			sor.filter(*cloud_2f);
			pcl::fromPCLPointCloud2(*cloud_2f, *cloud);
			*/
			//*cloud_filter = *cloud;
			//pcl::transformPointCloud(*cloud_filter, *cloud, Trans);
			//pcl::transformPointCloud(*cloud, *cloud_filter, Trans);
			//*cloud = *cloud_filter;
			
			// pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
			// viewer.addPointCloud(cloud);
			// viewer.setBackgroundColor(0, 0, 0);

			// viewer.spin();
			// system("pause");
			
			//pcl::io::savePCDFileASCII("cloud_test.pcd", *cloud);
			//i = 0;
			//break;
		}
		// Upload the color frame to OpenGL

		// app_state.tex.upload(color);

		// Draw the pointcloud
		// draw_pointcloud(app.width(), app.height(), app_state, points);
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
