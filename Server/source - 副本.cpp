
#define _SCL_SECURE_NO_WARNINGS
#pragma warning(disable: 4996)

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "Kinect2_grabber.h"
#include "server.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
# define sleep(x) Sleep((x)*1000)
#endif

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer() :
		viewer(" Point Cloud Compression Example")
	{
	}

	void
		cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped())
			viewer.showCloud(cloud);
		pcl::io::savePCDFile("Point Cloud Compression Example.pcd", *cloud);

		{
			// stringstream to store compressed point cloud
			std::stringstream compressedData;
			// output pointcloud
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

			// compress point cloud
			PointCloudEncoder->encodePointCloud(cloud, compressedData);

			// decompress point cloud
			PointCloudDecoder->decodePointCloud(compressedData, cloudOut);




			//show decompressed point cloud
			viewer.showCloud(cloudOut);
		}
	}
	void
		run()
	{

		bool showStatistics = true;

		// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

		// instantiate point cloud compression for encoding and decoding
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();


		// create a new grabber for OpenNI2 devices//
		pcl::Grabber* grabber = new pcl::Kinect2Grabber();

		// make callback function from member function
		boost::function<void
		(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);//ʹ��function��bind����һ���µĺ���Ϊע��ص�������׼��

		// connect callback function for desired signal. In this case its a point cloud with color values
		boost::signals2::connection c = grabber->registerCallback(f);//grabber���а���һ�������źŲ۵ķ�������f��Ϊ�ص�����ע���ȥ���Է��㴥���źŵ�ʱ�����ע�ᡣ

		// start receiving point clouds
		grabber->start();

		while (!viewer.wasStopped())
		{
			sleep(0.1);
		}

		grabber->stop();

		// delete point cloud compression instances
		delete (PointCloudEncoder);
		delete (PointCloudDecoder);

	}

	pcl::visualization::CloudViewer viewer;

	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;

};

int
initialGrabber()
{
	SimpleOpenNIViewer v;
	v.run();

	return (0);
}
