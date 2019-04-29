#pragma once
#include <stdio.h>  
#include <winsock2.h>
#include <ostream>
#include <string>
#include <fstream>

#include <pcl/io/ply_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#pragma comment(lib,"ws2_32.lib") 
#pragma warning(disable: 4996)

SOCKET _connect(char *);