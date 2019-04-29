#pragma once
#include<WINSOCK2.H>
#include<STDIO.H>
#include<iostream>
#include<cstring>
#include <fstream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
using namespace std;
#pragma comment(lib, "ws2_32.lib")

#define BACKLOG 5     // 最多支持多少客户端同时在线




