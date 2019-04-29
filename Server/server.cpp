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
#include <thread>
#include <mutex>

#ifdef WIN32
# define sleep(x) Sleep((x)*1000)
#endif

class Server
{
public:
	Server() :
		viewer(" Point Cloud Compression Example")
		,frameCount(0)
	{
		
	}

	int openServer()
	{
		//初始化WSA  
		WORD sockVersion = MAKEWORD(2, 2);
		WSADATA wsaData;
		if (WSAStartup(sockVersion, &wsaData) != 0)
		{
			return 0;
		}

		//创建套接字  
		SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (slisten == INVALID_SOCKET)
		{
			printf("socket error !");
			return 0;
		}

		//绑定IP和端口  
		sockaddr_in sin;
		sin.sin_family = AF_INET;
		sin.sin_port = htons(8888);
		sin.sin_addr.S_un.S_addr = INADDR_ANY;
		if (::bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)//用;;bind避免函数重名
		{
			printf("bind error !");
		}

		if (listen(slisten, BACKLOG + 1) == -1)//实际可监听的比最大连接数参数大1，为了避免一些bug
		{
			printf("listen error!\n");
		}
		std::cout << "Listening" << std::endl;
		conn_amount = 0;

		maxsock = slisten;

		sock_fd = slisten;//将服务器也存入列表

		while (true) {
			try
			{
				// 初始化文件句柄集合，因为每次select都会更改这个集合
				FD_ZERO(&fdsr);
				FD_SET(sock_fd, &fdsr);//把socket文件描述符添加到文件描述符列表中以监测他的变化

				// 设定select的超时时间，因为每次select都会清除这个值，所以设在循环内
				tv.tv_sec = 2;//秒
				tv.tv_usec = 0;//毫秒

				char revData[255];//从主机收到的数据
				// 检测仍在线的客机，并把他们添加到文件句柄集合
				for (int i = 0; i < BACKLOG; i++)
				{
					if (clientList[i] != 0)
					{
						FD_SET(clientList[i], &fdsr);
					}
				}

				int ret = select(maxsock + 1, &fdsr, NULL, NULL, &tv);//只设置fdsr，就是在监视是否有套接字接收到信息
				if (ret < 0)
				{//select出现错误，直接跳出while循环清除尚未清除的文件描述符
					perror("select");
					break;
				}
				else if (ret == 0)
				{//等于0表示出现超时，进入下一次轮询
					printf("timeout\n");
					continue;
				}
				mutex_lock.lock();
				// 检测每一个客户端是否发送了新的信息
				for (int i = 0; i < conn_amount; i++)
				{
					if (FD_ISSET(clientList[i], &fdsr))
					{//当某个客机的文件标示符发生改变时表示新发送了信息
						ret = recv(clientList[i], revData, sizeof(revData), 0);
						//printf("%s\n", revData);
						if (ret <= 0)
						{        // 客户端关闭处理
							printf("client[%d] close\n", i);
							close(clientList[i]);
							FD_CLR(clientList[i], &fdsr);
							clientList[i] = 0;
							clientReady[i] = 0;
							memset(clientName[i], '\0', 1);
							conn_amount--;
						}
						else
						{
						if (strncmp(revData, "Require next Frame!", sizeof("Require next Frame!")) == 0)
						{
							cout << "客户端" << clientName[i] << "准备好开始接收下一帧" << endl;
							clientReady[i] = 1;//将接收标志位维护为准备好开始接收
						}

						revData[0] = '\0';
						}
					}
				}
				
				// 检测是否有新的客户端发送连接请求
				if (FD_ISSET(sock_fd, &fdsr))
				{//当主机的文件标示符发生了改变时表示客机发过来了连接请求
					new_fd = accept(sock_fd, (struct sockaddr *)&client_addr, &sin_size);
					if (new_fd <= 0)
					{
						perror("accept");
						continue;
					}

					// 添加到客机列表中
					if (conn_amount < BACKLOG)
					{
						for (int i = 0; i < BACKLOG; i++)
						{
							if (clientList[i] == 0)
							{
								clientList[i] = new_fd;//完成添加
								char *temp = inet_ntoa(client_addr.sin_addr);//刷新客机名字
								strcpy(clientName[i], temp);
								break;
							}
						}
						conn_amount++;
						printf("new connection client[%d] %s:%d\n", conn_amount,
							inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
						if (new_fd > maxsock)
							maxsock = new_fd;//如果分配了值更大的句柄值，更新maxsock的值
					}
					else
					{
						printf("max connections arrive, exit\n");
						send(new_fd, "Sorry!List of service is full!", 31, 0);
						close(new_fd);
						continue;
					}
				}
				mutex_lock.unlock();
			}
			catch (const std::exception&)//出现异常释放所有资源并退出
			{
				for (int i = 0; i < BACKLOG; i++)
				{
					if (clientList[i] != 0) {
						closesocket(clientList[i]);
					}
				}
				close(slisten);
				WSACleanup();

				return 0;
			}
		}

		for (int i = 0; i < BACKLOG; i++)
		{
			if (clientList[i] != 0) {
				closesocket(clientList[i]);
			}
		}
		WSACleanup();
		return 0;
	}

	//捕捉到新的一帧事件的处理函数
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		mutex_lock.lock();
		auto cloudTemp = new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud);//创建一个临时存储值，以释放cloud资源
					
		//pcl::io::savePCDFile("Point Cloud Compression Example.pcd", *cloud);//暂时不需要保存

		// stringstream to store compressed point cloud
		std::stringstream compressedData;

		bool flag = false;
		int readyTemp[BACKLOG] = { 0 };
		for (int i = 0; i < BACKLOG; i++)
		{
			if (clientReady[i] == 1)
			{
				flag = true;
				clientReady[i] = 0;
				readyTemp[i] = 1;//临时保存，并在压缩之前就清0clientReady标志位，避免时序bug
			}
		}
		mutex_lock.unlock();

		pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
		bool showStatistics = true;

		// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

		// instantiate point cloud compression for encoding and decoding
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		if (!viewer.wasStopped())
		{
			viewer.showCloud(cloud);//更新下一帧
			printf("now is frame%d\n", ++frameCount);
		}

		// compress point cloud
		if (flag)
		{
			PointCloudEncoder->encodePointCloud(cloud, compressedData);
			std::string sendData;
			char buffer[255] = { '\0' };
			sendData = compressedData.str();

			for (int i = 0; i < BACKLOG; i++)
			{
				if (readyTemp[i] == 1)
				{
					//int ret = send(clientList[i], "FRAME", sizeof("FRAME"), 0);//暂时去掉握手过程
					//if (ret <= 0)
					//{
					//	printf("send error!\n");
					//	continue;
					//}
					//ret = recv(clientList[i], buffer, 254,0);
					//if ((ret > 0) && (strncmp("RECIEVED", buffer, 8) == 0))
					int ret = send(clientList[i], sendData.c_str(), sendData.length(), 0);
					if (ret <= 0)
					{
						printf("send error!\n");
						continue;
					}
				}
			}
		}
		delete (PointCloudEncoder);
	}


	void run()
	{
		std::thread myThread;
		myThread = std::thread(&Server::openServer, this);
		
		//bool showStatistics = true;

		//// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
		//pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

		//// instantiate point cloud compression for encoding and decoding
		//PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		//PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();


		// create a new grabber for OpenNI2 devices//
		pcl::Grabber* grabber = new pcl::Kinect2Grabber();

		// make callback function from member function
		boost::function<void
		(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&Server::cloud_cb_, this, _1);//使用function和bind创建一个新的函数为注册回调函数做准备

		// connect callback function for desired signal. In this case its a point cloud with color values
		boost::signals2::connection c = grabber->registerCallback(f);//grabber类中包含一个产生信号槽的方法，将f作为回调函数注册进去，以方便触发信号的时候进行注册。

		// start receiving point clouds
		std::cout << "Start receiving point clouds" << std::endl;
		grabber->start();

		while (!viewer.wasStopped())//在主线程上等待
		{
			sleep(0.01);
		}

		grabber->stop();
		
		// delete point cloud compression instances
		//delete (PointCloudEncoder);
		//delete (PointCloudDecoder);

	}

	pcl::visualization::CloudViewer viewer;

	//pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
	//pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;

	private:
		SOCKET sock_fd, new_fd;//最近连接的套接字		
		sockaddr_in client_addr;
		int sin_size = sizeof(client_addr);

		int conn_amount;    // 当前在线的客户端的数目
		int clientList[BACKLOG] = { 0 };    // 客户端的句柄列表
		char clientName[BACKLOG][32];//在线的客户端的名字
		int clientReady[BACKLOG] = { 0 };
		int frameCount;
		fd_set fdsr;//新建文件句柄
		int maxsock;//指集合中所有文件描述符的范围，即所有文件描述符的最大值加1
		struct timeval tv;
		std::mutex mutex_lock;
};

int
main()
{
	Server v;
	//std::thread newThread{ v.openServer() };//在新线程上启动监听
	v.run();
	
	return (0);
}
