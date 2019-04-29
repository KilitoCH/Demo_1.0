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
		//��ʼ��WSA  
		WORD sockVersion = MAKEWORD(2, 2);
		WSADATA wsaData;
		if (WSAStartup(sockVersion, &wsaData) != 0)
		{
			return 0;
		}

		//�����׽���  
		SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (slisten == INVALID_SOCKET)
		{
			printf("socket error !");
			return 0;
		}

		//��IP�Ͷ˿�  
		sockaddr_in sin;
		sin.sin_family = AF_INET;
		sin.sin_port = htons(8888);
		sin.sin_addr.S_un.S_addr = INADDR_ANY;
		if (::bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)//��;;bind���⺯������
		{
			printf("bind error !");
		}

		if (listen(slisten, BACKLOG + 1) == -1)//ʵ�ʿɼ����ı����������������1��Ϊ�˱���һЩbug
		{
			printf("listen error!\n");
		}
		std::cout << "Listening" << std::endl;
		conn_amount = 0;

		maxsock = slisten;

		sock_fd = slisten;//��������Ҳ�����б�

		while (true) {
			try
			{
				// ��ʼ���ļ�������ϣ���Ϊÿ��select��������������
				FD_ZERO(&fdsr);
				FD_SET(sock_fd, &fdsr);//��socket�ļ���������ӵ��ļ��������б����Լ�����ı仯

				// �趨select�ĳ�ʱʱ�䣬��Ϊÿ��select����������ֵ����������ѭ����
				tv.tv_sec = 2;//��
				tv.tv_usec = 0;//����

				char revData[255];//�������յ�������
				// ��������ߵĿͻ�������������ӵ��ļ��������
				for (int i = 0; i < BACKLOG; i++)
				{
					if (clientList[i] != 0)
					{
						FD_SET(clientList[i], &fdsr);
					}
				}

				int ret = select(maxsock + 1, &fdsr, NULL, NULL, &tv);//ֻ����fdsr�������ڼ����Ƿ����׽��ֽ��յ���Ϣ
				if (ret < 0)
				{//select���ִ���ֱ������whileѭ�������δ������ļ�������
					perror("select");
					break;
				}
				else if (ret == 0)
				{//����0��ʾ���ֳ�ʱ��������һ����ѯ
					printf("timeout\n");
					continue;
				}
				mutex_lock.lock();
				// ���ÿһ���ͻ����Ƿ������µ���Ϣ
				for (int i = 0; i < conn_amount; i++)
				{
					if (FD_ISSET(clientList[i], &fdsr))
					{//��ĳ���ͻ����ļ���ʾ�������ı�ʱ��ʾ�·�������Ϣ
						ret = recv(clientList[i], revData, sizeof(revData), 0);
						//printf("%s\n", revData);
						if (ret <= 0)
						{        // �ͻ��˹رմ���
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
							cout << "�ͻ���" << clientName[i] << "׼���ÿ�ʼ������һ֡" << endl;
							clientReady[i] = 1;//�����ձ�־λά��Ϊ׼���ÿ�ʼ����
						}

						revData[0] = '\0';
						}
					}
				}
				
				// ����Ƿ����µĿͻ��˷�����������
				if (FD_ISSET(sock_fd, &fdsr))
				{//���������ļ���ʾ�������˸ı�ʱ��ʾ�ͻ�����������������
					new_fd = accept(sock_fd, (struct sockaddr *)&client_addr, &sin_size);
					if (new_fd <= 0)
					{
						perror("accept");
						continue;
					}

					// ��ӵ��ͻ��б���
					if (conn_amount < BACKLOG)
					{
						for (int i = 0; i < BACKLOG; i++)
						{
							if (clientList[i] == 0)
							{
								clientList[i] = new_fd;//������
								char *temp = inet_ntoa(client_addr.sin_addr);//ˢ�¿ͻ�����
								strcpy(clientName[i], temp);
								break;
							}
						}
						conn_amount++;
						printf("new connection client[%d] %s:%d\n", conn_amount,
							inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
						if (new_fd > maxsock)
							maxsock = new_fd;//���������ֵ����ľ��ֵ������maxsock��ֵ
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
			catch (const std::exception&)//�����쳣�ͷ�������Դ���˳�
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

	//��׽���µ�һ֡�¼��Ĵ�����
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
	{
		mutex_lock.lock();
		auto cloudTemp = new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud);//����һ����ʱ�洢ֵ�����ͷ�cloud��Դ
					
		//pcl::io::savePCDFile("Point Cloud Compression Example.pcd", *cloud);//��ʱ����Ҫ����

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
				readyTemp[i] = 1;//��ʱ���棬����ѹ��֮ǰ����0clientReady��־λ������ʱ��bug
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
			viewer.showCloud(cloud);//������һ֡
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
					//int ret = send(clientList[i], "FRAME", sizeof("FRAME"), 0);//��ʱȥ�����ֹ���
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
		(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind(&Server::cloud_cb_, this, _1);//ʹ��function��bind����һ���µĺ���Ϊע��ص�������׼��

		// connect callback function for desired signal. In this case its a point cloud with color values
		boost::signals2::connection c = grabber->registerCallback(f);//grabber���а���һ�������źŲ۵ķ�������f��Ϊ�ص�����ע���ȥ���Է��㴥���źŵ�ʱ�����ע�ᡣ

		// start receiving point clouds
		std::cout << "Start receiving point clouds" << std::endl;
		grabber->start();

		while (!viewer.wasStopped())//�����߳��ϵȴ�
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
		SOCKET sock_fd, new_fd;//������ӵ��׽���		
		sockaddr_in client_addr;
		int sin_size = sizeof(client_addr);

		int conn_amount;    // ��ǰ���ߵĿͻ��˵���Ŀ
		int clientList[BACKLOG] = { 0 };    // �ͻ��˵ľ���б�
		char clientName[BACKLOG][32];//���ߵĿͻ��˵�����
		int clientReady[BACKLOG] = { 0 };
		int frameCount;
		fd_set fdsr;//�½��ļ����
		int maxsock;//ָ�����������ļ��������ķ�Χ���������ļ������������ֵ��1
		struct timeval tv;
		std::mutex mutex_lock;
};

int
main()
{
	Server v;
	//std::thread newThread{ v.openServer() };//�����߳�����������
	v.run();
	
	return (0);
}
