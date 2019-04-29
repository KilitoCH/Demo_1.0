#include "server.h"

int main(int argc, char* argv[])
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
	
	SOCKET sock_fd,new_fd;//最近连接的套接字
	int fd_A[BACKLOG] = { 0 };    // 客户端的句柄列表
	int conn_amount;    // 当前在线的客户端的数目

	

	if (listen(slisten, BACKLOG + 1) == -1)//实际可监听的比最大连接数参数大1，为了避免一些bug
	{
		printf("listen error!\n");
	}

	sockaddr_in client_addr;
	int sin_size = sizeof(client_addr);

	char fd_A_name[BACKLOG][32];//在线的客户端的名字
	int frameCount = 0;
	fd_set fdsr;//新建文件句柄
	int maxsock;//指集合中所有文件描述符的范围，即所有文件描述符的最大值加1
	struct timeval tv;

	conn_amount = 0;

	maxsock = slisten;

	sock_fd = slisten;//将服务器也存入列表

	//初始化现有的两帧
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);//frame 1 buffer
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);//frame 2 buffer

	pcl::io::loadPLYFile("D:\\graduateProj\\assets\\sayu-no-texture.ply", *cloud_1);
	pcl::io::loadPCDFile("D:\\graduateProj\\assets\\rabbit.pcd", *cloud_2);//将现有的两帧读入buffer
	pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompressionHigh(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
	std::stringstream compressedData_1, compressedData_2;

	octreeCompressionHigh.encodePointCloud(cloud_1, compressedData_1);//cloud需不需要进行释放
	octreeCompressionHigh.encodePointCloud(cloud_2, compressedData_2);//cloud需不需要进行释放

	while (true) {
		try
		{
			// 初始化文件句柄集合，因为每次select都会更改这个集合
			FD_ZERO(&fdsr);
			FD_SET(sock_fd, &fdsr);//把socket文件描述符添加到文件描述符列表中以监测他的变化

			// 设定select的超时时间，因为每次select都会清除这个值，所以设在循环内
			tv.tv_sec = 30;//秒
			tv.tv_usec = 0;//毫秒

			char revData[255];//从主机收到的数据
			// 检测仍在线的客机，并把他们添加到文件句柄集合
			for (int i = 0; i < BACKLOG; i++)
			{
				if (fd_A[i] != 0)
				{
					FD_SET(fd_A[i], &fdsr);
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
			// 检测每一个客户端是否发送了新的信息
			for (int i = 0; i < conn_amount; i++)
			{
				if (FD_ISSET(fd_A[i], &fdsr))
				{//当某个客机的文件标示符发生改变时表示新发送了信息
					ret = recv(fd_A[i], revData, sizeof(revData), 0);
					printf("%s\n", revData);
					if (ret <= 0)
					{        // 客户端关闭处理
						printf("client[%d] close\n", i);
						close(fd_A[i]);
						FD_CLR(fd_A[i], &fdsr);
						fd_A[i] = 0;
						memset(fd_A_name[i], '\0', 1);
						conn_amount--;
					}
					else
					{
						std::string sendData;
						if (frameCount % 2 == 0)
							sendData = compressedData_1.str();
						else
							sendData = compressedData_1.str();
												
						send(fd_A[i], "FRAME", sizeof("FRAME"), 0);
						recv(fd_A[i], revData, 254, 0);//会不会超出buffer大小之后出现循环写入的情况-->不会，问题在于buffer里面的数据没有读完 ！！！recv函数有独立的buffer
						if (strncmp(revData, "RECIEVED", sizeof("RECIEVED")) == 0)
							cout << "开始发送第" << ++frameCount << "帧" << endl;

						revData[0] = '\0';
						//send()用来将数据由指定的socket传给对方主机
						//int send(int s, const void * msg, int len, unsigned int flags)
						//s为已建立好连接的socket，msg指向数据内容，len则为数据长度，参数flags一般设0
						//成功则返回实际传送出去的字符数，失败返回-1，错误原因存于error 
						send(fd_A[i], sendData.c_str(), sendData.length(), 0);


						ret = recv(fd_A[i], revData, 254, 0);
						if (ret > 0) {
							revData[ret] = 0x00;
							printf("%s\n", revData);
						}
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
						if (fd_A[i] == 0)
						{
							fd_A[i] = new_fd;//完成添加
							char *temp = inet_ntoa(client_addr.sin_addr);//刷新客机名字
							strcpy(fd_A_name[i], temp);
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
		}
		catch (const std::exception&)
		{
			for (int i = 0; i < BACKLOG; i++)
			{
				if (fd_A[i] != 0) {
					closesocket(fd_A[i]);
				}
			}
			return 0;
		}
	}

	for (int i = 0; i < BACKLOG; i++)
	{
		if (fd_A[i] != 0) {
			closesocket(fd_A[i]);
		}
	}
	WSACleanup();
	return 0;

}
