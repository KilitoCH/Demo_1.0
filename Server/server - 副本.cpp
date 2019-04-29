#include "server.h"

int main(int argc, char* argv[])
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
	
	SOCKET sock_fd,new_fd;//������ӵ��׽���
	int fd_A[BACKLOG] = { 0 };    // �ͻ��˵ľ���б�
	int conn_amount;    // ��ǰ���ߵĿͻ��˵���Ŀ

	

	if (listen(slisten, BACKLOG + 1) == -1)//ʵ�ʿɼ����ı����������������1��Ϊ�˱���һЩbug
	{
		printf("listen error!\n");
	}

	sockaddr_in client_addr;
	int sin_size = sizeof(client_addr);

	char fd_A_name[BACKLOG][32];//���ߵĿͻ��˵�����
	int frameCount = 0;
	fd_set fdsr;//�½��ļ����
	int maxsock;//ָ�����������ļ��������ķ�Χ���������ļ������������ֵ��1
	struct timeval tv;

	conn_amount = 0;

	maxsock = slisten;

	sock_fd = slisten;//��������Ҳ�����б�

	//��ʼ�����е���֡
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);//frame 1 buffer
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);//frame 2 buffer

	pcl::io::loadPLYFile("D:\\graduateProj\\assets\\sayu-no-texture.ply", *cloud_1);
	pcl::io::loadPCDFile("D:\\graduateProj\\assets\\rabbit.pcd", *cloud_2);//�����е���֡����buffer
	pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompressionHigh(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
	std::stringstream compressedData_1, compressedData_2;

	octreeCompressionHigh.encodePointCloud(cloud_1, compressedData_1);//cloud�費��Ҫ�����ͷ�
	octreeCompressionHigh.encodePointCloud(cloud_2, compressedData_2);//cloud�費��Ҫ�����ͷ�

	while (true) {
		try
		{
			// ��ʼ���ļ�������ϣ���Ϊÿ��select��������������
			FD_ZERO(&fdsr);
			FD_SET(sock_fd, &fdsr);//��socket�ļ���������ӵ��ļ��������б����Լ�����ı仯

			// �趨select�ĳ�ʱʱ�䣬��Ϊÿ��select����������ֵ����������ѭ����
			tv.tv_sec = 30;//��
			tv.tv_usec = 0;//����

			char revData[255];//�������յ�������
			// ��������ߵĿͻ�������������ӵ��ļ��������
			for (int i = 0; i < BACKLOG; i++)
			{
				if (fd_A[i] != 0)
				{
					FD_SET(fd_A[i], &fdsr);
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
			// ���ÿһ���ͻ����Ƿ������µ���Ϣ
			for (int i = 0; i < conn_amount; i++)
			{
				if (FD_ISSET(fd_A[i], &fdsr))
				{//��ĳ���ͻ����ļ���ʾ�������ı�ʱ��ʾ�·�������Ϣ
					ret = recv(fd_A[i], revData, sizeof(revData), 0);
					printf("%s\n", revData);
					if (ret <= 0)
					{        // �ͻ��˹رմ���
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
						recv(fd_A[i], revData, 254, 0);//�᲻�ᳬ��buffer��С֮�����ѭ��д������-->���ᣬ��������buffer���������û�ж��� ������recv�����ж�����buffer
						if (strncmp(revData, "RECIEVED", sizeof("RECIEVED")) == 0)
							cout << "��ʼ���͵�" << ++frameCount << "֡" << endl;

						revData[0] = '\0';
						//send()������������ָ����socket�����Է�����
						//int send(int s, const void * msg, int len, unsigned int flags)
						//sΪ�ѽ��������ӵ�socket��msgָ���������ݣ�len��Ϊ���ݳ��ȣ�����flagsһ����0
						//�ɹ��򷵻�ʵ�ʴ��ͳ�ȥ���ַ�����ʧ�ܷ���-1������ԭ�����error 
						send(fd_A[i], sendData.c_str(), sendData.length(), 0);


						ret = recv(fd_A[i], revData, 254, 0);
						if (ret > 0) {
							revData[ret] = 0x00;
							printf("%s\n", revData);
						}
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
						if (fd_A[i] == 0)
						{
							fd_A[i] = new_fd;//������
							char *temp = inet_ntoa(client_addr.sin_addr);//ˢ�¿ͻ�����
							strcpy(fd_A_name[i], temp);
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
