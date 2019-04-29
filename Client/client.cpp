#include "client.h"

int user_data = 0;

//void
//viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
//	viewer.setBackgroundColor(1.0, 0.5, 1.0);
//	pcl::PointXYZ o;
//	o.x = 1.0;
//	o.y = 0;
//	o.z = 0;
//	viewer.addSphere(o, 0.25, "sphere", 0);
//	std::cout << "i only run once" << std::endl;
//
//}
//
//void
//viewerPsycho(pcl::visualization::PCLVisualizer& viewer) {
//	static unsigned count = 0;
//	std::stringstream ss;
//	ss << "Once per viewer loop: " << count++;
//	viewer.removeShape("text", 0);
//	viewer.addText(ss.str(), 200, 300, "text", 0);
//
//	//FIXME: possible race condition here:
//	user_data++;
//}


int main(int argc, char* argv[])
{
	SOCKET sclient = _connect(argv[1]);
	
	char revData[1500];//char revData[1000000];//���ջ�����
	//std::string revData;
	int frameCount = 0;//��ǰ��¼�ı��ؽ��յ�֡���

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Octree compression"));
	pcl::visualization::CloudViewer viewer("showCloud");

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> octreeCompressionHigh(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
	
	while (!viewer.wasStopped())
	{
		//������һ֡
		int ret = send(sclient, "Require next Frame!",sizeof("Require next Frame!"),0);
		if (ret <= 0)
		{
			printf("send error!\n");
			return 0;
		}
		//��������  
		ret = recv(sclient,revData, 1500, 0);
		int totalCount = 0;//��֡����������
		if (ret > 0)
		{
			if (ret < 1500)
			{
				revData[ret] = 0x00;
				return 0;//����
			}
			frameCount++;
			//printf(revData);
			printf("%d\n", ret); 
			//��ʱȡ������
			/*if (strncmp("FRAME", revData, 5) == 0)
			{
				frameCount++;
				send(sclient, "RECIEVED", 254, 0);
			}*/
			//else//ѭ���ѻ����������ݶ���ȡ����
			{
				stringstream compressedData;
				while (true)
				{
					if (ret == 1500)
					{
						string *tmpStr = new string(revData, ret);
						compressedData << *tmpStr;
						totalCount += ret;
						ret = recv(sclient, revData, 1500, 0);
					}
					else
					{
						string *tmpStr = new string(revData, ret);
						compressedData << *tmpStr;
						totalCount += ret;
						break;
					}
				}
				//ofstream *fileSave = new ofstream("D:\\file.txt", std::ios_base::app);
				printf("frame count:%d\n", frameCount);
				//string str = compressedData.str();
				//printf("Ԥ�ƽ������ݣ�%d\nʵ�ʽ������ݣ�%d\n", totalCount, str.length());
				
				//octreeCompressionHigh.decodePointCloud(compressedData, decompressedCloud);
				
				//viewer.showCloud(decompressedCloud);
			}
		}

		else
		{
			printf("receive error!");
			return 0;
		}			

		//��������  
		/*const char * sendData = "��ã�TCP�ͻ��ˣ�\n";
		send(sClient, sendData, strlen(sendData), 0);
		closesocket(sClient);*/
		Sleep(1000);
	}

	closesocket(sclient);
	return 0;
}