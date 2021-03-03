/*
���ܣ�����octree���е����ز���
*/

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

int main()
{
	// ����ԭʼ����
	pcl::PointCloud<pcl::PointXYZ> sourceCloud;
	if (pcl::io::loadPCDFile("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/pcd_file/pier_test.pcd", sourceCloud) == -1)
		return -1;

	pcl::PointCloud<pcl::PointXYZ> resultCloud1;
	pcl::io::loadPCDFile("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/TruncatedSVD/pcd_file/TruncatedSVD_down_0.05_0.3_PCD.pcd", resultCloud1);
	int number_result1 = resultCloud1.size();
	pcl::PointCloud<pcl::PointXYZ> resultCloud2;
	pcl::io::loadPCDFile("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/KernelPCA/pcd_file/KernelPCA_down_0.05_0.3_PCD.pcd", resultCloud2);
	int number_result2 = resultCloud2.size();
	pcl::PointCloud<pcl::PointXYZ> resultCloud3;
	pcl::io::loadPCDFile("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/PCA/pcd_file/PCA_down_0.05_0.3_PCD.pcd", resultCloud3);
	int number_result3 = resultCloud3.size();
	pcl::PointCloud<pcl::PointXYZ> resultCloud4;
	pcl::io::loadPCDFile("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/pcd_file/FA_down_0.05_0.3_PCD.pcd", resultCloud4);
	int number_result4 = resultCloud4.size();
	double number_result = (number_result1 + number_result2 + number_result3 + number_result4) / 4;
	cout << number_result1 << endl;
	cout << number_result2 << endl;
	cout << number_result3 << endl;
	cout << number_result4 << endl;
	cout << number_result << endl;

	// �Ƿ�鿴ѹ����Ϣ
	bool showStatistics = true;
	// �����ļ�������뿴�����ļ�����ϸ���ݣ����Բο�: /io/include/pcl/compression/compression_profiles.h
	pcl::io::compression_Profiles_e compressionProfile = pcl::io::MANUAL_CONFIGURATION;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

	double resolution = 1.0;
	for (int i = 0;
		i < 1000; i++) {
		if (cloudOut->size() < number_result - 10) {
			resolution = resolution - 0.001;
			// ��ʼ������ѹ�����ͽ�ѹ��
			pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudEncoder;
			PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile, showStatistics);
			PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile,
				false,
				0.01,
				resolution,  //ֻ��Ҫ����һ�����������Ҳ��С���صı߳���Խ��õ��ĵ���Խ��
				true,
				50,
				true,
				4);


			// ѹ�����stringstream
			std::stringstream compressedData;
			// �������
			// ѹ������
			PointCloudEncoder->encodePointCloud(sourceCloud.makeShared(), compressedData);
			//std::cout << compressedData.str() << std::endl;
			// ��ѹ����
			PointCloudEncoder->decodePointCloud(compressedData, cloudOut);
			std::cout << cloudOut->size() << endl;
			cout << resolution << endl;
			pcl::io::savePCDFile("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/pcd_file/pier_test_octreeResampling.pcd", *cloudOut);
		}
		else if (cloudOut->size() > number_result + 10) {
			resolution = resolution + 0.001;
			// ��ʼ������ѹ�����ͽ�ѹ��
			pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudEncoder;
			PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile, showStatistics);
			PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile,
				false,
				0.01,
				resolution,  //ֻ��Ҫ����һ�����������Ҳ��С���صı߳���Խ��õ��ĵ���Խ��
				true,
				50,
				true,
				4);


			// ѹ�����stringstream
			std::stringstream compressedData;
			// �������
			// ѹ������
			PointCloudEncoder->encodePointCloud(sourceCloud.makeShared(), compressedData);
			//std::cout << compressedData.str() << std::endl;
			// ��ѹ����
			PointCloudEncoder->decodePointCloud(compressedData, cloudOut);
			std::cout << cloudOut->size() << endl;
			cout << resolution << endl;
			pcl::io::savePCDFile("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/pcd_file/pier_test_octreeResampling.pcd", *cloudOut);
		}
	}

	// ��.pcdת����.txt
	int _Num = cloudOut->points.size();
	double *_X = new double[_Num] {0};
	double *_Y = new double[_Num] {0};
	double *_Z = new double[_Num] {0};
	// double *_R = new double[_Num] {0};
	// double *_G = new double[_Num] {0};
	// double *_B = new double[_Num] {0};
	for (size_t i = 0; i < cloudOut->points.size(); ++i)
	{
		_X[i] = cloudOut->points[i].x;
		_Y[i] = cloudOut->points[i].y;
		_Z[i] = cloudOut->points[i].z;
		// _R[i] = cloudresult->points[i].r;
		// _G[i] = cloudresult->points[i].g;
		// _B[i] = cloudresult->points[i].b;
	}
	ofstream _zos("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/pier_test_octreeResampling.txt");
	for (int i = 0; i < _Num; i++)
	{
		_zos << _X[i] << " " << _Y[i] << " " << _Z[i] << " " << endl;
	}
	cout << "trans has done!!!" << endl;
	cin.get();

	return 0;
}