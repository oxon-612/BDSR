#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <vector>
#include <ctime>
//利用octree_down的方法进行降采样


//octree_down函数
pcl::PointCloud<pcl::PointXYZ>
octree_down(const char * filename)
{
	//加载结果点云
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
	//cout << number_result1 << endl;
	//cout << number_result2<< endl;
	//cout << number_result3 << endl;
	//cout << number_result4 << endl;
	//cout << number_result << endl;

	pcl::PointCloud<pcl::PointXYZ>  cloudresult2;
	double resolution = 1.0;
	for (int i = 0; i < 1000; i++) {
		if (cloudresult2.size() < number_result - 10) {
			resolution = resolution - 0.001;
			//resolution_size就是小体素边长，filename是要进行octree降采样的文件
			typedef pcl::PointXYZ PointT;
			typedef std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;   ///分配内存对齐Eigen::MatrixXf;*/

			///loading datas.
			pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
			pcl::PCDReader reader;
			reader.read(filename, *cloud);
			//std::cerr << "loading " << cloud->points.size() << " data from file." << std::endl;
			///searching octree voxel center.
			AlignedPointTVector voxel_center_list_arg;
			voxel_center_list_arg.clear();      ///clear memory
			pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
			octree.setInputCloud(cloud);
			octree.addPointsFromInputCloud();   ////** \brief  Add points from input point cloud to octree. 4重载 */
			octree.getOccupiedVoxelCenters(voxel_center_list_arg);

			//输出octree降采样的点云文件
			cloudresult2.width = voxel_center_list_arg.size();
			cloudresult2.height = 1;
			cloudresult2.is_dense = false;
			cloudresult2.points.resize(cloudresult2.width * cloudresult2.height);
			//找中心最近的点
			pcl::PointXYZ searchPoint;
			int m;
			size_t j = 0;
			for (m = 0; m < voxel_center_list_arg.size(); m++) {
				searchPoint.x = voxel_center_list_arg[m].x;
				searchPoint.y = voxel_center_list_arg[m].y;
				searchPoint.z = voxel_center_list_arg[m].z;
				int K = 1;
				std::vector<int> pointIdxNKNSearch;
				std::vector<float> pointNKNSquaredDistance;
				if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
				{
					size_t i = 0;
					for (; i < pointIdxNKNSearch.size(); i++) {
						cloudresult2.points[j].x = cloud->points[pointIdxNKNSearch[i]].x;
						cloudresult2.points[j].y = cloud->points[pointIdxNKNSearch[i]].y;
						cloudresult2.points[j].z = cloud->points[pointIdxNKNSearch[i]].z;
						j++;
					}
				}
			}

		}
		else if (cloudresult2.size() > number_result + 10) {
			resolution = resolution + 0.001;
			//resolution_size就是小体素边长，filename是要进行octree降采样的文件
			typedef pcl::PointXYZ PointT;
			typedef std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;   ///分配内存对齐Eigen::MatrixXf;*/


			///loading datas.
			pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
			pcl::PCDReader reader;
			reader.read(filename, *cloud);
			//std::cerr << "loading " << cloud->points.size() << " data from file." << std::endl;
			///searching octree voxel center.
			AlignedPointTVector voxel_center_list_arg;
			voxel_center_list_arg.clear();      ///clear memory
			//float resolution = atof(argv[2]);   ///ascii to floating point numbers
			pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
			octree.setInputCloud(cloud);
			octree.addPointsFromInputCloud();   ////** \brief  Add points from input point cloud to octree. 4重载 */
			octree.getOccupiedVoxelCenters(voxel_center_list_arg);

			//输出octree降采样的点云文件
			cloudresult2.width = voxel_center_list_arg.size();
			cloudresult2.height = 1;
			cloudresult2.is_dense = false;
			cloudresult2.points.resize(cloudresult2.width * cloudresult2.height);
			//找中心最近的点
			pcl::PointXYZ searchPoint;
			int m;
			size_t j = 0;
			for (m = 0; m < voxel_center_list_arg.size(); m++) {
				searchPoint.x = voxel_center_list_arg[m].x;
				searchPoint.y = voxel_center_list_arg[m].y;
				searchPoint.z = voxel_center_list_arg[m].z;
				int K = 1;
				std::vector<int> pointIdxNKNSearch;
				std::vector<float> pointNKNSquaredDistance;
				if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
				{
					size_t i = 0;
					for (; i < pointIdxNKNSearch.size(); i++) {
						cloudresult2.points[j].x = cloud->points[pointIdxNKNSearch[i]].x;
						cloudresult2.points[j].y = cloud->points[pointIdxNKNSearch[i]].y;
						cloudresult2.points[j].z = cloud->points[pointIdxNKNSearch[i]].z;

						j++;
					}
				}
			}

		}

	}
	return cloudresult2;
}

int main(int argc, char** argv)
{
	//"E:/XMJ/3Drebuilding/paper/test/test_2019_10/test39/pcd_file/18.pcd"
	//0.028f是指octree的小体素的边长，这个数值越大，则降采样得到的点数越少，压缩性越强，见函数内部的解释
	pcl::PointCloud<pcl::PointXYZ> cloudresult2 = octree_down("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/pcd_file/pier_test.pcd"); //原始点云
	
	// Octree_Down pcd 结果
	pcl::io::savePCDFileASCII("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/pcd_file/pier_test_octreeDownsampling.pcd", cloudresult2);
	std::cout << cloudresult2.size() << std::endl;

	// 将.pcd转化成.txt
	int _Num = cloudresult2.points.size();
	double *_X = new double[_Num] {0};
	double *_Y = new double[_Num] {0};
	double *_Z = new double[_Num] {0};
	// double *_R = new double[_Num] {0};
	// double *_G = new double[_Num] {0};
	// double *_B = new double[_Num] {0};
	for (size_t i = 0; i < cloudresult2.points.size(); ++i)
	{
		_X[i] = cloudresult2.points[i].x;
		_Y[i] = cloudresult2.points[i].y;
		_Z[i] = cloudresult2.points[i].z;
		// _R[i] = cloudresult->points[i].r;
		// _G[i] = cloudresult->points[i].g;
		// _B[i] = cloudresult->points[i].b;
	}
	std::ofstream _zos("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/pier_test_octreeDownsampling.txt");
	for (int i = 0; i < _Num; i++)
	{
		_zos << _X[i] << " " << _Y[i] << " " << _Z[i] << " " << std::endl;
	}
	std::cout << "trans has done!!!" << std::endl;
	std::cin.get();


	return 0;
}
