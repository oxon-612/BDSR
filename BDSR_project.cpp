#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <vector>
#include <pcl/octree/octree_search.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Python.h>          //用来调用python代码
#include <pcl/surface/mls.h>
#include <atlstr.h>
#include <fileapi.h>
#include <string> 
#include <algorithm>     // 算法头文件，提供迭代器
#include <fstream>       //提供文件头文件
#include <stdlib.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
using namespace std;





//记录数据个数
int
numofPoints(const char *fname) {
	int n = 0;
	int c = 0;
	FILE *fp;
	fp = fopen(fname, "r");
	do {
		c = fgetc(fp);
		if (c == '\n') {
			++n;
		}
	} while (c != EOF);
	fclose(fp);
	return n;
}

//BDSR采样方法的函数
void BDSR(
	const char*originalfile,   //最原始点云pcd文件
	const char*txtfile,        //第一次OBB1得到的txt文件
	const char*txtfile0,       //第二次OBB得到的txt文件
	const char*txtfile1,       //第一次压缩得到的txt文件
	const char*txtfile2,       //第二次压缩得到的txt文件
	string BDSRway,            //BDSR方法
	// string folder,          //文件夹名称
	string cr1,                //一次压缩率
	string cr2,                //二次压缩率
	string path,               //BDSR结果路径
	double _r,                 //判断边缘点是否超过原始点云个数的_r倍
	double w,                  //圆柱添加假点的小体素是半径的w倍
	double e,                  //大体素的边长是小体素的e倍
	double c,                  //Cluster的最少点云数是边缘上采样到原始点云个数的点云的c倍
	double d,                  //Cluster的最多点云数是边缘上采样到原始点云个数的点云的d倍
	double curvature,          //curvature用来区分边缘和平面的阈值
	int K1,                    //原始点云用来计算normal的K值
	double f)                  //分cluster时的距离是resolution_size的f倍
{
	string WAY = BDSRway;
	// string FOLDER = folder;
	string PATH = path;
	string CR1 = cr1;
	string CR2 = cr2;
	

	//最原始点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read(originalfile, *cloud_original);
	//////////////////////////////////////////////////////////////////////////////得到原始点云文件
	////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////得到min_dimension////////////////////////////////////////////////////////////
	/*char filename1[] = "C:/Users/cvrl/Documents/PCD_projects/takeover/python/BDSR/OBB1.py"; 这里已经合并入BDSR.py
	//FILE* fp1;
	//Py_SetPythonHome(L"D:/MengjieXu/Software/anacondafile");
	Py_Initialize();
	fp1 = _Py_fopen(filename1, "r");
	PyRun_SimpleFile(fp1, filename1);
	Py_Finalize();*/
	///////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////txt_to_pcd
	int n = 0; //n用来计文件中点个数      
	FILE *fp_1;
	fp_1 = fopen(txtfile1, "r");
	n = numofPoints(txtfile1);   //使用numofPoints函数计算文件中点个数
	//新建一个点云文件，然后将结构中获取的xyz值传递到点云指针cloud中。  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = n;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	//将点云读入并赋给新建点云指针的xyz      
	double x, y, z;
	int i_ = 0;
	while (3 == fscanf(fp_1, "%lf %lf %lf \n", &x, &y, &z)) {
		cloud->points[i_].x = x;
		cloud->points[i_].y = y;
		cloud->points[i_].z = z;
		++i_;
	}
	fclose(fp_1);

	//将点云指针指向的内容传给pcd文件
	pcl::io::savePCDFileASCII(PATH + WAY + "/pcd_file/" + WAY + "_down_" + CR1 + "_PCD.pcd", *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points." << std::endl;
	///////////////////////////////////////////////////////////////////////////////////////得到cloud为pcdfile
	//创建法线估计对象，并将输入数据集传递给这个对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	//存储输出数据集
	pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);
	//创建一个空的kdtree对象，并把它传递给法线估计对象
	//基于给出的输入数据集，kdtree将被建立
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(K1);
	//计算特征值
	ne.compute(*pcNormal);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *pcNormal, *cloud_with_normals);
	cout << "normal_curvatrue done!" << endl;
	pcl::io::savePCDFile(PATH + WAY + "/nc_file/" + WAY + "_down_" + CR1 + "_NC.pcd", *cloud_with_normals);
	///////////////////////////////////////////////////////////////////////得到带有NC的ncfile：cloud_with_normals
	//读入原始点云，带有NC信息
	// 新建点云存储对象（存储曲率小于阈值部分的点云，也即非边缘点）
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_non_bounding(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_bounding1(new pcl::PointCloud<pcl::PointXYZ>);
	int k1 = 0;
	int count1 = 0;
	for (k1 = 0; k1 < cloud_with_normals->size(); k1++) {
		if (cloud_with_normals->points[k1].curvature < curvature)
			count1++;
	}
	cloud_non_bounding->width = count1;
	cloud_non_bounding->height = 1;
	cloud_non_bounding->is_dense = false;
	cloud_non_bounding->points.resize(cloud_non_bounding->width * cloud_non_bounding->height);
	cloud_non_bounding1->width = count1;
	cloud_non_bounding1->height = 1;
	cloud_non_bounding1->is_dense = false;
	cloud_non_bounding1->points.resize(cloud_non_bounding1->width * cloud_non_bounding1->height);
	//将curvature符合条件的输出到cloud_non_bounding中√
	size_t i = 0;
	size_t j = 0;
	for (; i < cloud_with_normals->size(); i++) {
		if (cloud_with_normals->points[i].curvature < curvature) {
			cloud_non_bounding->points[j].x = cloud_with_normals->points[i].x;
			cloud_non_bounding->points[j].y = cloud_with_normals->points[i].y;
			cloud_non_bounding->points[j].z = cloud_with_normals->points[i].z;
			cloud_non_bounding->points[j].curvature = cloud_with_normals->points[i].curvature;
			cloud_non_bounding->points[j].normal_x = cloud_with_normals->points[i].normal_x;
			cloud_non_bounding->points[j].normal_y = cloud_with_normals->points[i].normal_y;
			cloud_non_bounding->points[j].normal_z = cloud_with_normals->points[i].normal_z;
			cloud_non_bounding1->points[j].x = cloud_with_normals->points[i].x;
			cloud_non_bounding1->points[j].y = cloud_with_normals->points[i].y;
			cloud_non_bounding1->points[j].z = cloud_with_normals->points[i].z;
			j++;
		}
	}
	pcl::io::savePCDFileASCII(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_NC_non_bounding.pcd", *cloud_non_bounding);
	pcl::io::savePCDFile(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_NC_non_bounding1.pcd", *cloud_non_bounding1);
	///////////////////////////////////////////////////////////////////得到平面：cloud_non_bounding
	// 新建点云存储对象（存储曲率大于阈值部分的点云，也即边缘点）
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bounding1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_bounding(new pcl::PointCloud<pcl::PointNormal>);
	int k2 = 0;
	int count2 = 0;
	for (k2 = 0; k2 < cloud_with_normals->size(); k2++) {
		if (cloud_with_normals->points[k2].curvature > curvature)
			count2++;
	}
	int number = cloud_original->size();
	//判断是圆柱体还是非圆柱体
	//curvature都偏小，圆柱体
	if (count2 > _r*number) {
		cout << "This geometry is not a cylinder." << endl;
		//cout << count2 << endl;
		cloud_bounding->width = count2;
		cloud_bounding->height = 1;
		cloud_bounding->is_dense = false;
		cloud_bounding->points.resize(cloud_bounding->width * cloud_bounding->height);
		cloud_bounding1->width = count2;
		cloud_bounding1->height = 1;
		cloud_bounding1->is_dense = false;
		cloud_bounding1->points.resize(cloud_bounding1->width * cloud_bounding1->height);
		//将curvature符合条件的输出到cloud_bounding中√
		size_t m1 = 0;
		size_t n1 = 0;
		for (; m1 < cloud_with_normals->size(); m1++) {
			if (cloud_with_normals->points[m1].curvature > curvature) {
				cloud_bounding->points[n1].x = cloud_with_normals->points[m1].x;
				cloud_bounding->points[n1].y = cloud_with_normals->points[m1].y;
				cloud_bounding->points[n1].z = cloud_with_normals->points[m1].z;
				cloud_bounding->points[n1].curvature = cloud_with_normals->points[m1].curvature;
				cloud_bounding->points[n1].normal_x = cloud_with_normals->points[m1].normal_x;
				cloud_bounding->points[n1].normal_y = cloud_with_normals->points[m1].normal_y;
				cloud_bounding->points[n1].normal_z = cloud_with_normals->points[m1].normal_z;
				cloud_bounding1->points[n1].x = cloud_with_normals->points[m1].x;
				cloud_bounding1->points[n1].y = cloud_with_normals->points[m1].y;
				cloud_bounding1->points[n1].z = cloud_with_normals->points[m1].z;
				n1++;
			}
		}
		pcl::io::savePCDFileASCII(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_NC_bounding.pcd", *cloud_bounding);
		pcl::io::savePCDFile(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_NC_bounding1.pcd", *cloud_bounding1);
		///////////////////////////////////////////////////////////////////得到边缘：cloud_bounding
		//体素
		vector<double> V;
		vector<double>::iterator it;
		ifstream data(txtfile);
		double d0;
		while (data >> d0)
			V.push_back(d0);//将数据压入堆栈。//
		data.close();
		float min_dimension1 = (*V.begin());
		cout << "min_dimension1 is: " << min_dimension1 << endl;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(min_dimension1);
		octree.setInputCloud(cloud_original);
		octree.addPointsFromInputCloud();
		pcl::PointXYZ searchPoint1;
		//输出点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
		int count = 0;
		for (size_t j = 0; j < cloud_original->size(); ++j) {
			for (size_t i = 0; i < cloud_bounding->size(); ++i) {
				if (cloud_bounding->points[i].curvature > 0.01&&
					cloud_original->points[j].x == cloud_bounding->points[i].x&&
					cloud_original->points[j].y == cloud_bounding->points[i].y&&
					cloud_original->points[j].z == cloud_bounding->points[i].z) {
					//cout << "i: " << i << "  j: " << j << endl;
					//search voxel keep add
					searchPoint1.x = cloud_original->points[j].x;
					searchPoint1.y = cloud_original->points[j].y;
					searchPoint1.z = cloud_original->points[j].z;
					vector<int> pointIdxVec;
					if (octree.voxelSearch(searchPoint1, pointIdxVec))
					{
						size_t k = 0;
						for (; k < pointIdxVec.size(); ++k) {
							count++;
						}
					}
				}
			}
		}
		cloud_result->width = count;
		cloud_result->height = 1;
		cloud_result->is_dense = false;
		cloud_result->points.resize(cloud_result->width * cloud_result->height);
		pcl::PointXYZ searchPoint;
		//将curvature符合条件的在原始点云中找到
		size_t t = 0;
		for (size_t p = 0; p < cloud_original->size(); ++p) {
			for (size_t q = 0; q < cloud_bounding->size(); ++q) {
				if (cloud_bounding->points[q].curvature > 0.01&&
					cloud_original->points[p].x == cloud_bounding->points[q].x&&
					cloud_original->points[p].y == cloud_bounding->points[q].y&&
					cloud_original->points[p].z == cloud_bounding->points[q].z) {
					//search voxel keep add
					searchPoint.x = cloud_original->points[p].x;
					searchPoint.y = cloud_original->points[p].y;
					searchPoint.z = cloud_original->points[p].z;
					vector<int> pointIdxVec;
					if (octree.voxelSearch(searchPoint, pointIdxVec))
					{
						size_t h = 0;
						for (; h < pointIdxVec.size(); ++h) {
							cloud_result->points[t].x = cloud_original->points[pointIdxVec[h]].x;
							cloud_result->points[t].y = cloud_original->points[pointIdxVec[h]].y;
							cloud_result->points[t].z = cloud_original->points[pointIdxVec[h]].z;
							t++;
						}
					}
				}
			}
		}
		//上采样点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_up(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud_up = *cloud_bounding1 + *cloud_result;
		//剔除重复点
		for (size_t a = 0; a < cloud_up->size(); a++) {
			for (size_t b = 0; b < cloud_up->size(); b++) {
				if (a != b &&
					cloud_up->points[a].x == cloud_up->points[b].x&&
					cloud_up->points[a].y == cloud_up->points[b].y&&
					cloud_up->points[a].z == cloud_up->points[b].z)
				{
					cloud_up->erase(cloud_up->begin() + b);
				}
			}
		}
		pcl::io::savePCDFile(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_NC_bounding_upsampling_result1.pcd", *cloud_up);
		///////////////////////////////////////////////////////////////////上采样原始点云个数：cloud_up
		// 建立kd-tree对象用来搜索 .
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdtree->setInputCloud(cloud_up);
		// Euclidean 聚类对象.
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
		// 设置聚类的最小值 2cm (small values may cause objects to be divided
		// in several clusters, whereas big values may join objects in a same cluster).
		double ClusterTolerance = min_dimension1 * f;
		clustering.setClusterTolerance(ClusterTolerance);
		// 设置聚类的小点数和最大点云数
		int MinClusterSize = c * cloud_up->size();
		int MaxClusterSize = d * cloud_up->size();
		clustering.setMinClusterSize(MinClusterSize);
		clustering.setMaxClusterSize(MaxClusterSize);
		clustering.setSearchMethod(kdtree);
		clustering.setInputCloud(cloud_up);
		std::vector<pcl::PointIndices> clusters;
		clustering.extract(clusters);
		int currentClusterNum = 1;
		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
			//添加所有的点云到一个新的点云中
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				cluster->points.push_back(cloud_up->points[*point]);
			cluster->width = cluster->points.size();
			cluster->height = 1;
			cluster->is_dense = true;

			// 保存
			if (cluster->points.size() <= 0)
				break;
			std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
			std::string fileName = "cluster" + boost::to_string(currentClusterNum);
			pcl::io::savePCDFileASCII(PATH + WAY + "/cluster_pcd/" + fileName + ".pcd", *cluster);
			currentClusterNum++;
			//pcd转化成txt，不会写函数的输出格式，暂时直接使用代码跑
			int Num = cluster->points.size();
			double *X = new double[Num] {0};
			double *Y = new double[Num] {0};
			double *Z = new double[Num] {0};
			for (size_t i = 0; i < cluster->points.size(); ++i)
			{
				X[i] = cluster->points[i].x;
				Y[i] = cluster->points[i].y;
				Z[i] = cluster->points[i].z;
			}
			//颜色在cloudcompare里打开时用RGBAf
			ofstream zos(PATH + WAY + "/cluster_txt/" + fileName + ".txt");
			for (int i = 0; i < Num; i++)
			{
				zos << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
			}
			cout << "trans has done!!!" << endl;
			cin.get();
		}
		/////////////////////////////////////////////////////////得到clusters的txt文件，此时可以运行OBB
		char filename[] = "C:/Users/cvrl/Documents/PCD_projects/takeover/python/BDSR/OBB_cluster.py";
		FILE* fp;
		Py_SetPythonHome(L"C:/Users/cvrl/AppData/Local/Continuum/anaconda3");
		Py_Initialize();
		fp = _Py_fopen(filename, "r");
		PyRun_SimpleFile(fp, filename);
		//Py_Finalize();
		//////////////////////////////////////////////////////////得到min_dimension
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add1(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud_add1 = *cloud_up + *cloud_non_bounding1;
		pcl::io::savePCDFile(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_result1.pcd", *cloud_add1);
		////////////////////////////////////////////////////////得到第一次合并cloud_up和未处理的cloud_non_bounding之后的cloud_add1
		//体素
		//动态创建二维数组[cloud_add1->size()][4]
		int mm = cloud_add1->size(), nn = 4;
		int** array;
		array = new int*[mm];//这里是mm
		for (int i = 0; i < mm; i++) {
			array[i] = new int[nn];//这里是nn
		}
		vector<double> V1;
		vector<double>::iterator it1;
		ifstream data1(txtfile0);
		double d1;
		while (data1 >> d1)
			V1.push_back(d1);//将数据压入堆栈。//
		data1.close();
		float min_dimension = (*V1.begin());
		cout << min_dimension << endl;
		float resolution1 = min_dimension;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree1(resolution1);
		octree1.setInputCloud(cloud_add1);
		octree1.addPointsFromInputCloud();
		float resolution2 = resolution1 * e;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree2(resolution2);
		octree2.setInputCloud(cloud_add1);
		octree2.addPointsFromInputCloud();
		pcl::PointXYZ searchPoint3;
		//寻找每个点的体素中的点个数count
		for (size_t i = 0; i < cloud_add1->size(); ++i) {
			//search
			searchPoint3.x = cloud_add1->points[i].x;
			searchPoint3.y = cloud_add1->points[i].y;
			searchPoint3.z = cloud_add1->points[i].z;
			vector<int> pointIdxVec;
			if (octree1.voxelSearch(searchPoint3, pointIdxVec))
			{
				size_t k = 0;
				int count_small = 0;
				for (; k < pointIdxVec.size(); ++k)
				{
					count_small++;
				}
				//输出计数和density
				/*cout << "i = " << i << " : count_small = " << count_small << " "
					<< "x = " << cloud_add1->points[i].x << " "
					<< "y = " << cloud_add1->points[i].y << " "
					<< "z = " << cloud_add1->points[i].z << " "
					<< " density1 = " << count_small / (resolution1*resolution1*resolution1) << " "
					<< endl;*/
				array[i][0] = count_small / (resolution1*resolution1*resolution1);
				array[i][1] = count_small;
			}
			if (octree2.voxelSearch(searchPoint3, pointIdxVec))
			{
				size_t k = 0;
				int count_big = 0;
				for (; k < pointIdxVec.size(); ++k) {
					count_big++;
				}
				//输出计数和density
				/*cout << "i = " << i << " : count_big = " << count_big << " "
					<< "x = " << cloud_add1->points[i].x << " "
					<< "y = " << cloud_add1->points[i].y << " "
					<< "z = " << cloud_add1->points[i].z << " "
					<< " density2 = " << count_big / (resolution2*resolution2*resolution2) << " "
					<< endl;*/
				array[i][2] = count_big / (resolution2*resolution2*resolution2);
				array[i][3] = count_big;
			}
		}
		//现在的判断方法只判断sparse
		int sparse = 0;
		int voidp = 0;
		int sparse_1 = 0;
		for (size_t j = 0; j < cloud_add1->size(); j++) {
			if (array[j][2] > array[j][0]) {
				//cout << "j_sparse : " << j << endl;
				sparse++;
				if ((0.1*array[j][3] / 8 - array[j][1]) >= 1) {
					sparse_1++;
					//cout << "j_sparse_1 : " << j << endl;
				}
			}
		}
		cout << "sparse: " << sparse << endl;
		cout << "sparse_1: " << sparse_1 << endl;
		//计算平均每个ok大体素内的点的个数
		int sum = 0;
		for (size_t h = 0; h < cloud_add1->size(); h++) {
			if (array[h][2] < array[h][0]) {
				sum = array[h][3] + sum;
			}
		}
		int ave = (sum / ((cloud_add1->size()) - sparse));
		cout << "The sum is " << sum << " "
			<< "The average of big is " << ave << endl;
		//计算平均所有的ok小体素内的点的个数
		int sum1 = 0;
		for (size_t h1 = 0; h1 < cloud_add1->size(); h1++) {
			if (array[h1][2] < array[h1][0]) {
				sum1 = array[h1][1] + sum1;
			}
		}
		int ave1 = (sum1 / (cloud_add1->size() - sparse));
		cout << "The sum is " << sum1 << " "
			<< "The average of small is " << ave1 << endl;
		//对sparse点和void点进行上采样
		//将sparse点和void点输出到cloud_part中√
		//新建点云存储对象（存储sparse点和void点的点云）
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZ>);
		int m = 0;
		int count_sparse = 0;
		for (m = 0; m < cloud_add1->size(); m++) {
			if (array[m][2] > array[m][0])
				count_sparse++;
		}
		cout << count_sparse << endl;
		cloud_part->width = count_sparse;
		cloud_part->height = 1;
		cloud_part->is_dense = false;
		cloud_part->points.resize(cloud_part->width * cloud_part->height);
		int u, s = 0;
		for (; u < cloud_add1->size(); u++) {
			if (array[u][2] > array[u][0]) {
				cloud_part->points[s].x = cloud_add1->points[u].x;
				cloud_part->points[s].y = cloud_add1->points[u].y;
				cloud_part->points[s].z = cloud_add1->points[u].z;
				s++;
			}
		}
		delete[] array;
		pcl::io::savePCDFileASCII(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_result1_sparse.pcd", *cloud_part);
		/////////////////////////////////////////////////////////////////得到稀疏点云文件
		pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_bounding1(new pcl::PointCloud<pcl::PointXYZ>);
		int __count = 0;
		for (size_t j = 0; j < cloud_up->size(); ++j) {
			for (size_t i = 0; i < cloud_part->size(); ++i) {
				if (cloud_up->points[j].x == cloud_part->points[i].x&&
					cloud_up->points[j].y == cloud_part->points[i].y&&
					cloud_up->points[j].z == cloud_part->points[i].z) {
					size_t k = 0;
					__count++;
				}
			}
		}
		_cloud_bounding1->width = __count;
		_cloud_bounding1->height = 1;
		_cloud_bounding1->is_dense = false;
		_cloud_bounding1->points.resize(_cloud_bounding1->width * _cloud_bounding1->height);
		//将稀疏点云文件的边缘点的在原始点云的边缘点中找到
		size_t __t = 0;
		for (size_t p = 0; p < cloud_up->size(); ++p) {
			for (size_t q = 0; q < cloud_part->size(); ++q) {
				if (cloud_up->points[p].x == cloud_part->points[q].x&&
					cloud_up->points[p].y == cloud_part->points[q].y&&
					cloud_up->points[p].z == cloud_part->points[q].z) {
					_cloud_bounding1->points[__t].x = cloud_part->points[q].x;
					_cloud_bounding1->points[__t].y = cloud_part->points[q].y;
					_cloud_bounding1->points[__t].z = cloud_part->points[q].z;
					//颜色是原本的颜色，没有特殊标出
					__t++;
				}
			}
		}
		//剔除重复点
		for (size_t a = 0; a < _cloud_bounding1->size(); a++) {
			for (size_t b = 0; b < _cloud_bounding1->size(); b++) {
				if (a != b &&
					_cloud_bounding1->points[a].x == _cloud_bounding1->points[b].x&&
					_cloud_bounding1->points[a].y == _cloud_bounding1->points[b].y&&
					_cloud_bounding1->points[a].z == _cloud_bounding1->points[b].z)
				{
					_cloud_bounding1->erase(_cloud_bounding1->begin() + b);
				}
			}
		}
		pcl::io::savePCDFile(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_sparse_NC_bounding1.pcd", *_cloud_bounding1);

		////////////////////////////////////////////////////////////////////////////////得到sparse的边缘：_cloud_bounding1

		/// 滤波对象
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
		filter.setInputCloud(_cloud_bounding1);
		//建立搜索对象
		pcl::search::Octree<pcl::PointXYZ>::Ptr octree3;
		filter.setSearchMethod(octree3);
		filter.setComputeNormals(true);
		//设置搜索邻域的半径为3cm
		double searchradius = resolution1;
		filter.setSearchRadius(searchradius);
		// Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
		filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
		//RANDOM_UNIFORM_DENSITY
		int pointdensity = ave1;
		filter.setPointDensity(pointdensity);
		filter.process(*filteredCloud);
		//与第一次上采样至原始点云的文件合并
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_up2(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud_up2 = *filteredCloud + *cloud_up;
		//剔除重复点
		for (size_t a = 0; a < cloud_up2->size(); a++) {
			for (size_t b = 0; b < cloud_up2->size(); b++) {
				if (a != b &&
					cloud_up2->points[a].x == cloud_up2->points[b].x&&
					cloud_up2->points[a].y == cloud_up2->points[b].y&&
					cloud_up2->points[a].z == cloud_up2->points[b].z)
				{
					cloud_up2->erase(cloud_up2->begin() + b);
					cout << cloud_up2->size() << endl;
				}
			}
		}
		std::vector<int> mapping1;
		pcl::removeNaNFromPointCloud(*cloud_up2, *cloud_up2, mapping1);
		pcl::io::savePCDFileASCII(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_NC_bounding_upsampling_result2.pcd", *cloud_up2);
		///////////////////////////////////////////////////////////////////得到添加假点后的边缘点云文件cloud_up2
		int Num_ = cloud_up2->points.size();
		double *X = new double[Num_] {0};
		double *Y = new double[Num_] {0};
		double *Z = new double[Num_] {0};
		for (size_t i = 0; i < cloud_up2->points.size(); ++i)
		{
			X[i] = cloud_up2->points[i].x;
			Y[i] = cloud_up2->points[i].y;
			Z[i] = cloud_up2->points[i].z;
		}
		ofstream zos(PATH + WAY + "/txt_file1/" + WAY + "_down_" + CR1 + "_NC_bounding_upsampling_result2.txt");
		for (int i = 0; i < Num_; i++)
		{
			zos << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		}
		cout << "trans is done!!!" << endl;
		cin.get();
		/////////////////////////////////////////////////////////////////得到边缘添加假点的txt文件，下面可以进行BDS.py
		char pyfile[] = "C:/Users/cvrl/Documents/PCD_projects/takeover/python/BDSR/BDS_2nd_downsampling.py";
		FILE* fp_2;
		Py_SetPythonHome(L"C:/Users/cvrl/AppData/Local/Continuum/anaconda3");
		//Py_Initialize();
		fp_2 = _Py_fopen(pyfile, "r");
		PyRun_SimpleFile(fp_2, pyfile);
		Py_Finalize();
		////////////////////////////////////////////////////////////////////得到添加假点的边缘点二次压缩的txt文件
		int _n = 0; //n用来计文件中点个数      
		FILE *fp_3;
		fp_3 = fopen(txtfile2, "r");
		n = numofPoints(txtfile2);   //使用numofPoints函数计算文件中点个数  
		//std::cout << "there are " << n << " points in the file..." << std::endl;
		//新建一个点云文件，然后将结构中获取的xyz值传递到点云指针cloud_bounding3中。  
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bounding3(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_bounding3->width = n;
		cloud_bounding3->height = 1;
		cloud_bounding3->is_dense = false;
		cloud_bounding3->points.resize(cloud_bounding3->width * cloud_bounding3->height);
		//将点云读入并赋给新建点云指针的xyz      
		double _x, _y, _z;
		// double _r, _g, _b;
		int _i = 0;
		while (3 == fscanf(fp_3, "%lf %lf %lf\n", &_x, &_y, &_z)) {
			cloud_bounding3->points[_i].x = _x;
			cloud_bounding3->points[_i].y = _y;
			cloud_bounding3->points[_i].z = _z;
			++_i;
		}
		fclose(fp_3);
		//将点云指针指向的内容传给pcd文件  
		pcl::io::savePCDFileASCII(PATH + WAY + "/pcd_file/" + WAY + "_down_" + CR1 + "_" + CR2 + "_bounding.pcd", *cloud_bounding3);
		std::cerr << "Saved " << cloud_bounding3->points.size() << " data points." << std::endl;
		/////////////////////////////////////////////////////////////////////////////得到边缘二次压缩后的pcd文件
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudresult(new pcl::PointCloud<pcl::PointXYZ>);
		*cloudresult = *cloud_bounding3 + *cloud_non_bounding1;
		pcl::io::savePCDFileASCII(PATH + WAY + "/pcd_file/" + WAY + "_down_" + CR1 + "_" + CR2 + "_PCD.pcd", *cloudresult);
		////////////////////////////////////////////////////////////////////////////得到最终的pcd文件
		int _Num = cloudresult->points.size();
		double *_X = new double[_Num] {0};
		double *_Y = new double[_Num] {0};
		double *_Z = new double[_Num] {0};
		for (size_t i = 0; i < cloudresult->points.size(); ++i)
		{
			_X[i] = cloudresult->points[i].x;
			_Y[i] = cloudresult->points[i].y;
			_Z[i] = cloudresult->points[i].z;
		}
		ofstream _zos(PATH + WAY + "/txt_file1/" + WAY + "_down_" + CR1 + "_" + CR2 + "_PCD.txt");
		for (int i = 0; i < _Num; i++)
		{
			_zos << _X[i] << " " << _Y[i] << " " << _Z[i] << " " << endl;
		}
		cout << "trans has done!!!" << endl;
		cin.get();
	}
	else {
		//圆柱体
		cout << "This geometry is a cylinder." << endl;
		//体素
		typedef pcl::PointXYZ PointT;
		// All the objects needed
		pcl::PCDReader reader;
		pcl::PassThrough<PointT> pass;
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
		pcl::PCDWriter writer;
		pcl::ExtractIndices<PointT> extract;
		pcl::ExtractIndices<pcl::Normal> extract_normals;
		pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

		// Datasets
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
		pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

		// Read in the cloud data
		reader.read(originalfile, *cloud);
		std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

		// Build a passthrough filter to remove spurious NaNs
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0, 4);
		pass.filter(*cloud_filtered);
		std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

		// Estimate point normals
		ne.setSearchMethod(tree);
		ne.setInputCloud(cloud_filtered);
		ne.setKSearch(10);
		ne.compute(*cloud_normals);

		// Create the segmentation object for the planar model and set all the parameters
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		seg.setNormalDistanceWeight(0.1);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.03);
		seg.setInputCloud(cloud_filtered);
		seg.setInputNormals(cloud_normals);
		// Obtain the plane inliers and coefficients
		seg.segment(*inliers_plane, *coefficients_plane);
		std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

		// Extract the planar inliers from the input cloud
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers_plane);
		extract.setNegative(false);

		// Write the planar inliers to disk
		pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
		extract.filter(*cloud_plane);
		std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
		writer.write(PATH + WAY + "/pcd_file/18_plane.pcd", *cloud_plane, false);

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_filtered2);
		extract_normals.setNegative(true);
		extract_normals.setInputCloud(cloud_normals);
		extract_normals.setIndices(inliers_plane);
		extract_normals.filter(*cloud_normals2);

		// Create the segmentation object for cylinder segmentation and set all the parameters
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight(0.1);
		seg.setMaxIterations(1000);
		seg.setDistanceThreshold(1.5);
		seg.setRadiusLimits(0, 1.5);
		seg.setInputCloud(cloud_filtered2);
		seg.setInputNormals(cloud_normals2);

		// Obtain the cylinder inliers and coefficients
		seg.segment(*inliers_cylinder, *coefficients_cylinder);
		std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

		// Write the cylinder inliers to disk
		extract.setInputCloud(cloud_filtered2);
		extract.setIndices(inliers_cylinder);
		extract.setNegative(false);
		pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
		extract.filter(*cloud_cylinder);
		if (cloud_cylinder->points.empty())
			std::cerr << "Can't find the cylindrical component." << std::endl;
		else
		{
			std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
			writer.write(PATH + WAY + "/pcd_file/18_cylinder.pcd", *cloud_cylinder, false);
		}
		float sum_D = 0.0;
		float sum_Ave = 0.0;
		float x0 = coefficients_cylinder->values[0];
		float y0 = coefficients_cylinder->values[1];
		float z0 = coefficients_cylinder->values[2];
		float l = coefficients_cylinder->values[3];
		float m = coefficients_cylinder->values[4];
		float n = coefficients_cylinder->values[5];
		float r0 = coefficients_cylinder->values[6];
		//得到半径
		float min_dimension1 = coefficients_cylinder->values[6];
		//动态创建二维数组[cloud_add1->size()][4]
		int mm = cloud->size(), nn = 4;
		int** array;
		array = new int*[mm];//这里是mm
		for (int i = 0; i < mm; i++) {
			array[i] = new int[nn];//这里是nn
		}
		float resolution1 = min_dimension1 * w;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree1(resolution1);
		octree1.setInputCloud(cloud);
		octree1.addPointsFromInputCloud();
		float resolution2 = resolution1 * e;
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree2(resolution2);
		octree2.setInputCloud(cloud);
		octree2.addPointsFromInputCloud();
		pcl::PointXYZ searchPoint3;
		//寻找每个点的体素中的点个数count
		for (size_t i = 0; i < cloud->size(); ++i) {
			//search
			searchPoint3.x = cloud->points[i].x;
			searchPoint3.y = cloud->points[i].y;
			searchPoint3.z = cloud->points[i].z;
			vector<int> pointIdxVec;
			if (octree1.voxelSearch(searchPoint3, pointIdxVec))
			{
				size_t k = 0;
				int count_small = 0;
				for (; k < pointIdxVec.size(); ++k)
				{
					count_small++;
				}
				array[i][0] = count_small / (resolution1*resolution1*resolution1);
				array[i][1] = count_small;
			}
			if (octree2.voxelSearch(searchPoint3, pointIdxVec))
			{
				size_t k = 0;
				int count_big = 0;
				for (; k < pointIdxVec.size(); ++k) {
					count_big++;
				}
				array[i][2] = count_big / (resolution2*resolution2*resolution2);
				array[i][3] = count_big;
			}
		}
		//现在的判断方法只判断sparse
		int sparse = 0;
		int voidp = 0;
		int sparse_1 = 0;
		for (size_t j = 0; j < cloud->size(); j++) {
			if (array[j][2] > array[j][0]) {
				//cout << "j_sparse : " << j << endl;
				sparse++;
				if ((0.1*array[j][3] / 8 - array[j][1]) >= 1) {
					sparse_1++;
					//cout << "j_sparse_1 : " << j << endl;
				}
			}
		}
		cout << "sparse: " << sparse << endl;
		cout << "sparse_1: " << sparse_1 << endl;
		//计算平均每个ok大体素内的点的个数
		int sum = 0;
		for (size_t h = 0; h < cloud->size(); h++) {
			if (array[h][2] < array[h][0]) {
				sum = array[h][3] + sum;
			}
		}
		int ave = (sum / ((cloud->size()) - sparse));
		cout << "The sum is " << sum << " "
			<< "The average of big is " << ave << endl;
		//计算平均所有的ok小体素内的点的个数
		int sum1 = 0;
		for (size_t h1 = 0; h1 < cloud->size(); h1++) {
			if (array[h1][2] < array[h1][0]) {
				sum1 = array[h1][1] + sum1;
			}
		}
		int ave1 = (sum1 / (cloud->size() - sparse));
		cout << "The sum is " << sum1 << " "
			<< "The average of small is " << ave1 << endl;
		//对sparse点和void点进行上采样
		//将sparse点和void点输出到cloud_part中√
		//新建点云存储对象（存储sparse点和void点的点云）
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_part(new pcl::PointCloud<pcl::PointXYZ>);
		int m1 = 0;
		int count_sparse = 0;
		for (m1 = 0; m1 < cloud->size(); m1++) {
			if (array[m1][2] > array[m1][0])
				count_sparse++;
		}
		cout << count_sparse << endl;
		cloud_part->width = count_sparse;
		cloud_part->height = 1;
		cloud_part->is_dense = false;
		cloud_part->points.resize(cloud_part->width * cloud_part->height);
		int u, s = 0;
		for (; u < cloud->size(); u++) {
			if (array[u][2] > array[u][0]) {
				cloud_part->points[s].x = cloud->points[u].x;
				cloud_part->points[s].y = cloud->points[u].y;
				cloud_part->points[s].z = cloud->points[u].z;
				s++;
			}
		}
		delete[] array;
		pcl::io::savePCDFileASCII(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_result1_sparse.pcd", *cloud_part);
		/////////////////////////////////////////////////////////////////得到稀疏点云文件
		/// 滤波对象
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
		pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
		filter.setInputCloud(cloud_part);
		//建立搜索对象
		pcl::search::Octree<pcl::PointXYZ>::Ptr octree3;
		filter.setSearchMethod(octree3);
		filter.setComputeNormals(true);
		//设置搜索邻域的半径为3cm
		double searchradius = resolution1;
		filter.setSearchRadius(searchradius);
		// Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION
		filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
		//RANDOM_UNIFORM_DENSITY
		int pointdensity = ave1;
		filter.setPointDensity(pointdensity);
		filter.process(*filteredCloud);
		//与最初输入的文件合并
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_up2(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud_up2 = *filteredCloud + *cloud;
		//剔除重复点
		for (size_t a = 0; a < cloud_up2->size(); a++) {
			for (size_t b = 0; b < cloud_up2->size(); b++) {
				if (a != b &&
					cloud_up2->points[a].x == cloud_up2->points[b].x&&
					cloud_up2->points[a].y == cloud_up2->points[b].y&&
					cloud_up2->points[a].z == cloud_up2->points[b].z)
				{
					cloud_up2->erase(cloud_up2->begin() + b);
					cout << cloud_up2->size() << endl;
				}
			}
		}
		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*cloud_up2, *cloud_up2, mapping);
		//pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
		//save it back
		pcl::io::savePCDFileASCII(PATH + WAY + "/code_output/" + WAY + "_down_" + CR1 + "_NC_bounding_upsampling_result2.pcd", *cloud_up2);
		///////////////////////////////////////////////////////////////////得到添加假点后的点云文件cloud_up2
		int Num_ = cloud_up2->points.size();
		double *X = new double[Num_] {0};
		double *Y = new double[Num_] {0};
		double *Z = new double[Num_] {0};
		for (size_t i = 0; i < cloud_up2->points.size(); ++i)
		{
			X[i] = cloud_up2->points[i].x;
			Y[i] = cloud_up2->points[i].y;
			Z[i] = cloud_up2->points[i].z;
		}
		ofstream zos(PATH + WAY + "/txt_file1/" + WAY + "_down_" + CR1 + "_NC_bounding_upsampling_result2.txt");
		for (int i = 0; i < Num_; i++)
		{
			zos << X[i] << " " << Y[i] << " " << Z[i] << " " << endl;
		}
		cout << "trans has done!!!" << endl;
		cin.get();
	}
}


int
main()
{
	BDSR(
		"C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/pcd_file/pier_test.pcd",                  //最原始点云pcd文件
		"C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file1/min_dimension1.txt",            //OBB1_orig_data
		"C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file1/min_dimension.txt",             //OBB_cluster
		"C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/down_sampled_data1/FA_down_0.05_PCD.txt", //第一次压缩得到的txt文件
		"C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/down_sampled_data2/FA_down_0.3_PCD.txt",  //第二次压缩得到的txt文件
		"FA",      //BDSR中的方法
		// "4",    //文件夹名称
		"0.05",    //一压压缩率
		"0.3",     //二压压缩率
		"C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/", //BDSR结果路径
		0.001,     //判断边缘点云个数是原始点云的_r倍
		0.005,     //圆柱的小体素边长是半径的w倍
		1.2,       //大体素的边长是小体素的e倍
		0.1,       //Cluster的最少点云数是边缘上采样到原始点云个数的点云的c倍
		0.999,     //Cluster的最多点云数是边缘上采样到原始点云个数的点云的d倍                                                                                                                                                                                                 ；；；；
		0.005,     //curvature用来区分边缘和平面的阈值
		10,        //原始点云用来计算normal的K值
		0.08);     //分cluster时的距离是resolution_size的f倍, 其中resolution_size是原始点云OBB的最小dimension
	return 0;
}