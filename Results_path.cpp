#include<atlstr.h>
#include<iostream>
#include<fileapi.h>
#include<string> 
#include<list>
#include<vector>

using namespace std;
//代码说明：
//路径倒数第二个文件夹一定是本身就存在的，否则建不成文件，例如这次的test_2020_0X,必须要先建好
//可以随意更改下列布局

int main()
{
	//新建存放octree，random的结果的文件夹，因为每个实验对应四个BDSR的压缩方法，所以是每隔四个建一个文件夹
	for (int j = 1; j <= 1; j += 1) {
		CString src = "D:\MengjieXu\Science\BDSR2019-2020\test202009\OC_Ran_";
		//总共有9*30+18个file + 1 folder 产生
		char file1[10];
		sprintf_s(file1, "%d", j);
		src += file1;
		CreateDirectory(src, NULL);
		CString src1 = src + "\\pcd_file";  //装pcd文件
		CreateDirectory(src1, NULL);
		cout << 1 << endl;
	}


	//新建存放BDSR的文件夹，此文件夹是上述文件夹数量的四倍
	list<string> BDSRway;
	BDSRway.push_back("PCA");
	BDSRway.push_back("FA");
	BDSRway.push_back("KernelPCA");
	BDSRway.push_back("TruncatedSVD");
	typedef list<string> STRINGTLIST;
	STRINGTLIST::iterator i;
	for (i = BDSRway.begin(); i != BDSRway.end(); i++) {
		cout << *i << endl;
		CString str = "D:\MengjieXu\Science\BDSR2019-2020\test202009\BDSR\";
		char file[100];
		sprintf_s(file, "%s", *i);
		str += file;
		CreateDirectory(str, NULL);
		CString str1 = str + "\\cluster_pcd";   //装分割好的cluster的pcd文件
		CreateDirectory(str1, NULL);
		CString str2 = str + "\\cluster_txt";   //装分割好的cluster的txt文件
		CreateDirectory(str2, NULL);
		CString str3 = str + "\\code_output";   //装BDSR输出的结果文件
		CreateDirectory(str3, NULL);
		CString str4 = str + "\\down_sampled_data1";  //第一次降采样时的结果文件
		CreateDirectory(str4, NULL);
		CString str5 = str + "\\down_sampled_data2"; //第二次降采样时的结果文件
		CreateDirectory(str5, NULL);
		CString str6 = str + "\\txt_file";  //txt文件
		CreateDirectory(str6, NULL);
		CString str7 = str + "\\txt_file1";  //txt文件
		CreateDirectory(str7, NULL);
		CString str8 = str + "\\pcd_file";   //pcd文件
		CreateDirectory(str8, NULL);
		CString str9 = str + "\\nc_file";   //含有normal信息的文件
		CreateDirectory(str9, NULL);
		CString str10 = str + "\\vertex_txt_out";   //输出的OBB的顶点文件
		CreateDirectory(str10, NULL);
	}

	return 0;
}
