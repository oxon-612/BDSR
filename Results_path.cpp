#include<atlstr.h>
#include<iostream>
#include<fileapi.h>
#include<string> 
#include<list>
#include<vector>

using namespace std;
//����˵����
//·�������ڶ����ļ���һ���Ǳ���ʹ��ڵģ����򽨲����ļ���������ε�test_2020_0X,����Ҫ�Ƚ���
//��������������в���

int main()
{
	//�½����octree��random�Ľ�����ļ��У���Ϊÿ��ʵ���Ӧ�ĸ�BDSR��ѹ��������������ÿ���ĸ���һ���ļ���
	for (int j = 1; j <= 1; j += 1) {
		CString src = "D:\MengjieXu\Science\BDSR2019-2020\test202009\OC_Ran_";
		//�ܹ���9*30+18��file + 1 folder ����
		char file1[10];
		sprintf_s(file1, "%d", j);
		src += file1;
		CreateDirectory(src, NULL);
		CString src1 = src + "\\pcd_file";  //װpcd�ļ�
		CreateDirectory(src1, NULL);
		cout << 1 << endl;
	}


	//�½����BDSR���ļ��У����ļ����������ļ����������ı�
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
		CString str1 = str + "\\cluster_pcd";   //װ�ָ�õ�cluster��pcd�ļ�
		CreateDirectory(str1, NULL);
		CString str2 = str + "\\cluster_txt";   //װ�ָ�õ�cluster��txt�ļ�
		CreateDirectory(str2, NULL);
		CString str3 = str + "\\code_output";   //װBDSR����Ľ���ļ�
		CreateDirectory(str3, NULL);
		CString str4 = str + "\\down_sampled_data1";  //��һ�ν�����ʱ�Ľ���ļ�
		CreateDirectory(str4, NULL);
		CString str5 = str + "\\down_sampled_data2"; //�ڶ��ν�����ʱ�Ľ���ļ�
		CreateDirectory(str5, NULL);
		CString str6 = str + "\\txt_file";  //txt�ļ�
		CreateDirectory(str6, NULL);
		CString str7 = str + "\\txt_file1";  //txt�ļ�
		CreateDirectory(str7, NULL);
		CString str8 = str + "\\pcd_file";   //pcd�ļ�
		CreateDirectory(str8, NULL);
		CString str9 = str + "\\nc_file";   //����normal��Ϣ���ļ�
		CreateDirectory(str9, NULL);
		CString str10 = str + "\\vertex_txt_out";   //�����OBB�Ķ����ļ�
		CreateDirectory(str10, NULL);
	}

	return 0;
}
