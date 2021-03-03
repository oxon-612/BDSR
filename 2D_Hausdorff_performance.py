from scipy.spatial.distance import directed_hausdorff
import codecs
import numpy as np
import os
import re

# =========================================
#
#        Original data Concave Hull
#
# =========================================
orig_data_path = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file/" # 原始数据的路径
name = "pier_test.txt" # 原始数据
# =========================================
#            Calculate Hausdorff
# =========================================
f = codecs.open(orig_data_path + name, mode='r', encoding='utf-8')  # 打开txt文件，以‘utf-8’编码读取,这里读取的是利用get_concave_vertex得到的vertex点
line = f.readline()  # 以行的形式进行读取文件
list1 = []
while line:
    a = line.split()
    b = a[0:2]  # 这是选取需要读取的位数 前两列 X,Y
    list1.append(b)  # 将其添加在列表之中
    line = f.readline()
f.close()
x = [float(i[0]) for i in list1]
y = [float(i[1]) for i in list1]
list_orig = np.array(list1)
file_name = 'Orig_2D_Hausdorff_Results.txt'
f1 = open(orig_data_path + file_name, 'a')
f1.write("way 2DHausdorff\n")
f1.close()

# =========================================
#
#         Random data Hausdorff
#
# =========================================
# 读取所有Random开头的文件
random_files = []
data_dir = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/"
for i in os.listdir(data_dir):
    if os.path.isfile(os.path.join(data_dir,i)) and 'Random' in i:
        random_files.append(i)
seed_count = 0
file_name = 'rand_2D_Hausdorff_Results.txt'
f = open(data_dir + file_name, 'a')
for random_file in random_files:
    print(random_file)
    # =========================================
    #            Calculate Hausdorff
    # =========================================
    f = codecs.open('C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/' + str(random_file[0:]),
                    mode='r', encoding='utf-8')  # 打开txt文件，以‘utf-8’编码读取,这里读取的是利用get_concave_vertex得到的vertex点
    line = f.readline()  # 以行的形式进行读取文件
    list1 = []
    while line:
        a = line.split()
        b = a[0:2]  # 这是选取需要读取的位数 前两列 X,Y
        list1.append(b)  # 将其添加在列表之中
        line = f.readline()
    f.close()
    x = [float(i[0]) for i in list1]
    y = [float(i[1]) for i in list1]
    list_random = np.array(list1)
    h_dist = max(directed_hausdorff(list_orig, list_random)[0], directed_hausdorff(list_random, list_orig)[0])

    seed_value_list = [int(x) for x in re.findall('\d+', random_file)]
    seed_value = seed_value_list[2]

    # =========================================
    #     Write hausdorff results into file
    # =========================================
    file_line = ''
    seed_count += 1
    print(seed_count)
    file_line += ("{} {}\n".format(seed_value, h_dist))
    file_name = 'rand_2D_Hausdorff_Results.txt'
    f = open(data_dir + file_name, 'a')
    f.write(file_line)
    f.close()

# =========================================
#
#        Octree and BDSR data Hausdoff
#
# =========================================
cr1 = "0.05" # 一压压缩率
dataset = name.replace(".txt","")

WAY_list = ["FA", "PCA", "KernelPCA", "TruncatedSVD", "octreeDownsampling", "octreeResampling"]
for way in WAY_list:
    if way == "FA":
        way_folder = "FA/txt_file1"
        data_name = way + "_down_" + cr1 + "_0.3_PCD.txt"
    elif way == "PCA":
        way_folder = "PCA/txt_file1"
        data_name = way + "_down_" + cr1 + "_0.3_PCD.txt"
    elif way == "KernelPCA":
        way_folder = "KernelPCA/txt_file1"
        data_name = way + "_down_" + cr1 + "_0.3_PCD.txt"
    elif way == "TruncatedSVD":
        way_folder = "TruncatedSVD/txt_file1"
        data_name = way + "_down_" + cr1 + "_0.3_PCD.txt"
    elif way == "octreeDownsampling":
        way_folder = "Oct_Rand_Sample_1"
        data_name = dataset + "_" + way + ".txt"
    else:
        way_folder = "Oct_Rand_Sample_1"
        data_name = dataset + "_" + way + ".txt"

    data_dir = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/" + way_folder + "/"

    # =========================================
    #            Calculate Hausdorff
    # =========================================
    f = codecs.open(data_dir + data_name, mode='r', encoding='utf-8')  # 打开txt文件，以‘utf-8’编码读取,这里读取的是利用get_concave_vertex得到的vertex点
    line = f.readline()  # 以行的形式进行读取文件
    list1 = []
    while line:
        a = line.split()
        b = a[0:2]  # 这是选取需要读取的位数 前两列 X,Y
        list1.append(b)  # 将其添加在列表之中
        line = f.readline()
    f.close()
    x = [float(i[0]) for i in list1]
    y = [float(i[1]) for i in list1]
    list_not_random = np.array(list1)
    h_dist = max(directed_hausdorff(list_orig, list_not_random)[0], directed_hausdorff(list_not_random, list_orig)[0])
    # =========================================
    #      Write Hausdorff results into file
    # =========================================
    file_name = '{}_2D_Hausdorff_Results.txt'.format(way)
    f = open(data_dir + file_name, 'a')
    file_line = ""
    file_line += ("{} {}\n".format(way, h_dist))
    f.write(file_line)
    f.close()

# =================================================
#
#    Concatenate files and generate final result
#
# =================================================

orig_hausd_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file/Orig_2D_Hausdorff_Results.txt"
FA_hausd_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file1/FA_2D_Hausdorff_Results.txt"
PCA_hausd_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/PCA/txt_file1/PCA_2D_Hausdorff_Results.txt"
KernelPCA_hausd_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/KernelPCA/txt_file1/KernelPCA_2D_Hausdorff_Results.txt"
TruncatedSVD_hausd_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/TruncatedSVD/txt_file1/TruncatedSVD_2D_Hausdorff_Results.txt"
octDown_hausd_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/octreeDownsampling_2D_Hausdorff_Results.txt"
octRe_hausd_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/octreeResampling_2D_Hausdorff_Results.txt"
random_hausd_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/rand_2D_Hausdorff_Results.txt"

hausd_list = [orig_hausd_file, FA_hausd_file, PCA_hausd_file, KernelPCA_hausd_file, TruncatedSVD_hausd_file, octDown_hausd_file, octRe_hausd_file, random_hausd_file]

with open('C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Performance/2D_Hausdorff_Results.txt', 'w') as outfile:
    for hausd in hausd_list:
        with open(hausd) as infile:
            outfile.write(infile.read())
    outfile.close()



