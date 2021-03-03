#计算2D 文件的entropy（在这里应该是计算得到的 2D 的entropy）
import numpy as np
from scipy.stats import entropy
from math import log, e
import pandas as pd
import timeit
import codecs
import csv
import os
import re

def entropy1(labels, base=None):
  value,counts = np.unique(labels, return_counts=True)
  return entropy(counts, base=base)
'''def entropy2(labels, base=None):
  """ Computes entropy of label distribution. """
  n_labels = len(labels)
  if n_labels <= 1:
    return 0
  value,counts = np.unique(labels, return_counts=True)
  probs = counts / n_labels
  n_classes = np.count_nonzero(probs)
  if n_classes <= 1:
    return 0
  ent = 0.
  # Compute entropy
  base = e if base is None else base
  for i in probs:
    ent -= i * log(i, base)
  return ent

def entropy3(labels, base=None):
  vc = pd.Series(labels).value_counts(normalize=True, sort=False)
  base = e if base is None else base
  return -(vc * np.log(vc)/np.log(base)).sum()

def entropy4(labels, base=None):
  value,counts = np.unique(labels, return_counts=True)
  norm_counts = counts / counts.sum()
  base = e if base is None else base
  return -(norm_counts * np.log(norm_counts)/np.log(base)).sum()

#labels = [1,3,5,2,3,5,3,2,1,3,4,5]
#print("label: ",labels)
'''

# =========================================
#
#        Original data Concave Hull
#
# =========================================
orig_data_path = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file/" # 原始数据的路径
name = "pier_test.txt" # 原始数据
# =========================================
#            Calculate entropy
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
list_orig = np.array(list1).astype(float)
file_name = 'Orig_2D_Entropy_Results.txt'
f1 = open(orig_data_path + file_name, 'a')
f1.write("way 2DEntropy\n")
file_line = ''
file_line += ("{} {}\n".format(name.replace(".txt"," "), entropy1(list_orig)))
f1.write(file_line)
f1.close()

# =========================================
#
#        Random data 2D Entropy
#
# =========================================
# 读取所有Random开头的文件
random_files = []
data_dir = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/"
for i in os.listdir(data_dir):
    if os.path.isfile(os.path.join(data_dir,i)) and 'Random' in i:
        random_files.append(i)
seed_count = 0
file_name = 'rand_2D_Entropy_Results.txt'
f = open(data_dir + file_name, 'a')
for random_file in random_files:
    print(random_file)
    # =========================================
    #            Calculate 2D entropy
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
    list_random = np.array(list1).astype(float)

    seed_value_list = [int(x) for x in re.findall('\d+', random_file)]
    seed_value = seed_value_list[2]

    # =========================================
    #     Write entropy results into file
    # =========================================
    concave_hull_area_list = []
    file_line = ''

    seed_count += 1
    print(seed_count)
    file_line += ("{} {}\n".format(seed_value, entropy1(list_random)))
    file_name = 'rand_2D_Entropy_Results.txt'
    f = open(data_dir + file_name, 'a')
    f.write(file_line)
    f.close()

# =========================================
#
#        Octree and BDSR data 2D entropy
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
    #            Calculate area
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
    list_not_random = np.array(list1).astype(float)

    # =========================================
    #      Write entropy results into file
    # =========================================
    file_name = '{}_2D_Entropy_Results.txt'.format(way)
    f = open(data_dir + file_name, 'a')
    file_line = ""
    file_line += ("{} {}\n".format(way, entropy1(list_not_random)))
    f.write(file_line)
    f.close()


# =================================================
#
#    Concatenate files and generate final result
#
# =================================================

orig_entropy_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file/Orig_2D_Entropy_Results.txt"
FA_entropy_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file1/FA_2D_Entropy_Results.txt"
PCA_entropy_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/PCA/txt_file1/PCA_2D_Entropy_Results.txt"
KernelPCA_entropy_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/KernelPCA/txt_file1/KernelPCA_2D_Entropy_Results.txt"
TruncatedSVD_entropy_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/TruncatedSVD/txt_file1/TruncatedSVD_2D_Entropy_Results.txt"
octDown_entropy_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/octreeDownsampling_2D_Entropy_Results.txt"
octRe_entropy_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/octreeResampling_2D_Entropy_Results.txt"
random_entropy_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/rand_2D_Entropy_Results.txt"

entropy_list = [orig_entropy_file, FA_entropy_file, PCA_entropy_file, KernelPCA_entropy_file, TruncatedSVD_entropy_file, octDown_entropy_file, octRe_entropy_file, random_entropy_file]

with open('C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Performance/2D_Entropy_Results.txt', 'w') as outfile:
    for entropy in entropy_list:
        with open(entropy) as infile:
            outfile.write(infile.read())
    outfile.close()

