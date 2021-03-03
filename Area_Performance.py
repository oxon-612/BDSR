################################
#  author:mengjie xu
#  计算random的2D concave 的 area和hausdorff
################################
# import csv
import numpy as np
import matplotlib.pyplot as plt
from descartes import PolygonPatch
import shapely.geometry
# from shapely.geometry import Polygon
import pylab as pl
import math
from shapely.ops import cascaded_union, polygonize
from scipy.spatial import Delaunay
import codecs
# from scipy.spatial.distance import directed_hausdorff
import os
import re

# =========================================
#
#        Concave hull or Alpha Shape
#
# =========================================
def get_area(points):
    point_num = len(points)
    # print(points)
    if point_num < 3:
        return 0.0
    else:
        s = 0
        for i in range(len(points)):
            s += points[i, 0] * points[(i + 1) % point_num, 1] - (points[i, 1] * points[(i + 1) % point_num, 0])
        # plt.subplot(1, 2, 1)
        # plt.plot(points[:, 0], points[:, 1], 'o', color='#f16824')
        # plt.subplot(1, 2, 2)
        # plt.plot(points[:, 0], points[:, 1], 'r-', linewidth=2)
        # plt.show()
    return abs(s / 2)
def get_2D_concave(data_dir,data_file,myalpha):
    # =========================================
    #            get Vertex
    # =========================================
    # input是3D点云，也可以改动代码根据txt的读取读取2D点云
    data = np.loadtxt(data_dir + data_file)

    # 根据所选取的横截面进行平面的选取
    proj_plane = 0
    if proj_plane == 0:  # planeXY = 0
        points_plane = points_planeXY = np.loadtxt(data_dir + data_file, usecols=(0, 1))
    elif proj_plane == 1:  # planeYZ = 1
        points_plane = points_planeYZ = np.loadtxt(data_dir + data_file, usecols=(1, 2))
    else:  # planeXZ
        points_plane = points_planeXZ = np.loadtxt(data_dir + data_file, usecols=(0, 2))
    x = points_plane[:, 0]
    y = points_plane[:, 1]
    point_collection = shapely.geometry.MultiPoint(list(points_plane))

    def alpha_shape(_points, _alpha):
        '''
        Compute the alpha shape (concave hull) of a set
        of points.
        @param points: Iterable container of points.
        @param alpha: alpha value to influence the
            gooeyness of the border. Smaller numbers
            don't fall inward as much as larger numbers.
            Too large, and you lose everything!
        '''
        if len(_points) < 4:
            # When you have a triangle, there is no sense
            # in computing an alpha shape.
            return point_collection.convex_hull

        def add_edge(_edges, _edge_points, _coords, _i, _j):
            '''
            Add a line between the i-th and j-th points,
            if not in the list already
            '''
            if (_i, _j) in _edges or (_j, _i) in _edges:
                # already added
                return
            _edges.add((_i, _j))
            _edge_points.append(_coords[[_i, _j]])

        #
        _coords = np.array([_point for _point in _points])
        _tri = Delaunay(_coords)
        _edges = set()
        _edge_points = []
        # loop over triangles:
        # ia, ib, ic = indices of corner points of the triangle
        for _ia, _ib, _ic in _tri.vertices:
            _pa = _coords[_ia]
            _pb = _coords[_ib]
            _pc = _coords[_ic]
            # Lengths of sides of triangle
            _a = math.sqrt((_pa[0] - _pb[0]) ** 2 + (_pa[1] - _pb[1]) ** 2)
            _b = math.sqrt((_pb[0] - _pc[0]) ** 2 + (_pb[1] - _pc[1]) ** 2)
            _c = math.sqrt((_pc[0] - _pa[0]) ** 2 + (_pc[1] - _pa[1]) ** 2)
            # Semiperimeter of triangle
            _s = (_a + _b + _c) / 2.0
            # Area of triangle by Heron's formula
            if _s * (_s - _a) * (_s - _b) * (_s - _c) >= 0:
                _area = math.sqrt(_s * (_s - _a) * (_s - _b) * (_s - _c))
                if _area != 0:
                    # print(_area)
                    _circum_r = _a * _b * _c / (4.0 * _area)
                    # Here's the radius filter.
                    if _circum_r < 1.0 / _alpha:
                        add_edge(_edges, _edge_points, _coords, _ia, _ib)
                        add_edge(_edges, _edge_points, _coords, _ib, _ic)
                        add_edge(_edges, _edge_points, _coords, _ic, _ia)
                    _m = shapely.geometry.MultiLineString(_edge_points)
                    _triangles = list(polygonize(_m))
        return cascaded_union(_triangles), _edge_points

    # 越大越凹
    concave_hull_poly, edge_points = alpha_shape(_points=points_plane, _alpha=myalpha)
    x_concave_poly, y_concave_poly = concave_hull_poly.exterior.coords.xy
    # save to files
    concave_vertex_line = ''
    concave_count = 0
    for i in range(0, len(x_concave_poly) - 1):
        concave_count += 1
        concave_vertex_line += ("{} {}\n".format(x_concave_poly[i], y_concave_poly[i]))
    #
    # 创建concave hulls的folder
    concave_hull_file_name = data_file.replace('.txt', '_ch.txt')

    ch_dir = data_dir + 'concave_hull_results/'
    if not os.path.exists(ch_dir):
        os.makedirs(ch_dir)


    f_vertex = open(ch_dir + concave_hull_file_name, 'w')
    # f_vertex.write("//concave hull polygon vertex\n" + str(concave_count) + "\n")
    f_vertex.write(concave_vertex_line)
    f_vertex.close()
    #
    concave_hull_data = np.loadtxt(ch_dir + concave_hull_file_name)
    for i in range(0, len(concave_hull_data)):
        plt.annotate(i + 1, (concave_hull_data[i][0], concave_hull_data[i][1]))

    return concave_hull_file_name
def polygon_point(dataset):
    proj_plane = 0
    if proj_plane == 0:  # planeXY = 0
        points_plane = points_planeXY = np.loadtxt(dataset, skiprows=0, usecols=(0, 1))
    elif proj_plane == 1:  # planeYZ = 1
        points_plane = points_planeYZ = np.loadtxt(dataset, skiprows=0, usecols=(1, 2))
    else:  # planeXZ
        points_plane = points_planeXZ = np.loadtxt(dataset, skiprows=0, usecols=(0, 2))
    x = points_plane[:, 0]
    y = points_plane[:, 1]
    #print((shapely.geometry.Polygon(points_plane)).area)
    #point_collection = MultiPoint(list(points_plane)).convex_hull
    point_collection = shapely.geometry.Polygon(points_plane)
    return point_collection


# =========================================
#
#        Original data Concave Hull
#
# =========================================
orig_data_path = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file/" # 原始数据的路径
name = "pier_test.txt" # 原始数据
concave_hull_file_name = get_2D_concave(orig_data_path, name, 0.5)
# =========================================
#            Calculate area
# =========================================
f = codecs.open(orig_data_path + 'concave_hull_results/' + concave_hull_file_name,
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
list2 = np.array(list1).astype(float)
pts = np.array([point for point in list2])
orig_poly_pts = polygon_point(orig_data_path + 'concave_hull_results/' + concave_hull_file_name)

orig_ch_area = get_area(pts) # 计算面积

# =========================================
#        Write area results into file
# =========================================
concave_hull_area_list = []
file_line = ''
file_line += ("{} {} {}\n".format(name.replace(".txt"," "), orig_ch_area, orig_poly_pts.intersection(orig_poly_pts).area/orig_ch_area))
# orig_poly_pts.intersection(orig_poly_pts).area 是原始点云与原始点云自己的相交面积   除以orig_ch_area，即/orig_ch_area求到百分比

file_name = 'Orig_2D_Ch_Area_Results.txt'
f = open(orig_data_path + 'concave_hull_results/' + file_name, 'a')
f.write("way Area %Area\n")
f.write(file_line)
f.close()



# =========================================
#
#        Random data Concave Hull
#
# =========================================
# 读取所有Random开头的文件
random_files = []
data_dir = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/"
for i in os.listdir(data_dir):
    if os.path.isfile(os.path.join(data_dir,i)) and 'Random' in i:
        random_files.append(i)
seed_count = 0
file_dir = data_dir + 'concave_hull_results/'
if not os.path.exists(file_dir):
    os.makedirs(file_dir)
file_name = 'Random_2D_Ch_Area_Results.txt'
f = open(file_dir + file_name, 'a')
#f.write("Seed Area\n")
for random_file in random_files:
    concave_hull_file_name = get_2D_concave(data_dir, random_file, 0.5)
    print(concave_hull_file_name)
    # =========================================
    #            Calculate area
    # =========================================
    f = codecs.open('C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/concave_hull_results/' + concave_hull_file_name,
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
    list2 = np.array(list1).astype(float)
    pts = np.array([point for point in list2])
    random_poly_pts = polygon_point('C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/concave_hull_results/' + concave_hull_file_name)

    random_ch_area = get_area(pts) # 计算面积

    seed_value_list = [int(x) for x in re.findall('\d+', random_file)]
    seed_value = seed_value_list[2]

    # =========================================
    #        Write area results into file
    # =========================================
    concave_hull_area_list = []
    file_line = ''

    seed_count += 1
    print(seed_count)
    file_line += ("{} {} {}\n".format(seed_value, random_ch_area, random_poly_pts.intersection(orig_poly_pts).area/orig_ch_area))
    #
    file_name = 'Random_2D_Ch_Area_Results.txt'
    f = open(file_dir + file_name, 'a')
    f.write(file_line)
    f.close()

# =========================================
#
#      Octree and BDSR data Concave Hull
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
    concave_hull_file_name = get_2D_concave(data_dir, data_name, 0.5) # 0.5 --> alpha value
    print(concave_hull_file_name)

    # =========================================
    #            Calculate area
    # =========================================
    f = codecs.open(
        "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/" + way_folder + '/concave_hull_results/' + concave_hull_file_name,
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
    list2 = np.array(list1).astype(float)
    pts = np.array([point for point in list2])
    octree_bdsr_poly_pts = polygon_point("C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/" + way_folder + '/concave_hull_results/' + concave_hull_file_name)

    octree_bdsr_ch_area = get_area(pts)  # 计算面积

    # =========================================
    #        Write area results into file
    # =========================================
    file_name = '{}_2D_Ch_Area_Results.txt'.format(way)
    file_dir = data_dir + 'concave_hull_results/'
    f = open(file_dir + file_name, 'a')
    file_line = ""
    file_line += ("{} {} {}\n".format(way, octree_bdsr_ch_area, octree_bdsr_poly_pts.intersection(orig_poly_pts).area/orig_ch_area))
    #
    f.write(file_line)
    f.close()


# =================================================
#
#    Concatenate files and generate final result
#
# =================================================

orig_ch_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file/concave_hull_results/Orig_2D_Ch_Area_Results.txt"
FA_ch_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/FA/txt_file1/concave_hull_results/FA_2D_Ch_Area_Results.txt"
PCA_ch_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/PCA/txt_file1/concave_hull_results/PCA_2D_Ch_Area_Results.txt"
KernelPCA_ch_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/KernelPCA/txt_file1/concave_hull_results/KernelPCA_2D_Ch_Area_Results.txt"
TruncatedSVD_ch_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/TruncatedSVD/txt_file1/concave_hull_results/TruncatedSVD_2D_Ch_Area_Results.txt"
octDown_ch_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/concave_hull_results/octreeDownsampling_2D_Ch_Area_Results.txt"
octRe_ch_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/concave_hull_results/octreeResampling_2D_Ch_Area_Results.txt"
random_ch_file = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Oct_Rand_Sample_1/concave_hull_results/Random_2D_Ch_Area_Results.txt"

area_list = [orig_ch_file, FA_ch_file, PCA_ch_file, KernelPCA_ch_file, TruncatedSVD_ch_file, octDown_ch_file, octRe_ch_file, random_ch_file]

with open('C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/Performance/2D_Ch_Area_Results.txt', 'w') as outfile:
    for area in area_list:
        with open(area) as infile:
            outfile.write(infile.read())
    outfile.close()

