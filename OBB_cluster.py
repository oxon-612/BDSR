#计算每个cluster的OBB
import numpy as np
#from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import pyrr
from sklearn.preprocessing import MinMaxScaler
#from anytree.dotexport import RenderTreeGraph
from anytree import Node, LevelOrderIter
from scipy.spatial import ConvexHull
import os
from os.path import basename
import re
#from operator import itemgetter
from sympy import Plane, Line, Line3D, Point3D
import codecs
import math


def scale_data(_score):
    # Scale data
    mm = MinMaxScaler()
    _score = mm.fit_transform(_score)
    return _score
def get_basis(_score):
    '''
    get Covariance basis
    '''
    cov_matrix = np.cov(_score.T)
    # print("COV: ")
    # print(cov_matrix)
    u, s, v = np.linalg.svd(cov_matrix, full_matrices=False)
    return u
def get_cube_vertexs(_p1, _p8):
    '''
    :param _p1: vertex 1
    :param _p8: vertex 8
    :return: 8 vertex of cube
    '''
    _length, _width, _height = abs(_p1[0] - _p8[0]), abs(_p1[1] - _p8[1]), abs(_p1[2] - _p8[2])
    _p2 = [_p1[0] + _length, _p1[1], _p1[2]]
    _p3 = [_p1[0] + _length, _p1[1] + _width, _p1[2]]
    _p4 = [_p1[0], _p1[1] + _width, _p1[2]]
    _p5 = [_p8[0] - _length, _p8[1], _p8[2]]
    _p6 = [_p8[0] - _length, _p8[1] - _width, _p8[2]]
    _p7 = [_p8[0], _p8[1] - _width, _p8[2]]
    _points = np.array([_p1, _p2, _p3, _p4, _p5, _p6, _p7, _p8])
    return _points
def get_centroid_of_cube(_cube):
    _center = [(_cube[0][0] + _cube[7][0]) / 2, (_cube[0][1] + _cube[7][1]) / 2, (_cube[0][2] + _cube[7][2]) / 2]
    return _center
def split_8_cubes(_cube):
    '''
    split into 8 equal parts
    :param _cube:
    :return:
    '''
    center = get_centroid_of_cube(_cube)
    _cube1 = get_cube_vertexs(_cube[0], center)
    _cube2 = get_cube_vertexs([center[0], _cube[0][1], _cube[0][2]], [_cube[1][0], center[1], center[2]])
    _cube3 = get_cube_vertexs([center[0], center[1], _cube[0][2]], [_cube[2][0], _cube[2][1], center[2]])
    _cube4 = get_cube_vertexs([_cube[0][0], center[1], _cube[0][2]], [center[0], _cube[3][1], center[2]])
    _cube5 = get_cube_vertexs([_cube[0][0], center[1], center[2]], [center[0], _cube[4][1], _cube[4][2]])
    _cube6 = get_cube_vertexs([_cube[0][0], _cube[0][1], center[2]], [center[0], center[1], _cube[4][2]])
    _cube7 = get_cube_vertexs([center[0], _cube[0][1], center[2]], [_cube[1][0], center[1], _cube[4][2]])
    _cube8 = get_cube_vertexs(center, _cube[7])
    _cube_list = list()
    _cube_list.append(_cube1)
    _cube_list.append(_cube2)
    _cube_list.append(_cube3)
    _cube_list.append(_cube4)
    _cube_list.append(_cube5)
    _cube_list.append(_cube6)
    _cube_list.append(_cube7)
    _cube_list.append(_cube8)
    return np.array(_cube_list)
def octree(_cube, _root, _depth=1):
    # once equal split
    if _depth == 1:
        _first_cubes = split_8_cubes(_cube)
        return np.array(_first_cubes)
    elif _depth == 2:
        # first split
        first_cubes = split_8_cubes(_cube)
        _second_cube_list = list()
        for first_cube in first_cubes:
            # second split
            _second_cube_list.append(split_8_cubes(first_cube))
        return np.array(_second_cube_list)
    elif _depth == 3:
        # first split
        first_cubes = split_8_cubes(_cube)
        third_cube_list = list()
        for first_cube in first_cubes:
            # second split
            second_cubes = split_8_cubes(first_cube)
            for second_cube in second_cubes:
                # third split
                third_cube_list.append(split_8_cubes(second_cube))
        return np.array(third_cube_list)
def create_tree(_root_cube, _tree_root, depth):
    transformed_first_cubes = split_8_cubes(_root_cube)
    if depth == 1:
        for transformed_first_cube_index in range(0, len(transformed_first_cubes)):
            transformed_first_cube = transformed_first_cubes[transformed_first_cube_index]
            original_first_cube = transformed_first_cube @ u_inv
            first_level = Node("Cube_" + str(transformed_first_cube_index + 1), parent=_tree_root, vertexs=original_first_cube)
        return _tree_root
    elif depth == 2:
        for transformed_first_cube_index in range(0, len(transformed_first_cubes)):
            transformed_first_cube = transformed_first_cubes[transformed_first_cube_index]
            original_first_cube = transformed_first_cube @ u_inv
            first_level = Node("Cube_" + str(transformed_first_cube_index + 1), parent=_tree_root, vertexs=original_first_cube)
            transformed_second_cubes = split_8_cubes(transformed_first_cube)
            for transformed_second_cube_index in range(0, len(transformed_second_cubes)):
                transformed_second_cube = transformed_second_cubes[transformed_second_cube_index]
                original_second_cube = transformed_second_cube @ u_inv
                second_level = Node("Cube_" + str(transformed_first_cube_index + 1) + "_" + str(transformed_second_cube_index + 1), parent=first_level, vertexs=original_second_cube)
        return _tree_root
    elif depth ==3:
        for transformed_first_cube_index in range(0, len(transformed_first_cubes)):
            transformed_first_cube = transformed_first_cubes[transformed_first_cube_index]
            original_first_cube = transformed_first_cube @ u_inv
            first_level = Node("Cube_" + str(transformed_first_cube_index + 1), parent=_tree_root, vertexs=original_first_cube)
            transformed_second_cubes = split_8_cubes(transformed_first_cube)
            for transformed_second_cube_index in range(0, len(transformed_second_cubes)):
                transformed_second_cube = transformed_second_cubes[transformed_second_cube_index]
                original_second_cube = transformed_second_cube @ u_inv
                second_level = Node("Cube_" + str(transformed_first_cube_index + 1) + "_" + str(transformed_second_cube_index + 1), parent=first_level, vertexs=original_second_cube)
                # level 3
                transformed_third_cubes = split_8_cubes(transformed_second_cube)
                for transformed_third_cube_index in range(0, len(transformed_third_cubes)):
                    transformed_third_cube = transformed_third_cubes[transformed_third_cube_index]
                    original_third_cube = transformed_third_cube @ u_inv
                    third_level = Node("Cube_" + str(transformed_first_cube_index + 1) + "_" + str(transformed_second_cube_index + 1) + "_" + str(transformed_third_cube_index + 1), parent=second_level, vertexs=original_third_cube)
        return _tree_root
def point_in_cuboid(points, _new_point):
    hull = ConvexHull(points)
    points = np.insert(points, len(points), _new_point, axis=0)
    new_hull = ConvexHull(points)
    if hull.volume == new_hull.volume:
        return True
    else:
        return False

folder = "FA"
path_cluster = "C:/Users/cvrl/Documents/PCD_projects/Results/BDSR/"
segment_dir = path_cluster + folder + "/cluster_txt/"
# /------------------------------------/
# /               OBB                  /
# /------------------------------------/
segments_f_list = [i for i in os.listdir(segment_dir) if os.path.isfile(os.path.join(segment_dir, i))]
f_count = 0

#fig = plt.figure(figsize=(18, 5))
#ax = fig.add_subplot(111, projection='3d')
#ax._axis3don = False

OBB_vertex_list = []
for a in segments_f_list:
    f_count += 1
    f_name = os.path.splitext(basename(a))[0]
    #print("{}.".format(f_count), f_name)
    f_data = np.array(np.loadtxt(segment_dir+a, skiprows=2, usecols=[0, 1, 2]))

    basis = get_basis(f_data) # get the new basis

    new_f_data = f_data@basis # projection to new coordinates system
    transformed_aabb_points = pyrr.aabb.create_from_points(new_f_data) # AABB in new coord system --> OBB in orginal coord system
    # get 8 vertex of the OBB
    # accoding to the returned 2 points coords
    transformed_cube_points = get_cube_vertexs(transformed_aabb_points[0], transformed_aabb_points[1])

    u_inv = np.linalg.inv(basis) # inverse matrix of U, used for map to original coord system
    # print("INVERSE MATRIX:")
    # print(u_inv)
    original_points = transformed_cube_points@u_inv # transform to original coord system
    OBB_vertex_list.append(original_points)
    #print(OBB_vertex_list)
    #print(type(OBB_vertex_list));
    #将顶点数据输出到txt文件中
    file = open(path_cluster + folder+'/vertex_txt_out/vertex.txt', 'w')
    for i in range(len(OBB_vertex_list)):
        #写入txt文件
        s=str(OBB_vertex_list[i]).replace('[', '').replace(']', '')
        s = s.replace("'", '').replace(',', '') + '\n'
        file.write(s)
    file.close()
    #print(OBB_vertex_list)
    #print(len(OBB_vertex_list))

#计算每八个顶点之间的距离
#读取txt文件中的所有数据
f = codecs.open(path_cluster + folder + '/vertex_txt_out/vertex.txt', mode='r',
                encoding='utf-8')  # 打开txt文件，以‘utf-8’编码读取
line = f.readline()  # 以行的形式进行读取文件
list1 = []
while line:
    a = line.split()
    b = a[0:3]  # 这是选取需要读取的位数 前两列 X,Y
    list1.append(b)  # 将其添加在列表之中
    line = f.readline()
f.close()
x = [float(i[0]) for i in list1]
y = [float(i[1]) for i in list1]
z = [float(i[2]) for i in list1]
length = len(x)
_index = np.arange(0,length,8) #每一组的起始点，每一组是八个点,length是对应的OBB的总个数
list2 = np.array(list1).astype(float)
points = np.array([point for point in list2])
#print(points[0])

_alldistance = []
for i in _index:
    _distance = []
    for j in range(7):
        distance = math.sqrt((points[i][0] - points[j+i+1][0])*(points[i][0] - points[j+i+1][0]) +
			(points[i][1] - points[j+i+1][1])*(points[i][1] - points[j+i+1][1]) +
			(points[i][2] - points[j+i+1][2])*(points[i][2] - points[j+i+1][2]))
        _distance.append(distance)
        _alldistance.append(distance)
        #print("i : ",i,"j+i+1 : ",j+i+1,"dis: ",distance)
    #每个cluster的min_dimension
    print("min_dimension of cluster " , math.ceil((i+1)/8), ": ",np.array(_distance).min())
    #print(_distance)
min_dis = np.array(_alldistance).min()
print("min dimension of all clusters: ", min_dis)
    #print((np.array(_distance).min().mean()))
# 将最小距离写入txt文件
file = open(path_cluster + folder + '/txt_file1/min_dimension.txt', 'w')
for i in range(1):
    # 写入txt文件
    s = str(min_dis)
    file.write(s)
file.close()
