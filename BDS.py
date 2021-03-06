#第一次降采样用的代码
import numpy as np
import random
import os
from sklearn import preprocessing
import pyrr
from sklearn.preprocessing import MinMaxScaler
#from anytree.dotexport import RenderTreeGraph
from anytree import Node, LevelOrderIter
from scipy.spatial import ConvexHull
from os.path import basename
import codecs
import math

# /------------------------------------/
# /                                    /
# /         BDS functions              /
# /                                    /
# /------------------------------------/
"""
1. Get down-sampled PCD by using dimension-reduction methods: Random, PCA, FA, KernelPCA, TruncatredSVD 
2. For each dimension-reduction method, get 10%, 20%, ..., 90% of original data (down-sampling) and save the down-sampled data
"""

def BDS_Downsampling(_input_data, _output_dir, _digit=38, _which_dimension_reduction = ['PCA', 'FA', 'KernelPCA', 'TruncatedSVD']):
    '''
    A function to conduct the best-discripancy downsampling
    :param _input_data: a multi-dimensional dataset with feacture vectors and a class label vector
    :param _digit: how many digits after the decimal place of constant e, by default 38
    :param _which_dimension_reduction: choose one or multiple dimensionality reduction technique(s) to produce a linear transformation T and to result in an one-dimensional vector E
    ['PCA', 'FA', 'KernelPCA', 'TruncatedSVD']
    :return: mean EPEs over k iterations of the three classifiers
    '''

    def get_BDS(_r, _digit):
        '''
        A subfunction to gerenate a best-discrepancy number with Equation 3
        :param _r: an integer
        :param _digit: round the best-discrepancy number to a certain number of digits
        :return: a best-discrepancy number
        '''
        _product = _r * 2.71828182845904523536028747135266249775
        _product_decimal = round(_product - int(_product), _digit)
        return float(str(_product_decimal))

    def get_rank(_input_list):
        '''
        A subfunction to get a ranking vector of a sequence
        :param _input_list: a one-dimensional list
        :return: a ranking vector
        '''
        _array = np.array(_input_list)
        _temp = _array.argsort()
        _ranks = np.arange(len(_array))[_temp.argsort()]
        return list(_ranks)

    def dimension_redu(_data, _method):
        '''
        A subfunction to transform a multi-dimensional dataset from the high-diemsnional space to a one-dimensional space
        :param _data: a multi-dimensional dataset
        :param _method: one or multiple dimensionality-reduction techniques
        :return: a one-dimensional vector
        '''
        min_max_scaler = preprocessing.MinMaxScaler()
        # print(_data[:, :-2])

        z_data = min_max_scaler.fit_transform(_data)
        # print(z_data)
        from sklearn import decomposition
        # Choose one method
        if _method == 'PCA':
            dim_redu_method = decomposition.PCA(n_components=1)
        elif _method == 'FA':
            dim_redu_method = decomposition.FactorAnalysis(n_components=1, max_iter=5000)
        elif _method == 'KernelPCA':
            dim_redu_method = decomposition.KernelPCA(kernel='cosine', n_components=1)
        elif _method == 'TruncatedSVD':
            dim_redu_method = decomposition.TruncatedSVD(1)

        dimension_redu_vector = dim_redu_method.fit_transform(z_data)

        z_dimension_redu_vector = np.ndarray.tolist(min_max_scaler.fit_transform(dimension_redu_vector))
        return z_dimension_redu_vector

    def get_temporary_data(_data, _dim_vector):
        '''
        A subfunction to
        1) attach the one-dimensional vector E to the original dataset D;
        2) assendingly sort E as E_tilde and then sort D as D_tilde
        :param _data: a multi-dimensional dataset D
        :param _dim_vector: the one-dimensional vector E
        :return: sorted dataset D_tilde
        '''
        _labels = _data[:, -1]
        _features = _data[:, :-1]
        #_features_minmax = np.ndarray.tolist(min_max_scaler.fit_transform(_features))  # normalize feature vectors
        _features_minmax = np.ndarray.tolist(_features)

        for i in range(len(_data)):
            _features_minmax[i].append(_labels[i])
            _features_minmax[i].append(_dim_vector[i][0])

        # D is sorted along E_tilde and becomes D_tilde
        _conjointed_data_sorted = sorted(_features_minmax, key=lambda a_entry: a_entry[-1])  # sort the dataset by the one-dimensional vector E_tilde
        # E_tilde is removed from D_tilde
        for cj in _conjointed_data_sorted:  # delete the one-dimensional vector E_tilde
            ################################################################################################
            #                                                                                              #
            #                             this is the one-dimensional feature                              #
            #                                                                                              #
            ################################################################################################
            # print(cj[-1])
            del cj[-1]
        rearranged_data = np.array(_conjointed_data_sorted)

        return rearranged_data

    min_max_scaler = preprocessing.MinMaxScaler()
    _duplicated_data = [i for i in _input_data]  # Create a copy of the input data so that the original input data won't be affected by a k-fold CV function.
    _data_size = len(_duplicated_data)
    # Generate a BDS with n elements using Equation 3
    _BD_seqence = []
    for bd in range(_data_size):
        _BD_seqence.append(get_BDS(bd + 1, _digit))
    print("Generate a BDS with {} elements using Equation 3".format(len(_BD_seqence)))
    # Generate the BDS's ranking vector R
    _BDS_ranking = list(get_rank(_BD_seqence))
    print("\n")
    print("Generate the ranking vector of the BDS with {} elements".format(len(_BDS_ranking)))
    # print(_BDS_ranking)
    print("\n")


    for dim_method in _which_dimension_reduction:
        print("-" * 100)
        print("Generate one-dimensional vector E based on D with a dimensionality-reduction technique {}".format(dim_method))
        print("-" * 100)
        _z_duplicated_data = min_max_scaler.fit_transform(_duplicated_data)
        _z_dim_vector = dimension_redu(_z_duplicated_data, dim_method)



        _temporary_data = get_temporary_data(_input_data, _z_dim_vector)
        print('\t',"Ascendingly sort E as E_tilde")
        print('\t',"Sort D as D_tilde using E_tilde")

        # print(_temporary_data[:, -1])

        _BDS_rearranged_data = []

        for l in _BDS_ranking:
            _BDS_rearranged_data.append(_temporary_data[l])
        print('\t',"D_tilde is rearranged with R, the ranking vector of a BDS")

        # _file_name='./Datasets/'+dim_method+"_Sleep"+".txt"
        _file_name = _output_dir + dim_method + ".txt"

        np.savetxt(_file_name, _BDS_rearranged_data)
"""
1. Read a data file
2. Dimension reduction
3. Get the lowest discrepancy
"""
def get_normalized_list(_list):
    '''
    normalize data to [0, 1]
    '''
    _norm_list = []
    for _i in _list:
        _j = (_i-min(_list))/(max(_list)-min(_list))
        _norm_list.append(_j)
    return _norm_list
def get_spec_norm_liste(_list):
    _zero_one_list = []
    for _i in _list:
        _j = (_i-min(_list))/(max(_list)-min(_list))
        _zero_one_list.append(_j)
    # [1, 100]
    _range_min = 1
    _range_max = 100
    _norm_list = []
    for _m in _zero_one_list:
        _n = _m * (_range_max-_range_min) + 1
        _norm_list.append(int(_n))
    return _norm_list

# /------------------------------------/
# /                                    /
# /         OBB functions              /
# /                                    /
# /------------------------------------/
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



if __name__ == '__main__':
    # folder = "1" # '1' is the '1' in 'Result_BDSR1' folder
    path_1 = "D:/MengjieXu/Science/BDSR2019-2020/test202009/BDSR/"

    for folder in ['PCA', 'FA', 'KernelPCA', 'TruncatedSVD']:

        dataset = path_1 + folder + "/txt_file/2.txt"
        raw_data = np.loadtxt(dataset)
        print("nb of pts:", len(raw_data))
        BDS_Downsampling(_input_data=raw_data[:, 0:3], _output_dir = path_1 + folder  + "/")

        evaluation_table_dir = path_1 + folder + "/evaluation_data/"

        up_to_how_many_to_keep = 0.25  # 保留数据的50%
        for how_many_to_keep in np.arange(start = 0.05, stop=up_to_how_many_to_keep, step = 0.05):  # 从1%开始，压缩率是1%，一直到10%，可以自己设置

            ################################
            #                              #
            #      down-sampled PCD        #
            #                              #
            ################################
            # for  PCA, FA, KernelPCA，TruncatedSVD
            down_method_list = ['PCA', 'FA', 'KernelPCA', 'TruncatedSVD']  # 每一个都用上一个循环的压缩率过一遍

            print("*" * 22)
            print("*                    *")
            print("*                    *")
            print("*   keep {} data    *".format(how_many_to_keep))
            print("*                    *")
            print("*                    *")
            print("*" * 22)

            '''#不输出30倍的random文件的代码，因为名称相同，前面的均被覆盖
            for down_method in down_method_list:
                 output_f_dir = "E:/XMJ/3Drebuilding/paper/test/test_2019_10/test32/down_sampled_data1/"
                 output_f_name = "{}_down_{}_PCD.txt".format(down_method, how_many_to_keep)
    
                 # random down-sampling
                 if down_method == 'Random':
                     rand_count = 0
    
                     for rand_seed in rand_seed_list:
                         rand_count += 1
                         random.seed(rand_seed)
                         down_data = random.sample(list(raw_data), int(how_many_to_keep*len(raw_data)))
                         np.savetxt(output_f_dir + output_f_name, down_data)
    
                 ################################################################################################################
                 ################################################################################################################
                 else:
                     bds_re_ordered_data = np.loadtxt("E:/XMJ/3Drebuilding/paper/test/test_2019_10/test20/" + down_method + ".txt")
                     down_data = bds_re_ordered_data[0:int(how_many_to_keep*len(bds_re_ordered_data))]
                     np.savetxt(output_f_dir + output_f_name, down_data)'''

            # 输出30倍random文件的代码，因为名称中加了rand_seed，使得名称各不相同，不会覆盖了
            for down_method in down_method_list:
                output_f_dir = path_1 + folder + "/down_sampled_data1/"
                output_f_name = "{}_down_{}_PCD.txt".format(down_method, how_many_to_keep)


                bds_re_ordered_data = np.loadtxt(path_1 + folder + "/" + down_method + ".txt")
                # print(len(bds_re_ordered_data))
                down_data = bds_re_ordered_data[0:int(how_many_to_keep * len(bds_re_ordered_data))]
                np.savetxt(output_f_dir + output_f_name, down_data)


        # /------------------------------------/
        # /                                    /
        # /               OBB                  /
        # /                                    /
        # /------------------------------------/
        segment_dir = path_1 + folder + "/txt_file/"
        segments_f_list = [i for i in os.listdir(segment_dir) if os.path.isfile(os.path.join(segment_dir, i))]
        f_count = 0
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
            file = open(path_1 + folder + '/vertex_txt_out/vertex1.txt', 'w')
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
        f = codecs.open(path_1 + folder + '/vertex_txt_out/vertex1.txt', mode='r',
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

        for i in _index:
            _distance = []
            for j in range(7):
                distance = math.sqrt((points[i][0] - points[j+i+1][0])*(points[i][0] - points[j+i+1][0]) +
                    (points[i][1] - points[j+i+1][1])*(points[i][1] - points[j+i+1][1]) +
                    (points[i][2] - points[j+i+1][2])*(points[i][2] - points[j+i+1][2]))
                _distance.append(distance)
                #print("i : ",i,"j+i+1 : ",j+i+1,"dis: ",distance)
            #每个cluster的min_dimension
            print("min_dimension of cluster " , math.ceil((i+1)/8), ": ",np.array(_distance).min())
            # 将最小距离写入txt文件
            file = open(path_1 + folder + '/txt_file1/min_dimension1.txt', 'w')
            for i in range(math.ceil((i+1)/8)):
                # 写入txt文件
                s = str(np.array(_distance).min())
                file.write(s)
            file.close()




