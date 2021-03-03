#第二次降采样使用的代码
import numpy as np
import random
# import sys
import os
from sklearn import preprocessing
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import math
from scipy.stats import norm
# from sklearn.neighbors import KernelDensity
# import statistics
# from scipy.stats import ks_2samp
# from scipy.stats import ttest_1samp
# from scipy.stats import ttest_ind
# from scipy.stats import chisquare
# from scipy.spatial import ConvexHull



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
def get_spec_norm_list(_list):
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

if __name__ == '__main__':
    folder = "FA"
    CR1 = "0.2" # 这是BDS里面选中的一压 压缩率 BDSR_project.cpp 一次只能做一个一压压缩率
    path_2 = "D:/MengjieXu/Science/BDSR2019-2020/test202009/BDSR/"

    dataset = path_2 + folder + "/txt_file1/" + folder +"_down_" + CR1 + "_NC_bounding_upsampling_result2.txt"

    raw_data = np.loadtxt(dataset)
    BDS_Downsampling(_input_data=raw_data[:, 0:3], _output_dir=path_2 + folder + "/")

    evaluation_table_dir = path_2 + folder + "/evaluation_data/"

up_to_how_many_to_keep = 0.4  #保留数据的50%
for how_many_to_keep in np.arange(start = 0.3, stop = up_to_how_many_to_keep, step = 0.1): #从1%开始，压缩率是1%，一直到10%，可以自己设置

    ################################
    #                              #
    #      down-sampled PCD        #
    #                              #
    ################################
    # for PCA, FA, KernelPCA，TruncatedSVD
    down_method_list = ['FA', 'PCA', 'KernelPCA', 'TruncatedSVD']  #每一个都用上一个循环的压缩率过一遍

    print("*" * 22)
    print("*                    *")
    print("*                    *")
    print("*   keep {} data    *".format(how_many_to_keep))
    print("*                    *")
    print("*                    *")
    print("*" * 22)

    '''#不输出30倍的random文件的代码，因为名称相同，前面的均被覆盖
    for down_method in down_method_list:
         output_f_dir = "E:/XMJ/3Drebuilding/paper/test/test_2019_10/test32/down_sampled_data2/"
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

    #输出30倍random文件的代码，因为名称中加了rand_seed，使得名称各不相同，不会覆盖了
    for down_method in down_method_list:
        output_f_dir = path_2 + folder + "/down_sampled_data/"
        output_f_name = "{}_down_{}_PCD.txt".format(down_method, how_many_to_keep)

        bds_re_ordered_data = np.loadtxt(path_2 + folder + "/" + down_method + ".txt")
        down_data = bds_re_ordered_data[0:int(how_many_to_keep*len(bds_re_ordered_data))]
        np.savetxt(output_f_dir + output_f_name, down_data)