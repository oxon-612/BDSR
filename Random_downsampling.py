#第一次降采样用的代码
import numpy as np
import random


if __name__ == '__main__':
    # folder = "1" # '1' is the '1' in 'Result_BDSR1' folder
    path_1 = "D:/MengjieXu/Science/BDSR2019-2020/test202009/BDSR/"
    path_2 = "D:/MengjieXu/Science/BDSR2019-2020/test202009/"

    dataset = path_1 + "FA" + "/txt_file/2.txt"
    raw_data = np.loadtxt(dataset)
    print("nb of pts:", len(raw_data))

    cr1 = 0.2
    cr2 = 0.3

    #result_PCA = len(np.loadtxt(path_1 + 'PCA/txt_file1/PCA_down_{}_{}_PCD.txt'.format(cr1, cr2)))
    result_FA = len(np.loadtxt(path_1 + 'FA/txt_file1/FA_down_{}_{}_PCD.txt'.format(cr1, cr2)))
    #result_KernelPCA = len(np.loadtxt(path_1 + 'KernelPCA/txt_file1/KernelPCA_down_{}_{}_PCD.txt'.format(cr1, cr2)))
    #result_TruncatedSVD = len(np.loadtxt(path_1 + 'TruncatedSVD/txt_file1/TruncatedSVD_down_{}_{}_PCD.txt'.format(cr1, cr2)))

    #result_list = [result_PCA, result_FA, result_KernelPCA, result_TruncatedSVD]

    how_many_to_keep = np.average(3300)
    cr_random = how_many_to_keep/len(raw_data)


    ################################
    #                              #
    #      down-sampled PCD        #
    #                              #
    ################################
    # for Random
    down_method = 'Random'  # 每一个都用上一个循环的压缩率过一遍

    # fix 30 seeds to calculate
    rand_seed_list = [25, 8111, 909, 874, 7, 10005, 12, 237, 5467, 9001,
                      205, 7118, 910, 234, 722, 105, 1232, 200037, 57174, 336,
                      117, 20, 41299, 34158, 107, 2097, 211, 102, 5016, 67]  # random还自己设置了30个种子，因此还要进行30次循环


    print("*" * 22)
    print("*                    *")
    print("*                    *")
    print("*   keep {} data    *".format(cr_random))
    print("*                    *")
    print("*                    *")
    print("*" * 22)


    # 输出30倍random文件的代码，因为名称中加了rand_seed，使得名称各不相同，不会覆盖了

    output_f_dir = path_2 + "OC_Ran_1/"
    output_f_name = "{}_down_{}_PCD.txt".format(down_method, cr_random)

    # random down-sampling
    rand_count = 0

    for rand_seed in rand_seed_list:
        rand_count += 1
        random.seed(rand_seed)
        down_data = random.sample(list(raw_data), int(cr_random * len(raw_data)))
        output_f_name = "{}_down_{}_{}_PCD.txt".format(down_method, cr_random, rand_seed)
        np.savetxt(output_f_dir + output_f_name, down_data)