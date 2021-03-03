import numpy as np
import random
import matplotlib.pyplot as plt
import os,sys,shutil

# 生成指定范围内不重复的随机数
def _random(a, b, n) :
    # 定义一个空列表存储随机数
    a_list = []
    while len(a_list) < n :
        d_int = random.uniform(a, b)
        if(d_int not in a_list) :
            a_list.append(d_int)
        else :
            pass
    return a_list

# Noise
def add_noise(_linspaces, rand_min, rand_max):
    new_linspace = list()
    for _linspace in _linspaces:
        new_linspace.append(_linspace + random.uniform(rand_min, rand_max))
    return new_linspace

#分段函数
def f1(x):
    if 0<=x<=0.5:
        return 0.5*x
    elif 0.5<x<1:
        return 0.25
    elif 1<=x<1.5:
        return -0.5*x+0.75
    else:
        return 0

def f2(x):
    if 0 <= x <= 0.5:
        return -0.5*x
    elif 0.5 < x < 1:
        return -0.25
    elif 1 <= x <1.5:
        return 0.5*x - 0.75
    else:
        return 0

def face_thwartwise(x_min,x_max,step,function,file,noise_min,noise_max,rgb,z_min,z_max):
    fw = open(file, 'w')
    x = np.arange(x_min, x_max, step).T
    y = []
    for i in x:
        y_1 = function(i)
        y.append(y_1)
    x = add_noise(x.reshape(x.shape[0], 1), noise_min, noise_max)
    y = add_noise(np.array(y).reshape(np.array(y).shape[0], 1), noise_min, noise_max)
    z = np.array(_random(z_min, z_max, len(y))).reshape(np.array(_random(z_min, z_max, len(y))).shape[0], 1)
    xyz = np.hstack((np.hstack((x, y)), z[:, 0].reshape(z[:, 0].shape[0], 1)))
    rgb = np.array([rgb] * len(y)).reshape(np.array([rgb] * len(y)).shape[0], 1)
    points = np.hstack((xyz, rgb[:, 0].reshape(rgb[:, 0].shape[0], 1)))
    for line in points:
        for a in line:
            fw.write(str(a))
            fw.write('\t')
        fw.write('\n')
    fw.close()

number = "hexagon"
face_thwartwise(0,1.5,0.003,f1,'E:/XMJ/3Drebuilding/paper/test/test_file/'+number+'_1.txt',-0.005,0.005,200,0,2)
face_thwartwise(0,1.5,0.003,f2,'E:/XMJ/3Drebuilding/paper/test/test_file/'+number+'_2.txt',-0.005,0.005,200,0,2)

#combine faces
match_image_list = []  # 初始化一个空列表
for i in np.arange(1,3,1):
    match_image_i_class = open("E:/XMJ/3Drebuilding/paper/test/test_file/" + number + "_"+str(i)+".txt", "r")
    match_image_i_class_file = match_image_i_class.readlines()
    for match_image_i_class_file_one in match_image_i_class_file:
        match_image_list.append(match_image_i_class_file_one)  # 读取一个插入一个
# 写入一个新的txt文件中
file = open("E:/XMJ/3Drebuilding/paper/test/test_file/"+number+"_result.txt", "w")
for i in match_image_list:
    file.write(i)
file.close()


'''#face 1
fw=open('E:/XMJ/3Drebuilding/paper/test/test_file/'+number+'_1.txt','w')
x = np.arange(0, 1.5, 0.001).T
y = []
for i in x:
    y_1 = f1(i)
    y.append(y_1)
length = len(y)
print(length)

x = x.reshape(x.shape[0],1)
y = np.array(y)
y = y.reshape(y.shape[0],1)
x = add_noise(x, -0.005, 0.005)
y = add_noise(y, -0.005, 0.005)
xy = np.hstack((x, y))
z = _random(0,2,length)
z = np.array(z)
z = z.reshape(z.shape[0], 1)
xyz = np.hstack((xy, z[:, 0].reshape(z[:, 0].shape[0], 1)))
rgb = [200]*length
rgb = np.array(rgb)
rgb = rgb.reshape(rgb.shape[0], 1)
points = np.hstack((xyz, rgb[:, 0].reshape(rgb[:, 0].shape[0], 1)))
for line in points:
    for a in line:
        fw.write(str(a))
        fw.write('\t')
    fw.write('\n')


fw.close()

#face 2
fw=open('E:/XMJ/3Drebuilding/paper/test/test_file/'+number+'_2.txt','w')
x = np.arange(0, 1.5, 0.001).T
y = []
for i in x:
    y_1 = f2(i)
    y.append(y_1)
length = len(y)
print(length)

x = x.reshape(x.shape[0],1)
y = np.array(y)
y = y.reshape(y.shape[0],1)
x = add_noise(x, -0.005, 0.005)
y = add_noise(y, -0.005, 0.005)
xy = np.hstack((x, y))
z = _random(0,2,length)
z = np.array(z)
z = z.reshape(z.shape[0], 1)
xyz = np.hstack((xy, z[:, 0].reshape(z[:, 0].shape[0], 1)))
rgb = [200]*length
rgb = np.array(rgb)
rgb = rgb.reshape(rgb.shape[0], 1)
points = np.hstack((xyz, rgb[:, 0].reshape(rgb[:, 0].shape[0], 1)))
for line in points:
    for a in line:
        fw.write(str(a))
        fw.write('\t')
    fw.write('\n')


fw.close()'''