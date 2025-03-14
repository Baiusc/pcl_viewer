'''
Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
Date         : 2025-02-25 10:44:34
LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
LastEditTime : 2025-03-12 16:43:07
FilePath     : /pcl_viewer/python/os_script.py
Description  : 

Copyright (c) 2025 by vitalchem, All Rights Reserved. 
'''
import os

def add_prefix_to_files(folder_path, prefix, file_extension):
    """
    为文件夹内所有指定扩展名的文件添加前缀
    :param folder_path: 文件夹路径
    :param prefix: 要添加的前缀
    :param file_extension: 文件扩展名（如 '.csv'）
    """
    # 遍历文件夹中的所有文件
    for filename in os.listdir(folder_path):
        # 检查文件是否为指定扩展名的文件
        if filename.endswith(file_extension):
            # 构造新的文件名
            new_filename = prefix + filename
            # 获取文件的完整路径
            old_file_path = os.path.join(folder_path, filename)
            new_file_path = os.path.join(folder_path, new_filename)
            # 重命名文件
            os.rename(old_file_path, new_file_path)
            print(f'Renamed: {filename} -> {new_filename}')

def add_suffix_to_files(folder_path, suffix, file_extension):
    """
    为文件夹内所有指定扩展名的文件在文件名末尾添加后缀
    :param folder_path: 文件夹路径
    :param suffix: 要添加的后缀
    :param file_extension: 文件扩展名（如 '.csv'）
    """
    # 遍历文件夹中的所有文件
    for filename in os.listdir(folder_path):
        # 检查文件是否为指定扩展名的文件
        if filename.endswith(file_extension):
            # 构造新的文件名
            name, ext = os.path.splitext(filename)  # 分离文件名和扩展名
            new_filename = name + suffix + ext  # 在文件名末尾添加后缀
            # 获取文件的完整路径
            old_file_path = os.path.join(folder_path, filename)
            new_file_path = os.path.join(folder_path, new_filename)
            # 重命名文件
            os.rename(old_file_path, new_file_path)
            print(f'Renamed: {filename} -> {new_filename}')

def extract_and_rename_txt_files(folder_path):
    """
    提取 folder_path 文件夹下所有子文件夹中的 .txt 文件，并依次改名为 1.txt, 2.txt, 3.txt 等。
    :param folder_path: 文件夹路径
    """
    # 初始化计数器
    counter = 1
    
    # 递归遍历文件夹及其子文件夹
    for root, dirs, files in os.walk(folder_path):
        for filename in files:
            # 检查文件是否为 .txt 文件
            if filename.endswith('.txt'):
                # 获取文件的完整路径
                old_file_path = os.path.join(root, filename)
                # 构造新的文件名
                new_filename = f'{counter}.txt'
                new_file_path = os.path.join(folder_path, new_filename)
                # 重命名并移动文件
                os.rename(old_file_path, new_file_path)
                print(f'Moved and renamed: {old_file_path} -> {new_file_path}')
                # 更新计数器
                counter += 1

# 示例用法
if __name__ == '__main__':
    # 定义文件夹路径
    folder_path = './data/far_distance'

    # 文件扩展名
    file_extension = '.txt'

    # 为所有指定扩展名的文件添加前缀
    # add_prefix_to_files(folder_path, '20.0m_', file_extension)

    # 提取并重命名所有子文件夹中的 .txt 文件
    extract_and_rename_txt_files(folder_path)

    # 为所有指定扩展名的文件在文件名末尾添加后缀
    # add_suffix_to_files(folder_path, 'm', file_extension)  # 添加 'm' 作为后缀