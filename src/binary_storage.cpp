/*
 * @Author       : baizs zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-16 11:59:43
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-08-07 17:47:02
 * @FilePath     : /calib_reflectivity/src/binary_storage.cpp
 * @Description  : 读写二进制文件（反射率表）
 *
 * Copyright (c) 2024 by vitalchem, All Rights Reserved.
 */
#include "binary_storage.h"
#include <iostream>
#include <fstream>

namespace binary_storage
{
    bool saveMapToBinary(const std::string &filename,
                         const std::map<uint16_t, std::map<float, std::array<float, _reflect_size>>> &map)
    {
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs)
        {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            return false;
        }

        uint16_t outer_size = map.size();
        ofs.write(reinterpret_cast<const char *>(&outer_size), sizeof(outer_size));
        for (const auto &entry : map)
        {
            ofs.write(reinterpret_cast<const char *>(&entry.first), sizeof(entry.first));
            uint16_t inner_size = entry.second.size();
            ofs.write(reinterpret_cast<const char *>(&inner_size), sizeof(inner_size));
            for (const auto &sub_entry : entry.second)
            {
                ofs.write(reinterpret_cast<const char *>(&sub_entry.first), sizeof(sub_entry.first));
                ofs.write(reinterpret_cast<const char *>(sub_entry.second.data()),
                          sizeof(float) * sub_entry.second.size());
            }
        }

        ofs.close();
        return true;
    }

    bool loadMapFromBinary(const std::string &filename,
                           std::map<uint16_t, std::map<float, std::array<float, _reflect_size>>> &map)
    {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs)
        {
            std::cerr << "Failed to open file for reading: " << filename << std::endl;
            return false;
        }

        uint16_t outer_size;
        ifs.read(reinterpret_cast<char *>(&outer_size), sizeof(outer_size));
        for (uint16_t i = 0; i < outer_size; ++i)
        {
            uint16_t outer_key;
            ifs.read(reinterpret_cast<char *>(&outer_key), sizeof(outer_key));
            uint16_t inner_size;
            ifs.read(reinterpret_cast<char *>(&inner_size), sizeof(inner_size));

            std::map<float, std::array<float, _reflect_size>> inner_map;
            for (uint32_t j = 0; j < inner_size; ++j)
            {
                float inner_key;
                std::array<float, _reflect_size> inner_value;
                ifs.read(reinterpret_cast<char *>(&inner_key), sizeof(inner_key));
                ifs.read(reinterpret_cast<char *>(inner_value.data()), sizeof(float) * inner_value.size());
                inner_map[inner_key] = inner_value;
            }

            map[outer_key] = inner_map;
        }

        ifs.close();
        return true;
    }
    // 保存 map 到 CSV
    bool saveMapToCsv(const std::string &filename,
                      const std::map<uint16_t, std::map<float, std::array<float, _reflect_size>>> &map)
    {

        std::ofstream ofs(filename);
        if (!ofs.is_open())
        {
            std::cerr << "错误: 无法打开文件进行写入: " << filename << std::endl;
            return false;
        }

        // 写入CSV头部
        ofs << "pixel_target,distance";
        for (size_t i = 0; i < _reflect_size; ++i)
        {
            ofs << ",reflectance_" << i;
        }
        ofs << "\n"; // 换行

        // 遍历 map 并写入数据
        for (const auto &entry : map)
        {
            uint16_t pixel_target = entry.first;
            for (const auto &sub_entry : entry.second)
            {
                float distance = sub_entry.first;
                const std::array<float, _reflect_size> &peaks = sub_entry.second;

                ofs << pixel_target << "," << distance;
                for (const float peak : peaks)
                {
                    ofs << "," << peak;
                }
                ofs << "\n"; // 换行
            }
        }

        ofs.close();
        if (ofs.fail())
        {
            std::cerr << "错误: 写入文件时发生错误: " << filename << std::endl;
            return false;
        }

        std::cout << "成功将映射表保存到 " << filename << std::endl;
        return true;
    }
}
