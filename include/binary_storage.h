/*
 * @Author       : baizs_work_pc_ubuntu zhongshan.bai@vitalchem.com
 * @Date         : 2024-12-15 16:01:11
 * @LastEditors  : baizs_work_pc_ubuntu_kioxia zhongshan.bai@vitalchem.com
 * @LastEditTime : 2025-08-07 17:48:31
 * @FilePath     : /calib_reflectivity/include/binary_storage.h
 * @Description  : 读写二进制文件（反射率表）
 * 
 * Copyright (c) 2024 by vitalchem, All Rights Reserved. 
 */
#ifndef BINARY_STORAGE_H
#define BINARY_STORAGE_H

#include <string>
#include <map>
#include <array>

namespace binary_storage {
    static const uint8_t _reflect_size = 5;

    // 将 map 保存为二进制文件
    bool saveMapToBinary(const std::string &filename, 
                         const std::map<uint16_t, std::map<float, std::array<float, _reflect_size>>> &map);

    // 从二进制文件读取 map
    bool loadMapFromBinary(const std::string &filename,
                           std::map<uint16_t, std::map<float, std::array<float, _reflect_size>>> &map);

    // 保存 map 到 CSV
    bool saveMapToCsv(const std::string &filename,
                      const std::map<uint16_t, std::map<float, std::array<float, _reflect_size>>> &map);
}

#endif // BINARY_STORAGE_H
