#ifndef YS_SCENE_YSC_FILE_H_
#define YS_SCENE_YSC_FILE_H_

#include "ys_lidar_type.h"
#include "ys_lidar_const.h"
#include <fstream>
#include <memory>
#include <vector>
#include <QMetaType>

namespace ys_scene {

 // 回放文件头
 struct ReplayFileHeader {
     char signature[16];      // 文件标识,固定为 "YOU_SEE"
     uint32_t version;   // 文件版本号
     uint64_t start_time; // 录制开始时间戳(毫秒)
     uint64_t end_time;   // 录制结束时间戳(毫秒)
     uint32_t frame_count;// 总帧数
     uint32_t magic_code; 
 };

 // 每一帧的帧头
 struct ReplayFrameHeader {
     uint64_t timestamp; // 该帧的时间戳(毫秒)
     uint32_t point_count;// 该帧包含的点数
     uint32_t data_size;  // 点数据的字节数
     uint32_t slots_count;  // slot数量
 };
 Q_DECLARE_METATYPE(ReplayFileHeader)


class YscFile {
public:
    ~YscFile();
    YscFile();
    YscFile(const std::string& filename);
    bool Open(const std::string& filename);
    void Close();
    std::shared_ptr<you_see::CloudFrame<you_see::CartesianPoint>> ReadFrame();
    std::shared_ptr<you_see::CloudFrame<you_see::CartesianPoint>> ReadLastFrame();
    void SaveFrame(std::shared_ptr<std::vector<you_see::CartesianPoint>> data, uint64_t timestamp, uint32_t slots_count);
    ReplayFileHeader GetYscFileHeader();
    uint64_t GetCurrentFrameNumber();
    void SeekToFrame(uint64_t frame_number);

private:
    void SeekToEnd();

private:
    std::ofstream file_write_;
    std::string file_write_name_;
    uint64_t start_time_;
    uint32_t frame_count_;
    uint32_t frame_number_;
    ReplayFileHeader file_header_;
    std::fstream file_read_;
    void* file_read_handle_;
    void* file_read_map_;
    void* file_read_data_;
    size_t file_read_size_;
    size_t current_read_pos_;
};

}//namespace ys_scene

#endif // YS_SCENE_YSC_FILE_H_