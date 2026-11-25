#include "ysc_file.h"
#include "log/logging.h"
#include "tiny_hut/common_toolset.h" 

#include <windows.h>

namespace ys_scene {
YscFile::YscFile():
start_time_(0),
frame_count_(0),
frame_number_(0),
file_read_handle_(nullptr),
file_read_map_(nullptr),
file_read_data_(nullptr)
{
}

YscFile::YscFile(const std::string& filename) :
    file_write_(filename, std::ios::binary),
    file_write_name_(filename),
    start_time_(0),
    frame_count_(0) 
{
    if (file_write_.is_open()) {
        ReplayFileHeader header = { "YOU_SEE", 1, 0, 0, 0, 0 };
        file_write_.write(reinterpret_cast<char*>(&header), sizeof(header));
        //LOG_INFO("create file: {}", filename);
    }
    else {
        LOG_ERROR("Failed to open file for writing: {}", filename);
    }
}

bool YscFile::Open(const std::string& filename)
{
    file_read_handle_ = CreateFileA(filename.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file_read_handle_ == INVALID_HANDLE_VALUE) {
        return false;
    }

    file_read_size_ = GetFileSize(file_read_handle_, nullptr);
    if (file_read_size_ == INVALID_FILE_SIZE) {
        CloseHandle(file_read_handle_);
        return false;
    }

    file_read_map_ = CreateFileMappingA(file_read_handle_, nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (file_read_map_ == nullptr) {
        CloseHandle(file_read_handle_);
        return false;
    }

    file_read_data_ = MapViewOfFile(file_read_map_, FILE_MAP_READ, 0, 0, 0);
    if (file_read_data_ == nullptr) {
        CloseHandle(file_read_map_);
        CloseHandle(file_read_handle_);
        return false;
    }

    // 读取 .ysc 文件头
    memcpy(&file_header_, file_read_data_, sizeof(ReplayFileHeader));
    current_read_pos_ = sizeof(ReplayFileHeader);
    frame_count_ = file_header_.frame_count;
    return true;
}

void YscFile::Close()
{
    if (file_read_data_) {
        UnmapViewOfFile(file_read_data_);
        file_read_data_ = nullptr;
    }
    if (file_read_map_) {
        CloseHandle(file_read_map_);
        file_read_map_ = nullptr;
    }
    if (file_read_handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(file_read_handle_);
        file_read_handle_ = nullptr;
    }
}

std::shared_ptr<CloudFrame<CartesianPoint>> YscFile::ReadFrame()
{
    if (!file_read_data_ || current_read_pos_ >= file_read_size_) {
        return nullptr;
    }

    ReplayFrameHeader* frameHeader = reinterpret_cast<ReplayFrameHeader*>((char*)file_read_data_ + current_read_pos_);
    current_read_pos_ += sizeof(ReplayFrameHeader);

    if (current_read_pos_ + frameHeader->data_size > file_read_size_) {
        return nullptr;
    }

    std::shared_ptr<CloudFrame<CartesianPoint>> frame = std::make_shared<CloudFrame<CartesianPoint>>(frame_number_, frameHeader->point_count);
    frame_number_++;
    frame->data.resize(frameHeader->point_count);
    frame->ms_timestamp = frameHeader->timestamp;

    memcpy(frame->data.data(), (char*)file_read_data_ + current_read_pos_, frameHeader->data_size);
    current_read_pos_ += frameHeader->data_size;
    frame->slot_counts.resize(frameHeader->slots_count);
    return frame;
}

std::shared_ptr<CloudFrame<CartesianPoint>> YscFile::ReadLastFrame()
{
    SeekToEnd();
    std::shared_ptr<CloudFrame<CartesianPoint>> last_frame = ReadFrame();
    Close();
    return last_frame;
}

YscFile::~YscFile()
{
    try {
        if (file_write_.is_open()) {
            file_write_.seekp(0);
            ReplayFileHeader header = { "YOU_SEE", 1, start_time_, toolset::GetTimeStamp(), frame_count_, 0 };
            file_write_.write(reinterpret_cast<char*>(&header), sizeof(header));
            file_write_.close();
            //LOG_INFO("file:{} finished, frame count:{}", file_write_name_, frame_count_);
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception during YscFile destructor: {}", e.what());
    }
}

void YscFile::SaveFrame(std::shared_ptr<std::vector<CartesianPoint>> data, uint64_t timestamp, uint32_t slots_count)
{
    if (start_time_ == 0) {
        start_time_ = toolset::GetTimeStamp();
    }

    // 写入帧头
    ReplayFrameHeader frameHeader;
    frameHeader.timestamp = timestamp;
    frameHeader.point_count = data->size();
    frameHeader.data_size = data->size() * sizeof(CartesianPoint);
    frameHeader.slots_count = slots_count;
    if (!file_write_) {
        LOG_ERROR("Failed to write frame header.");
        return;
    }
    file_write_.write((char*)&frameHeader, sizeof(frameHeader));

    if (!file_write_) {
        LOG_ERROR("Failed to write frame data.");
        return;
    }
    // 写入点数据
    file_write_.write((char*)(data->data()), frameHeader.data_size);
    frame_count_++;
}

ReplayFileHeader YscFile::GetYscFileHeader()
{
    return file_header_;
}

uint64_t YscFile::GetCurrentFrameNumber()
{
    return frame_number_;
}

void YscFile::SeekToFrame(uint64_t frame_number)
{
    if (!file_read_data_) {
        return;
    }

    current_read_pos_ = sizeof(ReplayFileHeader);
    uint64_t current_frame = 0;

    while (current_read_pos_ < file_read_size_ && current_frame < frame_number) {
        ReplayFrameHeader* frameHeader = reinterpret_cast<ReplayFrameHeader*>((char*)file_read_data_ + current_read_pos_);
        current_read_pos_ += sizeof(ReplayFrameHeader) + frameHeader->data_size;
        current_frame++;
    }

    frame_number_ = current_frame;
}

void YscFile::SeekToEnd()
{
    if (!file_read_data_) {
        return;
    }

    // 从文件尾部开始查找最后一帧
    current_read_pos_ = file_read_size_;
    while (current_read_pos_ > sizeof(ReplayFileHeader)) {
        current_read_pos_ -= sizeof(ReplayFrameHeader);
        ReplayFrameHeader* frameHeader = reinterpret_cast<ReplayFrameHeader*>((char*)file_read_data_ + current_read_pos_);
        if (current_read_pos_ - frameHeader->data_size >= sizeof(ReplayFileHeader)) {
            current_read_pos_ -= frameHeader->data_size;
            break;
        }
    }
}

}; // namespace ys_scene