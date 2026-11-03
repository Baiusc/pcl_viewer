#pragma once
#include "range6532_pipeline.h"
#include <map>
#include <vector>

namespace cloud_viewer {
// 接收线程独占；完整组帧后才交给 UI，模式切换时销毁并重新创建。
class RangeStreamDecoder {
public:
    explicit RangeStreamDecoder(range6532::RangePipeline::FrameCallback callback);
    bool ingest(const std::uint8_t* data, std::size_t size, std::string& error);
private:
    struct Partial {
        std::vector<std::uint8_t> meta;
        std::map<unsigned, std::vector<std::uint8_t>> packets;
        int end = -1;
        std::size_t bytes = 0;
        std::uint64_t touch = 0;
    };
    bool finish(Partial& partial, std::string& error);
    range6532::RangePipeline pipeline_;
    std::map<unsigned, Partial> partial_;
    std::uint64_t tick_ = 0;
};
}
