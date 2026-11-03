#include "cloud_viewer/stream/RangeStreamDecoder.h"
#include <lz4frame.h>
#include <algorithm>
#include <array>

namespace cloud_viewer {
namespace {
unsigned u16(const std::uint8_t* p) { return unsigned(p[0]) | unsigned(p[1]) << 8; }
unsigned u32(const std::uint8_t* p) { return u16(p) | u16(p+2) << 16; }
constexpr std::size_t kLimit = 16 * 1024 * 1024;
}
RangeStreamDecoder::RangeStreamDecoder(range6532::RangePipeline::FrameCallback callback)
    : pipeline_(range6532::EchoPolicy::PreferEcho0, true) {
    pipeline_.setFrameCallback(std::move(callback));
}
bool RangeStreamDecoder::ingest(const std::uint8_t* data, std::size_t size, std::string& error) {
    error.clear();
    // 旧 0x2e 协议必须严格校验 40 B 头，禁用启发式 payload 扫描。
    if (size >= 4 && data[0] == 0x5a && data[1] == 0xa5 && data[3] == 0x2e) {
        range6532::UdpPacketMetadata meta;
        if (!range6532::parseArmUdpHeader(data,size,meta,&error)) return false;
        return pipeline_.ingestRawPayload(data+40,size-40,meta.roiIndex,&error);
    }
    if (size < 14 || data[0] != 0xaa || data[1] != 1) {
        error = "not a RANGE datagram"; return false;
    }
    const unsigned fid = u16(data+2), seq = u16(data+4), flag = data[6], sub = data[7];
    const std::size_t header = flag == 0 ? 86 : 14;
    const unsigned len = u16(data+10);
    if (flag > 2 || size < header || u16(data+8) != size || !len || len > size-header || seq > 16383) {
        error = "invalid RANGE LZ4 header"; return false;
    }
    const unsigned key = (fid << 8) | sub;
    if (!partial_.count(key) && partial_.size() >= 8) {
        const auto old = std::min_element(partial_.begin(), partial_.end(),
            [](const auto& a, const auto& b) { return a.second.touch < b.second.touch; });
        partial_.erase(old);
    }
    auto& p = partial_[key]; p.touch = ++tick_;
    if (flag == 0) {
        if (seq != 0) { partial_.erase(key); error = "RANGE start sequence is not zero"; return false; }
        p.meta.assign(data+14, data+86);
    }
    if (!p.packets.count(seq)) {
        p.bytes += len;
        if (p.bytes > kLimit) { partial_.erase(key); error = "RANGE compressed frame too large"; return false; }
        p.packets[seq].assign(data+header, data+header+len);
    }
    if (flag == 2) p.end = static_cast<int>(seq);
    if (p.end < 0 || p.meta.empty() || p.packets.size() != static_cast<std::size_t>(p.end+1)) return true;
    for (int i=0; i<=p.end; ++i) if (!p.packets.count(i)) return true;
    const bool ok = finish(p, error); partial_.erase(key); return ok;
}
bool RangeStreamDecoder::finish(Partial& p, std::string& error) {
    // version 1: 16 B frame meta + 4 × 14 B ROI descriptor（共 72 B）。
    const auto* m = p.meta.data();
    const unsigned pixel = u16(m), count = u32(m+2), rois = m[8];
    if (pixel != range6532::kRecordSize || !count || count > kLimit/pixel || !rois || rois > 2) {
        error = "unsupported RANGE LZ4 metadata/layout"; return false;
    }
    std::size_t declared = 0;
    for (unsigned roi=0; roi<rois; ++roi) {
        const auto* d=m+16+roi*14;
        const unsigned push=u16(d+4), groups=d[6];
        if (!push || !groups || push > 768 || groups > 32 || u16(d) != 192 ||
            u16(d+2) != (roi ? 48 : 128)) { error="invalid RANGE ROI descriptor"; return false; }
        declared += std::size_t(push)*groups;
    }
    if (declared != count) { error="RANGE metadata pixel count mismatch"; return false; }
    std::vector<std::uint8_t> compressed;
    compressed.reserve(p.bytes);
    for (const auto& item:p.packets) compressed.insert(compressed.end(),item.second.begin(),item.second.end());
    std::vector<std::uint8_t> trans(std::size_t(count)*pixel);
    LZ4F_dctx* ctx=nullptr;
    auto status=LZ4F_createDecompressionContext(&ctx,LZ4F_VERSION);
    if (LZ4F_isError(status)) { error="LZ4 context failed"; return false; }
    std::size_t src=0,dst=0;
    do {
        std::size_t in=compressed.size()-src,out=trans.size()-dst;
        status=LZ4F_decompress(ctx,trans.data()+dst,&out,compressed.data()+src,&in,nullptr);
        src+=in; dst+=out;
        if (LZ4F_isError(status) || (!in && !out)) break;
    } while(status && src<compressed.size() && dst<trans.size());
    LZ4F_freeDecompressionContext(ctx);
    if (LZ4F_isError(status) || status != 0 || dst!=trans.size() || src!=compressed.size()) {
        error="truncated or invalid RANGE LZ4 stream"; return false;
    }
    // 每个 push 独立逆转置，不跨 ROI / group 混排。
    std::size_t offset=0;
    for (unsigned roi=0;roi<rois;++roi) {
        const auto* d=m+16+roi*14;
        const unsigned push=u16(d+4),groups=d[6];
        std::vector<std::uint8_t> records(std::size_t(push)*pixel);
        for (unsigned g=0;g<groups;++g) {
            for (unsigned i=0;i<push;++i)
                for (unsigned j=0;j<pixel;++j) records[i*pixel+j]=trans[offset+j*push+i];
            if (!pipeline_.ingestRawPayload(records.data(),records.size(),static_cast<std::uint8_t>(roi),&error)) return false;
            offset+=records.size();
        }
    }
    return true;
}
}
