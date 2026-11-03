#include "cloud_viewer/stream/RangeStreamDecoder.h"
#include "cloud_viewer/stream/UdpRadarStream.h"
#include "cloud_viewer/preprocess/OnlineCalibration.h"
#include <lz4frame.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <atomic>
#ifndef _WIN32
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
using namespace cloud_viewer;
using Bytes=std::vector<std::uint8_t>;
static void check(bool ok,const char* why){ if(!ok) throw std::runtime_error(why); }
static void w16(Bytes& b,std::size_t o,unsigned v){ b[o]=v&255;b[o+1]=(v>>8)&255; }
static void w32(Bytes& b,std::size_t o,unsigned v){w16(b,o,v);w16(b,o+2,v>>16);}
static Bytes records(unsigned roi,unsigned frame,unsigned sub=0) {
    const unsigned pixels=roi?48:192,groups=roi?12:32;
    Bytes b(groups*4*pixels*278);std::size_t off=0;
    for(unsigned g=0;g<groups;++g)for(unsigned slot=0;slot<4;++slot)for(unsigned p=0;p<pixels;++p) {
        w16(b,off+2,frame);w16(b,off+6,g+(roi?4:0));w16(b,off+8,slot);
        // ROI1 使用 0..3 子帧编号。
        b[off+15]=sub;w16(b,off+18,p);b[off+0x22]=1;b[off+0x23]=1;
        b[off+0x3a+2]=100;w32(b,off+0x3a+5,51200);w32(b,off+0x3a+11,50000);
        b[off+0x4d+2]=255;off+=278;
    }
    return b;
}
static Bytes rawPacket(const Bytes& records,unsigned roi,std::size_t first,std::size_t count) {
    Bytes b(40+count*278);w16(b,0,0xa55a);b[2]=2;b[3]=0x2e;b[6]=2;
    w16(b,11,count);b[13]=5;w16(b,16,192);w16(b,18,roi?48:128);
    b[24]=roi;w16(b,25,278);w16(b,27,40);w16(b,29,count*278);
    std::copy(records.begin()+first*278,records.begin()+(first+count)*278,b.begin()+40);return b;
}
static std::vector<Bytes> lz4Packets(const Bytes& raw) {
    constexpr unsigned n=768,pixel=278;
    Bytes trans(raw.size());
    for(std::size_t off=0;off<raw.size();off+=n*pixel)
        for(unsigned i=0;i<n;++i)for(unsigned j=0;j<pixel;++j)trans[off+j*n+i]=raw[off+i*pixel+j];
    Bytes compressed(LZ4F_compressFrameBound(trans.size(),nullptr));
    const auto size=LZ4F_compressFrame(compressed.data(),compressed.size(),trans.data(),trans.size(),nullptr);
    check(!LZ4F_isError(size),"compression");compressed.resize(size);
    std::vector<Bytes> out;std::size_t pos=0;
    while(pos<size) {
        const auto header=pos?14:86;
        const auto count=std::min<std::size_t>(size-pos,1400-header);
        Bytes b(1400);b[0]=0xaa;b[1]=1;w16(b,2,5);w16(b,4,out.size());
        b[6]=pos==0?0:pos+count==size?2:1;w16(b,8,1400);w16(b,10,count);
        if(!pos) {
            w16(b,14,278);w32(b,16,24576);b[20]=32;b[21]=32;b[22]=1;b[23]=1;
            w16(b,30,192);w16(b,32,128);w16(b,34,768);b[36]=32;
        }
        std::copy(compressed.begin()+pos,compressed.begin()+pos+count,b.begin()+header);
        out.push_back(std::move(b));pos+=count;
    }
    return out;
}
static void decoderTests() {
    unsigned frames=0;std::string error;
    RangeStreamDecoder decoder([&](range6532::PointCloudFrame&& f){
        ++frames;check(f.validPointCount==f.rows*f.cols,"all RANGE points valid");
        check(f.points.front().distanceMetres==2.0f,"RANGE distance scale");
    });
    auto raw=records(0,5);
    for(std::size_t i=0;i<24576;i+=4) {auto b=rawPacket(raw,0,i,4);check(decoder.ingest(b.data(),b.size(),error),error.c_str());}
    check(frames==1,"raw ROI0 assembled");
    auto packets=lz4Packets(raw);
    // 先发末包，逆序、重复：只有所有序号齐全才发帧。
    std::reverse(packets.begin(),packets.end());
    check(decoder.ingest(packets.front().data(),packets.front().size(),error),error.c_str());
    for(auto& b:packets)check(decoder.ingest(b.data(),b.size(),error),error.c_str());
    check(frames==2,"reordered LZ4 ROI0 assembled");
    for(auto& b:packets)check(decoder.ingest(b.data(),b.size(),error),error.c_str());
    // 完整重放是一帧新输入，pipeline 自身允许重放；不检验帧号全局去重。
    check(frames==3,"complete replay assembled");
    const auto before=frames;packets.pop_back();
    for(auto& b:packets)check(decoder.ingest(b.data(),b.size(),error),error.c_str());
    check(frames==before,"missing LZ4 start does not emit");
    Bytes bad(10);check(!decoder.ingest(bad.data(),bad.size(),error),"short packet rejected");
    // ROI1 四个子帧：物理行号仍为 16..63。
    for(unsigned sub=0;sub<4;++sub){auto one=records(1,8+sub,sub);
        for(std::size_t i=0;i<one.size()/278;i+=4){auto b=rawPacket(one,1,i,4);check(decoder.ingest(b.data(),b.size(),error),error.c_str());}}
    check(frames==before+1,"ROI1 four subframes assembled");
    auto malformed=rawPacket(raw,0,0,4);w16(malformed,25,1);
    check(!decoder.ingest(malformed.data(),malformed.size(),error),"bad pixel length rejected");
    RangeStreamDecoder truncated([](range6532::PointCloudFrame&&){throw std::runtime_error("truncated stream emitted");});
    auto cut=lz4Packets(raw);
    w16(cut.back(),10,((unsigned(cut.back()[11])<<8)|cut.back()[10])-1);
    bool ok=true;
    for(auto& b:cut)ok=truncated.ingest(b.data(),b.size(),error);
    check(!ok,"truncated LZ4 rejected");
    std::cout<<"RANGE raw/LZ4/reorder/duplicate/missing/ROI1/malformed/truncated: PASS\n";
}
static void calibrationTests() {
    algo_lidar::PointData p{};p.is_valid=true;p.r=2;p.x=2;p.pixel_idx=0;p.slot_idx=0;p.list_pulse.emplace_back(0,0,100);
    FrameMetadata meta;meta.roiId=0;meta.calibrationState="raw_range";
    auto raw=makeFrame({{p}},meta);
    algo_lidar::Range6532LegacyCalibrationBin table;
    auto& roi=table.roi[0];roi.rows=1;roi.cols=1;roi.intensity_reference={100};roi.distance_correction_cm={-10};roi.tail3_raw={0,0,5};
    auto b=calibrateOnlineRange(raw,table,OnlineCalibrationMode::TableB);
    check(std::abs(b->points[0][0].r-1.9f)<1e-5,"TableB applied");
    auto t=calibrateOnlineRange(raw,table,OnlineCalibrationMode::Tail3LnFixed);
    check(std::abs(t->points[0][0].r-1.95f)<1e-5,"tail3 applied");
    check(raw->points[0][0].r==2,"raw immutable");
    check(calibrateOnlineRange(raw,table,OnlineCalibrationMode::Off)==raw,"off returns original");
    check(calibrateOnlineRange(t,table,OnlineCalibrationMode::Tail3LnFixed)==t,"no repeated compensation");
    meta.calibrationState="device_unknown";
    auto cloud=makeFrame({{p}},meta);
    check(calibrateOnlineRange(cloud,table,OnlineCalibrationMode::Tail3LnFixed)==cloud,"Cloud never double compensated");
    std::cout<<"Calibration off/TableB/Tail3/raw-immutable/Cloud-passthrough: PASS\n";
}
#ifndef _WIN32
static void cloudTests() {
    const int sender=socket(AF_INET,SOCK_DGRAM,0);check(sender>=0,"UDP sender");
    sockaddr_in source{};source.sin_family=AF_INET;source.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    check(bind(sender,reinterpret_cast<sockaddr*>(&source),sizeof(source))==0,"UDP sender bind");
    socklen_t len=sizeof(source);getsockname(sender,reinterpret_cast<sockaddr*>(&source),&len);
    UdpRadarConfig config;config.radarIp="127.0.0.1";config.radarPort=ntohs(source.sin_port);config.localPort=0;
    UdpRadarStream stream;std::atomic<unsigned> frames{0};std::atomic<bool> good{true};std::string error;
    check(stream.open(config,[&](algo_lidar::PointMatrix&& matrix,std::uint64_t,int roi){
        if(matrix.size()!=128 || matrix[0].size()!=192 || roi!=0 || matrix[0][0].r!=2.0f) good=false;
        ++frames;
    },error),error.c_str());
    sockaddr_in dest{};dest.sin_family=AF_INET;dest.sin_addr.s_addr=htonl(INADDR_LOOPBACK);dest.sin_port=htons(stream.config().localPort);
    std::vector<Bytes> packets;
    for(unsigned first=0,seq=0;first<24576;first+=79,++seq) {
        unsigned count=std::min(79u,24576-first);Bytes b(40+18*count);
        w16(b,0,0xa55a);b[2]=2;b[3]=0x21;b[6]=2;w16(b,7,seq);w16(b,9,7);w16(b,11,count);
        b[13]=2;b[15]=first+count==24576?2:first?1:0;w16(b,16,192);w16(b,18,128);
        w16(b,25,18);w16(b,27,40);w16(b,29,count*18);
        for(unsigned i=0;i<count;++i){auto o=40+i*18;w16(b,o+4,(first+i)%192);w16(b,o+6,(first+i)/192);w16(b,o+8,200);w16(b,o+10,200);}
        packets.push_back(std::move(b));
    }
    auto send=[&](const Bytes& b){sendto(sender,b.data(),b.size(),0,reinterpret_cast<sockaddr*>(&dest),sizeof(dest));std::this_thread::sleep_for(std::chrono::microseconds(250));};
    // 帧尾先到不触发残帧输出；重复首包不重复计数。
    send(packets.back());std::this_thread::sleep_for(std::chrono::milliseconds(10));check(frames==0,"end-first not partial");
    send(packets.front());send(packets.front());
    for(std::size_t i=1;i+1<packets.size();++i)send(packets[i]);
    for(int i=0;i<200&&frames==0;++i)std::this_thread::sleep_for(std::chrono::milliseconds(10));
    stream.stop();close(sender);
    check(frames==1&&good,"complete Cloud via loopback, correct scale and ROI");
    std::cout<<"Cloud UDP loopback/end-first/duplicate/complete-frame: PASS\n";
}
static void controlTests() {
    const int server=socket(AF_INET,SOCK_STREAM,0);check(server>=0,"control socket");
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    check(bind(server,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"control bind");
    socklen_t len=sizeof(address);getsockname(server,reinterpret_cast<sockaddr*>(&address),&len);listen(server,1);
    bool correct=false;
    std::thread worker([&]{
        const int client=accept(server,nullptr,nullptr);if(client<0)return;
        unsigned char request[20]{};std::size_t count=0;
        while(count<20){auto n=recv(client,request+count,20-count,0);if(n<=0)break;count+=n;}
        correct=count==20 && request[16]==0x2a && request[8]==2;
        unsigned char ack[16]={0x88,0x77,0,0,0,0,0xaa,0x99,2,0,0,0,0,0,0,0};
        for(auto c:ack)send(client,&c,1,MSG_NOSIGNAL);close(client);
    });
    const auto error=setRadarOutputMode("127.0.0.1",ntohs(address.sin_port),RadarOutputMode::Range);
    worker.join();close(server);check(error.empty(),error.c_str());check(correct,"mode command wire values");
    std::cout<<"TCP mode command + fragmented ACK: PASS\n";
}
#endif
int main(){try{decoderTests();calibrationTests();
#ifndef _WIN32
cloudTests();controlTests();
#endif
return 0;}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
