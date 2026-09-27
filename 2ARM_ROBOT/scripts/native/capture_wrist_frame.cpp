// One attended wrist RGB frame with a V4L2 monotonic frame timestamp. No motor I/O.
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

using json = nlohmann::json;
namespace {
std::string sha256(const void* data, std::size_t size) {
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(static_cast<const unsigned char*>(data),size,digest);
  std::ostringstream out;
  for (unsigned char byte:digest) out<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<unsigned>(byte);
  return out.str();
}
std::int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::int64_t frame_timestamp(const v4l2_buffer& b, std::int64_t received) {
  if ((b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC ||
      b.timestamp.tv_sec < 0 || b.timestamp.tv_usec < 0 || b.timestamp.tv_usec >= 1000000)
    throw std::runtime_error("camera did not provide a valid monotonic frame timestamp");
  const auto stamp = static_cast<std::int64_t>(b.timestamp.tv_sec)*1000000000LL + b.timestamp.tv_usec*1000LL;
  if (stamp <= 0 || stamp > received || received-stamp > 100000000)
    throw std::runtime_error("camera frame is future or older than 100ms; no timestamp replacement");
  return stamp;
}
cv::Mat decode(const void* data, std::size_t count, unsigned stride) {
  if (stride < 640 || count < static_cast<std::size_t>(239)*stride+640)
    throw std::runtime_error("short YUYV320x240 frame");
  cv::Mat bgr;
  cv::cvtColor(cv::Mat(240,320,CV_8UC2,const_cast<void*>(data),stride),bgr,cv::COLOR_YUV2BGR_YUY2);
  return bgr;
}
void save(const std::string& path, const void* data, std::size_t size) {
  int fd = open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600);
  if (fd < 0) throw std::runtime_error("exclusive output required: "+path);
  std::size_t offset=0;
  while (offset<size) {
    const auto n=write(fd,static_cast<const char*>(data)+offset,size-offset);
    if (n<=0) { close(fd); throw std::runtime_error("output write failed"); }
    offset+=static_cast<std::size_t>(n);
  }
  close(fd);
}
struct Camera {
  int fd{-1}; bool streaming{false};
  std::vector<std::pair<void*,std::size_t>> buffers;
  ~Camera() {
    if (streaming) { int type=V4L2_BUF_TYPE_VIDEO_CAPTURE; ioctl(fd,VIDIOC_STREAMOFF,&type); }
    for (const auto& b:buffers) munmap(b.first,b.second);
    if (fd>=0) close(fd);
  }
  void call(unsigned long op,void* value) {
    if (ioctl(fd,op,value)<0) throw std::runtime_error("V4L2 ioctl failed: "+std::to_string(op)+" errno="+std::to_string(errno));
  }
};
void self_test() {
  if (sha256("abc",3)!="ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")
    throw std::runtime_error("frame digest failed");
  v4l2_buffer b{}; b.flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
  b.timestamp.tv_sec=1; b.timestamp.tv_usec=123;
  if (frame_timestamp(b,1000200000)!=1000123000) throw std::runtime_error("timestamp conversion failed");
  for (auto flags:{0U,static_cast<unsigned>(V4L2_BUF_FLAG_TIMESTAMP_COPY)}) {
    b.flags=flags; bool rejected=false;
    try { frame_timestamp(b,1000200000); } catch (const std::runtime_error&) { rejected=true; }
    if (!rejected) throw std::runtime_error("unknown timestamp accepted");
  }
  b.flags=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
  for (auto received:{1000122999LL,1100123001LL}) {
    bool rejected=false;
    try { frame_timestamp(b,received); } catch (const std::runtime_error&) { rejected=true; }
    if (!rejected) throw std::runtime_error("future/stale timestamp accepted");
  }
  std::vector<unsigned char> bytes(320*240*2,128);
  auto image=decode(bytes.data(),bytes.size(),640);
  if (image.rows!=240 || image.cols!=320 || image.type()!=CV_8UC3) throw std::runtime_error("decode failed");
  bool rejected=false;
  try { decode(bytes.data(),bytes.size()-1,640); } catch (const std::runtime_error&) { rejected=true; }
  if (!rejected) throw std::runtime_error("short frame accepted");
  std::cout<<"PASS: timestamp/encoding checks; no device opened\n";
}
}

int main(int argc,char** argv) {
  json result={{"schema_version","dapier.wrist-frame.v1"},{"hardware_opened",false},
               {"frame_acquired",false},{"normal_stream_close",false},{"motion_authorized",false}};
  std::string output;
  try {
    if (argc==2 && std::string(argv[1])=="--self-test") { self_test(); return 0; }
    if (argc!=6 || std::string(argv[1])!="VISIBLE_LEFT_WRIST_FRAME_ONCE" || !isatty(0) || !isatty(1))
      throw std::runtime_error("usage in attended TTY: capture_wrist_frame VISIBLE_LEFT_WRIST_FRAME_ONCE DEVICE EXPECTED_BUS_INFO OUTPUT_PREFIX --operator-present");
    const std::string device=argv[2], bus_info=argv[3]; output=argv[4];
    if (device!="/dev/dapier/left_wrist_rgb" || bus_info.empty() || std::string(argv[5])!="--operator-present")
      throw std::runtime_error("explicit left wrist role, bus identity and attendance required");
    std::ifstream boot_file("/proc/sys/kernel/random/boot_id");
    std::string boot_id;
    if (!(boot_file>>boot_id) || boot_id.size()!=36) throw std::runtime_error("host boot identity unavailable");
    result["host_boot_id"]=boot_id;
    // Reserve the run before opening a device. Failures also retain this evidence.
    save(output+".reserved", "wrist acquisition\n",18);
    if (access((output+".png").c_str(),F_OK)==0 || access((output+".json").c_str(),F_OK)==0)
      throw std::runtime_error("output prefix already used");
    {
      Camera camera;
      camera.fd=open(device.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);
      if (camera.fd<0) throw std::runtime_error("cannot open approved wrist device");
      result["hardware_opened"]=true;
      v4l2_capability cap{}; camera.call(VIDIOC_QUERYCAP,&cap);
      if (std::string(reinterpret_cast<char*>(cap.bus_info),strnlen(reinterpret_cast<char*>(cap.bus_info),sizeof(cap.bus_info)))!=bus_info)
        throw std::runtime_error("wrist camera bus identity mismatch");
      const auto caps=(cap.capabilities&V4L2_CAP_DEVICE_CAPS)?cap.device_caps:cap.capabilities;
      if (!(caps&V4L2_CAP_VIDEO_CAPTURE) || !(caps&V4L2_CAP_STREAMING)) throw std::runtime_error("capture/streaming capability required");
      v4l2_format format{}; format.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
      format.fmt.pix.width=320; format.fmt.pix.height=240; format.fmt.pix.pixelformat=V4L2_PIX_FMT_YUYV;
      format.fmt.pix.field=V4L2_FIELD_NONE; camera.call(VIDIOC_S_FMT,&format);
      if (format.fmt.pix.width!=320 || format.fmt.pix.height!=240 || format.fmt.pix.pixelformat!=V4L2_PIX_FMT_YUYV)
        throw std::runtime_error("requested320x240 YUYV mode unavailable; no rescaling");
      v4l2_streamparm rate{}; rate.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
      rate.parm.capture.timeperframe.numerator=1; rate.parm.capture.timeperframe.denominator=30;
      camera.call(VIDIOC_S_PARM,&rate);
      result["timeperframe"]={rate.parm.capture.timeperframe.numerator,rate.parm.capture.timeperframe.denominator};
      v4l2_requestbuffers request{}; request.count=2; request.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; request.memory=V4L2_MEMORY_MMAP;
      camera.call(VIDIOC_REQBUFS,&request);
      if (request.count<1 || request.count>8) throw std::runtime_error("unexpected camera buffer count");
      for (unsigned i=0;i<request.count;++i) {
        v4l2_buffer b{}; b.type=request.type; b.memory=request.memory; b.index=i; camera.call(VIDIOC_QUERYBUF,&b);
        void* memory=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,camera.fd,b.m.offset);
        if (memory==MAP_FAILED) throw std::runtime_error("camera mmap failed");
        camera.buffers.emplace_back(memory,b.length); camera.call(VIDIOC_QBUF,&b);
      }
      int type=V4L2_BUF_TYPE_VIDEO_CAPTURE; camera.call(VIDIOC_STREAMON,&type); camera.streaming=true;
      const auto deadline=now_ns()+3000000000LL;
      for (int frame=0;frame<4;) {
        if (now_ns()>deadline) throw std::runtime_error("wrist acquisition3s deadline exceeded");
        pollfd ready{camera.fd,POLLIN,0}; int status=poll(&ready,1,100);
        if (status<0) throw std::runtime_error("wrist poll failed");
        if (!status) continue;
        if (ready.revents&(POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("wrist stream disconnected");
        v4l2_buffer b{}; b.type=request.type; b.memory=request.memory; camera.call(VIDIOC_DQBUF,&b);
        if (b.index>=camera.buffers.size() || b.bytesused>camera.buffers[b.index].second || (b.flags&V4L2_BUF_FLAG_ERROR))
          throw std::runtime_error("invalid/error camera buffer");
        const auto received=now_ns(); const auto stamp=frame_timestamp(b,received);
        if (++frame==4) {
          auto image=decode(camera.buffers[b.index].first,b.bytesused,format.fmt.pix.bytesperline);
          std::vector<unsigned char> png;
          if (!cv::imencode(".png",image,png)) throw std::runtime_error("PNG encoding failed");
          save(output+".png",png.data(),png.size());
          result.update({{"frame_acquired",true},{"side","left"},{"device",device},{"bus_info",bus_info},
              {"clock","host_monotonic_ns"},{"timestamp_ns",stamp},{"received_monotonic_ns",received},
              {"timestamp_source_flags",b.flags&V4L2_BUF_FLAG_TSTAMP_SRC_MASK},
              {"timestamp_semantics","V4L2 frame timestamp; source flags preserve SOE/EOF distinction"},
              {"sequence",b.sequence},{"frames_dequeued",frame},{"width",320},{"height",240},
              {"encoding","bgr8 decoded from YUYV"},{"frame_path",output+".png"},
              {"frame_sha256",sha256(png.data(),png.size())}});
        }
        camera.call(VIDIOC_QBUF,&b);
      }
      camera.call(VIDIOC_STREAMOFF,&type); camera.streaming=false;
      result["normal_stream_close"]=true;
    }
    auto text=result.dump(2); save(output+".json",text.data(),text.size());
    std::cout<<result.dump()<<'\n'; return 0;
  } catch(const std::exception& error) {
    result["error"]=error.what();
    if (!output.empty()) { try { auto text=result.dump(2); save(output+".json",text.data(),text.size()); } catch(...) {} }
    std::cerr<<result.dump()<<'\n'; return 1;
  }
}
