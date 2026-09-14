#include "EYS3DSystem.h"
#include "devices/CameraDevice.h"
#include "devices/Pipeline.h"
#include "video/Frame.h"

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <unistd.h>

namespace {
constexpr uint32_t kMagic = 0x48323031;

struct __attribute__((packed)) Header {
    uint32_t magic;
    uint32_t width;
    uint32_t height;
    uint32_t payload_bytes;
    uint64_t timestamp_ns;
};

bool write_all(int fd, const void* data, size_t size) {
    const auto* cursor = static_cast<const uint8_t*>(data);
    while (size > 0) {
        const ssize_t written = write(fd, cursor, size);
        if (written < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        cursor += written;
        size -= static_cast<size_t>(written);
    }
    return true;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    const int output_fd = std::atoi(argv[1]);
    if (output_fd < 0) return 2;

    using namespace libeYs3D;
    auto system = std::make_shared<EYS3DSystem>(EYS3DSystem::COLOR_BYTE_ORDER::COLOR_BGR24);
    if (system->getCameraDeviceCount() != 1) return 3;
    auto device = system->getCameraDevice(0);
    const auto info = device->getCameraDeviceInfo();
    if (info.devInfo.wVID != 0x3438 || info.devInfo.wPID != 0x0173) return 4;

    auto pipeline = device->initStream(
        video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2, 0, 0, 15,
        video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_11_BITS,
        640, 460, DEPTH_IMG_COLORFUL_TRANSFER, IMAGE_SN_SYNC, 0);
    if (!pipeline) return 5;
    device->enableStream();

    video::Frame frame;
    for (;;) {
        if (pipeline->waitForDepthFrame(&frame, 1000) != devices::Pipeline::OK) continue;
        const size_t pixels = static_cast<size_t>(frame.width) * frame.height;
        if (frame.width <= 0 || frame.height <= 0 || frame.zdDepthVec.size() < pixels) continue;
        const Header header{kMagic, static_cast<uint32_t>(frame.width),
                            static_cast<uint32_t>(frame.height),
                            static_cast<uint32_t>(pixels * sizeof(uint16_t)),
                            static_cast<uint64_t>(frame.tsUs) * 1000};
        if (!write_all(output_fd, &header, sizeof(header)) ||
            !write_all(output_fd, frame.zdDepthVec.data(), header.payload_bytes)) break;
    }

    device->closeStream();
    close(output_fd);
    return 0;
}
