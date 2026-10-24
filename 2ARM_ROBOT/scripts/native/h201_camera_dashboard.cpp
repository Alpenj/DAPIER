#include "EYS3DSystem.h"
#include "devices/CameraDevice.h"
#include "devices/Pipeline.h"
#include "video/Frame.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {
constexpr int kWidth = 320;
constexpr int kHeight = 240;

cv::Mat panel(const cv::Mat& input, const std::string& label, const std::string& source) {
    cv::Mat out(kHeight, kWidth, CV_8UC3, cv::Scalar(30, 30, 30));
    if (!input.empty()) {
        cv::resize(input, out, out.size(), 0, 0, cv::INTER_AREA);
    } else {
        cv::putText(out, "DISCONNECTED / NO FRAME", {26, 150},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, {80, 80, 255}, 2, cv::LINE_AA);
    }
    cv::rectangle(out, {0, 0}, {kWidth, 38}, {20, 20, 20}, cv::FILLED);
    cv::putText(out, label, {12, 26}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                {255, 255, 255}, 2, cv::LINE_AA);
    cv::putText(out, source, {12, kHeight - 12}, cv::FONT_HERSHEY_SIMPLEX,
                0.4, {220, 220, 220}, 1, cv::LINE_AA);
    return out;
}

cv::VideoCapture open_wrist(const std::string& path) {
    cv::VideoCapture capture(path, cv::CAP_V4L2);
    if (!capture.isOpened()) return capture;
    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    capture.set(cv::CAP_PROP_FRAME_WIDTH, kWidth);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, kHeight);
    capture.set(cv::CAP_PROP_FPS, 30);
    capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
    return capture;
}
}  // namespace

int main(int argc, char** argv) {
    const std::string left_path = argc > 1 ? argv[1] : "/dev/dapier/left_wrist_rgb";
    const std::string right_path = argc > 2 ? argv[2] : "/dev/dapier/right_wrist_rgb";

    using namespace libeYs3D;
    auto system = std::make_shared<EYS3DSystem>(EYS3DSystem::COLOR_BYTE_ORDER::COLOR_BGR24);
    if (system->getCameraDeviceCount() != 1) {
        std::cerr << "expected exactly one eYs3D H201 camera\n";
        return 2;
    }
    auto device = system->getCameraDevice(0);
    const auto info = device->getCameraDeviceInfo();
    if (info.devInfo.wVID != 0x3438 || info.devInfo.wPID != 0x0173) {
        std::cerr << "expected HP-ASC-H201 (3438:0173)\n";
        return 3;
    }

    auto depth_pipeline = device->initStream(
        video::COLOR_RAW_DATA_TYPE::COLOR_RAW_DATA_YUY2, 0, 0, 15,
        video::DEPTH_RAW_DATA_TYPE::DEPTH_RAW_DATA_SCALE_DOWN_11_BITS,
        640, 460, DEPTH_IMG_COLORFUL_TRANSFER, IMAGE_SN_SYNC, 0);
    if (!depth_pipeline) {
        std::cerr << "H201 depth stream initialization failed\n";
        return 4;
    }
    device->enableStream();

    auto left = open_wrist(left_path);
    auto right = open_wrist(right_path);
    cv::namedWindow("DAPIER cameras: LEFT | H201 DEPTH | RIGHT", cv::WINDOW_NORMAL);

    video::Frame depth;
    for (;;) {
        cv::Mat left_frame, right_frame, depth_frame;
        if (left.isOpened()) left.read(left_frame);
        if (right.isOpened()) right.read(right_frame);
        if (depth_pipeline->waitForDepthFrame(&depth, 100) == devices::Pipeline::OK &&
            depth.width > 0 && depth.height > 0 &&
            depth.rgbVec.size() >= static_cast<size_t>(depth.width * depth.height * 3)) {
            cv::Mat sdk_rgb(depth.height, depth.width, CV_8UC3, depth.rgbVec.data());
            cv::cvtColor(sdk_rgb, depth_frame, cv::COLOR_RGB2BGR);
        }

        cv::Mat dashboard;
        cv::hconcat(std::vector<cv::Mat>{
                        panel(left_frame, "LEFT WRIST RGB", left_path),
                        panel(depth_frame, "TOP H201 DEPTH", "official eYs3D SDK / uint16 mm"),
                        panel(right_frame, "RIGHT WRIST RGB", right_path)},
                    dashboard);
        cv::imshow("DAPIER cameras: LEFT | H201 DEPTH | RIGHT", dashboard);
        const int key = cv::waitKey(1) & 0xff;
        if (key == 'q' || key == 27) break;
    }

    left.release();
    right.release();
    device->closeStream();
    cv::destroyAllWindows();
    return 0;
}
