/*************************************************************************************************************************
 * Copyright 2023 Xidian114
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *************************************************************************************************************************/
#pragma once

#include <opencv2/opencv.hpp>

#include <string>
#include <atomic>
#include <vector>
#include <cstdint>

#include "threadsafe_queue.h"

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/error.h>
#include <libavutil/mem.h>
#include <libavutil/time.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
}

static uint8_t startCode[4] = {0x00, 0x00, 0x00, 0x01};
static uint8_t NRICode[1] = {0x06};
static uint8_t PAYLOADCode[1] = {0x05};
static uint8_t UUID[16] = {0xdc, 0x45, 0xe9, 0xbd, 0xe6,
                           0xd9, 0x48, 0xb7, 0x96, 0x2c, 0xd8, 0x20, 0xd9, 0x23, 0xee, 0xef};

// Avoid coupling with OpenCV
class cvMat
{
public:
    cvMat(int height, int width, int channel) : height(height), width(width), step(step), data(new uint8_t[height * width * channel]) {}
    cvMat(int height, int width, int channel, uint8_t *data) : height(height), width(width), step(step), data(data) {}
    cvMat(const cv::Mat &mat) : height(mat.rows), width(mat.cols), step(mat.step), channel(mat.channels()), data(new uint8_t[height * width * channel])
    {

        // "cv::Mat: mat is not continuous,NOT SUPPORTED!"
        memcpy(data, mat.data, height * width * channel);
    }
    int width;
    int height;
    int step;
    int channel;
    uint8_t *data;
    ~cvMat() { delete[] data; }
};

struct Label
{
    uint8_t label;
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint16_t id;
};

struct Target
{
    cvMat frame;
    std::vector<Label> labels;
};

class RtspStream
{
public:
    RtspStream(const std::string media_server = "127.0.0.1:8554",
               const std::string codec = "hevc_nvmpi",
               const int frameRate = 10,
               const int gop = 5,
               const int bit_rate = 500000,
               const int witdh = 512,
               const int height = 512,
               const int grabId = 0,
               const int log_level = AV_LOG_QUIET);

    ~RtspStream();

    void start();
    bool connect();

    void stop();
    void push_target(const Target &target);
    void set_media_server(const std::string &address);

    int get_frame_rate() const { return frame_rate_; }
    int get_grab_id() const { return grab_id_; }
    bool isStopped();

private:
    bool mat2frame(const cvMat &, AVFrame *);
    void addTarget2Sei(AVPacket *packet, const std::vector<Label> &labels);

    const AVPixelFormat fmt_ = AV_PIX_FMT_YUV420P;

    const std::string des_codec_;
    const int frame_rate_;
    const int grab_id_;
    const int gop_;
    const int64_t bit_rate_;
    const int width_;
    const int height_;

    uint8_t Header_[22];
    std::string rtsp_url_;

    threadsafe_queue<Target> data_queue_;
    int64_t pts_ = 0;
    int64_t dts_ = 0;
    std::atomic<bool> stopped_;

    AVFormatContext *desFmtContext_ = nullptr;
    AVCodecContext *desCodecContext_ = nullptr;
    AVStream *desStream_ = nullptr;
    SwsContext *swsContext_ = nullptr;
    const int64_t max_interleave_delta_ = 100000;
};
