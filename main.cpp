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
#include "rtsp_stream.h"

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <ctime>
#include <iomanip>

#include <opencv2/opencv.hpp>

#include <unistd.h>

#include "logger.h"

int main(int argc, char *argv[])
{
    if (argc < 5)
    {
        std::cout << "Usage: main <input_video> <codec> <rtsp address> <port>  \n"
                  << "\t input_video: the path of the video file to be sent\n"
                  << "\t codec: the codec used to encode the video, (optional: h264_nvmpi , hevc_nvmpi)\n"
                  << "\t rtsp address: the address of the rtsp server\n"
                  << "\t port: the port of the rtsp server\n"
                  << "Examples:\n"
                  << "\t ./main  ./data/output.mp4 h264_nvmpi 127.0.0.1 8554 \n"
                  << std::endl;
        return -1;
    }

    std::string input_video = argv[1];
    std::string codec = argv[2];
    std::string rtsp_address = argv[3];
    std::string port = argv[4];

    cv::VideoCapture cap(input_video, cv::CAP_FFMPEG);

    if (!cap.isOpened())
    {
        std::cerr << "Error: Failed to open video file." << std::endl;
        return -1;
    }

    auto media = new RtspStream(rtsp_address + ":" + port, codec, 8, 5, 500000, 512, 512, 0);

    while (!media->connect())
    {
        /* code */
        sleep(2);
    }
    MLOG_INFO("Connect success");

    // 启动推流线程
    MLOG_INFO("Start push thread");
    auto push_thread = new std::thread(&RtspStream::start, media);
    push_thread->detach();

    MLOG_INFO("Start Read Img");
    int i = 0;
    while (true)
    {
        cv::Mat frame;
        MLOG_INFO("Read frame %d", i);
        if (!cap.read(frame))
        {
            MLOG_INFO("Read finished");
            break;
        }
        MLOG_INFO("Read frame %d", i);
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        media->push_target(new Target{frame, {{0, 1, 2, 3, 4, 5}, {1, 1, 2, 3, 4, 5}}});
        i++;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    cap.release();
    media->stop();
    delete media;
    // delete push_thread;
    return 0;
}
