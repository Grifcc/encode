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
#include "logger.h"

static void avError(const std::string &what, int errNum)
{
    char text[1024];
    av_strerror(errNum, text, sizeof(text));
    MLOG_ERROR("%s: %s", what.c_str(), text);
}

static uint32_t seiNaluSize(uint32_t content)
{
    // start_code   NRI    payload_type     UUID   len   content     tail
    // 4             1      1                16     2     N         1(0x80)
    return 4 + 1 + 1 + 16 + 2 + content + 1;
}

RtspStream::RtspStream(const std::string media_server,
                       const std::string codec,
                       const int frameRate,
                       const int gop,
                       const int bit_rate,
                       const int width,
                       const int height,
                       const int grabId,
                       const int log_level) : rtsp_url_("rtsp://" + media_server + "/" + std::to_string(grabId)),
                                              des_codec_(codec),
                                              frame_rate_(frameRate),
                                              gop_(gop),
                                              bit_rate_(bit_rate),
                                              width_(width),
                                              height_(height),
                                              grab_id_(grabId),
                                              stopped_(false)
{

    MLOG_DEBUG("rtsp_url:%s", rtsp_url_.c_str());
    av_log_set_level(log_level);

    memcpy(Header_, startCode, sizeof(startCode));
    memcpy(Header_ + sizeof(startCode), NRICode, sizeof(NRICode));
    memcpy(Header_ + sizeof(startCode) + sizeof(NRICode), PAYLOADCode, sizeof(PAYLOADCode));
    memcpy(Header_ + sizeof(startCode) + sizeof(NRICode) + sizeof(PAYLOADCode), UUID, sizeof(UUID));

    avdevice_register_all();
    avformat_network_init();
    int code = 0;
    const AVCodec *desCodec = avcodec_find_encoder_by_name(des_codec_.c_str());
    if (!desCodec)
    {
        MLOG_ERROR("avcodec_find_encoder_by_name, can't file codec:%s", des_codec_.c_str());
    }
    desCodecContext_ = avcodec_alloc_context3(desCodec);
    if (!desCodecContext_)
    {
        MLOG_ERROR("avcodec_alloc_context3 err");
        return;
    }
    desCodecContext_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    desCodecContext_->pix_fmt = fmt_;
    desCodecContext_->codec_type = AVMEDIA_TYPE_VIDEO;
    desCodecContext_->width = width_;
    desCodecContext_->height = height_;
    desCodecContext_->framerate = {frame_rate_, 1};
    desCodecContext_->time_base = {1, frame_rate_};
    desCodecContext_->gop_size = gop_;
    desCodecContext_->max_b_frames = 0; //   禁用B帧

    desCodecContext_->bit_rate = bit_rate_;
    desCodecContext_->rc_max_rate = bit_rate_ * 2;
    desCodecContext_->rc_buffer_size = desCodecContext_->width * desCodecContext_->height * desCodecContext_->framerate.num;
    desCodecContext_->qcompress = 1.0;

    av_opt_set_int(desCodecContext_->priv_data, "udu_sei", 1, AV_DICT_MATCH_CASE); // enable udu sei
    if ((code = avcodec_open2(desCodecContext_, desCodec, nullptr)) < 0)
    {
        avError("open desCodecContext err", code);
    }

    // rtsp
    if (avformat_alloc_output_context2(&desFmtContext_, nullptr, "rtsp", rtsp_url_.c_str()) < 0)
    {
        avError("avformat_alloc_output_context2", code);
    }
    av_dump_format(desFmtContext_, 1, desFmtContext_->url, 1);
    desFmtContext_->max_interleave_delta = max_interleave_delta_;
    av_opt_set(desFmtContext_->priv_data, "rtsp_transport", "tcp", 0);
    desStream_ = avformat_new_stream(desFmtContext_, desCodec);
    if (!desStream_)
    {
        MLOG_ERROR("avformat_new_stream err");
    }
    if ((code = avcodec_parameters_from_context(desStream_->codecpar, desCodecContext_)) < 0)
    {
        avError("copy des codecpar err", code);
    }
    desStream_->time_base = {1, frame_rate_};

    if (!(desFmtContext_->oformat->flags & AVFMT_NOFILE))
    {
        if (avio_open(&desFmtContext_->pb, desFmtContext_->url, AVIO_FLAG_WRITE) < 0)
        {
            MLOG_ERROR("error: avio_open('%s')", desFmtContext_->url);
        }
    }

    swsContext_ = sws_getContext(width_, height_, AV_PIX_FMT_GRAY8,
                                 width_, height_, fmt_,
                                 0, nullptr, nullptr, nullptr);

    MLOG_DEBUG("RtspStream init success");
}

bool RtspStream::connect()
{
    int code = 0;
    if ((code = avformat_write_header(desFmtContext_, nullptr)))
    {
        /* code */
        avError("avformat_write_header err", code);
        return false;
    }
    return true;
}
/*视频的采集与转码处理*/
void RtspStream::start()
{
    AVPacket *desPkt = av_packet_alloc();
    AVFrame *desFrame = av_frame_alloc();
    while (!stopped_.load())
    {

        desFrame->format = fmt_;
        auto target = data_queue_.wait_and_pop();
        mat2frame(target->frame, desFrame);
        desFrame->pts = pts_;
        ++pts_;

        // 编码
        int code = 0;
        if ((code = avcodec_send_frame(desCodecContext_, desFrame)) < 0)
        {
            avError("avcodec_send_frame err", code);
            continue;
        }
        if ((avcodec_receive_packet(desCodecContext_, desPkt)) < 0)
        {
            avError("avcodec_receive_packet", code);
            continue;
        }

        desPkt->pts = av_rescale_q_rnd(desPkt->pts, desCodecContext_->time_base, desStream_->time_base, (AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_NEAR_INF));
        desPkt->dts = av_rescale_q_rnd(desPkt->dts, desCodecContext_->time_base, desStream_->time_base, (AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_NEAR_INF));
        desPkt->duration = av_rescale_q(desPkt->duration, desCodecContext_->time_base, desStream_->time_base);

        addTarget2Sei(desPkt, target->labels);
        if ((code = av_interleaved_write_frame(desFmtContext_, desPkt)) < 0)
        {
            avError("av_write_frame err", code);
            continue;
        }
        delete target;
        av_packet_unref(desPkt);
        av_frame_unref(desFrame);
    }
    av_frame_free(&desFrame);
    av_packet_free(&desPkt);
    av_write_trailer(desFmtContext_);
    sws_freeContext(swsContext_);
}

RtspStream::~RtspStream()
{
    if (!stopped_.load())
    {
        /* code */
        stop();
    }

    avcodec_close(desCodecContext_);
    avcodec_free_context(&desCodecContext_);
    avformat_close_input(&desFmtContext_);
    avformat_free_context(desFmtContext_);
}

void RtspStream::stop()
{
    stopped_.store(true);
}

void RtspStream::addTarget2Sei(AVPacket *packet, const std::vector<Label> &labels)
{
    if (packet->stream_index != AVMEDIA_TYPE_VIDEO)
        return;

    auto ori_size = packet->size;
    char *ori_data = (char *)malloc(packet->size);
    memcpy(ori_data, packet->data, packet->size);
    size_t len = labels.size() * 11;

    uint32_t naluSize = seiNaluSize(len);

    av_grow_packet(packet, naluSize);

    uint8_t *pd = packet->data;

    memcpy(pd, startCode, 22);
    pd += 22;

    // NALU UDU SEI LEN
    *pd++ = static_cast<char>(len >> 8 & 0xff); // NALU
    *pd++ = static_cast<char>(len & 0xff);      // NALU

    for (auto &label : labels)
    {
        *pd++ = static_cast<char>(label.label);          // UDU
        *pd++ = static_cast<char>(label.x >> 8 & 0xff);  // x
        *pd++ = static_cast<char>(label.x & 0xff);       // x
        *pd++ = static_cast<char>(label.y >> 8 & 0xff);  // y
        *pd++ = static_cast<char>(label.y & 0xff);       // y
        *pd++ = static_cast<char>(label.w >> 8 & 0xff);  // w
        *pd++ = static_cast<char>(label.w & 0xff);       // w
        *pd++ = static_cast<char>(label.h >> 8 & 0xff);  // h
        *pd++ = static_cast<char>(label.h & 0xff);       // h
        *pd++ = static_cast<char>(label.id >> 8 & 0xff); // id
        *pd++ = static_cast<char>(label.id & 0xff);      // id
    }
    pd += len;
    *pd = 0x80;

    memcpy(pd + 1, ori_data, ori_size);
}

void RtspStream::push_target(Target *target)
{
    data_queue_.push(target);
}

bool RtspStream::isStopped()
{
    return stopped_.load();
}

void RtspStream::set_media_server(const std::string &address)
{
    rtsp_url_ = "rtsp://" + address + "/" + std::to_string(grab_id_);
}

bool RtspStream::mat2frame(const cvMat &inMat, AVFrame *frame)
{

    frame->width = inMat.width;
    frame->height = inMat.height;

    int buffer_size = av_image_get_buffer_size(fmt_, inMat.width, inMat.height, 1);
    uint8_t *buffer = (uint8_t *)av_malloc(buffer_size);
    av_image_fill_arrays(frame->data, frame->linesize, buffer, fmt_, inMat.width, inMat.height, 1);
    memcpy(frame->data[0], inMat.data, inMat.width * inMat.height);
}
