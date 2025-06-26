/*
 * Copyright (c) 2003 Fabrice Bellard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * simple media player based on the FFmpeg libraries
 */

#include "config.h"
#include "config_components.h"
#include <math.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/dict.h"
#include "libavutil/fifo.h"
#include "libavutil/samplefmt.h"
#include "libavutil/time.h"
#include "libavutil/bprint.h"
#include "libavformat/avformat.h"
#include "libavdevice/avdevice.h"
#include "libswscale/swscale.h"
#include "libavutil/opt.h"
#include "libavutil/tx.h"
#include "libswresample/swresample.h"

#include "libavfilter/avfilter.h"
#include "libavfilter/buffersink.h"
#include "libavfilter/buffersrc.h"

#include <SDL.h>
#include <SDL_thread.h>

#include "cmdutils.h"
#include "ffplay_renderer.h"
#include "opt_common.h"

const char program_name[] = "ffplay";
const int program_birth_year = 2003;

#define MAX_QUEUE_SIZE (15 * 1024 * 1024)
#define MIN_FRAMES 25
#define EXTERNAL_CLOCK_MIN_FRAMES 2
#define EXTERNAL_CLOCK_MAX_FRAMES 10

/* Minimum SDL audio buffer size, in samples. */
#define SDL_AUDIO_MIN_BUFFER_SIZE 512
/* Calculate actual buffer size keeping in mind not cause too frequent audio callbacks */
#define SDL_AUDIO_MAX_CALLBACKS_PER_SEC 30

/* Step size for volume control in dB */
#define SDL_VOLUME_STEP (0.75)

/* no AV sync correction is done if below the minimum AV sync threshold */
#define AV_SYNC_THRESHOLD_MIN 0.04
/* AV sync correction is done if above the maximum AV sync threshold */
#define AV_SYNC_THRESHOLD_MAX 0.1
/* If a frame duration is longer than this, it will not be duplicated to compensate AV sync */
#define AV_SYNC_FRAMEDUP_THRESHOLD 0.1
/* no AV correction is done if too big error */
#define AV_NOSYNC_THRESHOLD 10.0

/* maximum audio speed change to get correct sync */
#define SAMPLE_CORRECTION_PERCENT_MAX 10

/* external clock speed adjustment constants for realtime sources based on buffer fullness */
#define EXTERNAL_CLOCK_SPEED_MIN  0.900
#define EXTERNAL_CLOCK_SPEED_MAX  1.010
#define EXTERNAL_CLOCK_SPEED_STEP 0.001

/* we use about AUDIO_DIFF_AVG_NB A-V differences to make the average */
#define AUDIO_DIFF_AVG_NB   20

/* polls for possible required screen refresh at least this often, should be less than 1/fps */
#define REFRESH_RATE 0.01

/* NOTE: the size must be big enough to compensate the hardware audio buffersize size */
/* TODO: We assume that a decoded and resampled frame fits into this buffer */
#define SAMPLE_ARRAY_SIZE (8 * 65536)

#define CURSOR_HIDE_DELAY 1000000

#define USE_ONEPASS_SUBTITLE_RENDER 1

typedef struct MyAVPacketList {
    AVPacket *pkt;
    int serial;
} MyAVPacketList;

typedef struct PacketQueue {
    AVFifo *pkt_list;
    int nb_packets;
    int size;
    int64_t duration;
    int abort_request;
    int serial;
    SDL_mutex *mutex;
    SDL_cond *cond;
} PacketQueue;

#define VIDEO_PICTURE_QUEUE_SIZE 3
#define SUBPICTURE_QUEUE_SIZE 16
#define SAMPLE_QUEUE_SIZE 9
#define FRAME_QUEUE_SIZE FFMAX(SAMPLE_QUEUE_SIZE, FFMAX(VIDEO_PICTURE_QUEUE_SIZE, SUBPICTURE_QUEUE_SIZE))

typedef struct AudioParams {
    int freq;
    AVChannelLayout ch_layout;
    enum AVSampleFormat fmt;
    /* 总样本数 = frame_size × 声道数 用于计算音频帧时长：frame_size / freq 秒 */
    int frame_size; // 每帧的样本数（每个声道），即单个音频帧中每个声道包含的样本数量
    int bytes_per_sec; // 计算比特率和缓冲区大小 bytes_per_sec = freq × 每样本字节数 × 声道数
} AudioParams;

typedef struct Clock {
    double pts;           /* clock base */
    /*// 时钟基准与最后一次更新时间之间的差值（用于补偿系统时间漂移）
    pts_drift = pts - system_time_at_last_update 将系统时间差转换为流时间差*/
    double pts_drift;     /* clock base minus time at which we updated the clock */
    /*// 最后一次更新时钟的系统时间（单位通常为秒）*/
    double last_updated;
    double speed;
    // 当前时钟关联的数据包序列号
    int serial;           /* clock is based on a packet with this serial */
    //  序列与同步检测
    int paused;
    // 指向当前队列序列号的指针（用于检测时钟是否过时）
    // 序列号在 seek 或重置队列时递增，旧序列号的时钟将被视为过时。
    int *queue_serial;    /* pointer to the current packet queue serial, used for obsolete clock detection */
} Clock;

typedef struct FrameData {
    int64_t pkt_pos;
} FrameData;

/* Common struct for handling all types of decoded data and allocated render buffers. */
typedef struct Frame {
    AVFrame *frame;
    AVSubtitle sub;
    int serial;
    double pts;           /* presentation timestamp for the frame */
    double duration;      /* estimated duration of the frame */
    int64_t pos;          /* byte position of the frame in the input file */
    int width;
    int height;
    int format;
    AVRational sar; // 样本宽高比
    int uploaded;
    int flip_v;
} Frame;

typedef struct FrameQueue {
    Frame queue[FRAME_QUEUE_SIZE];
    int rindex; // （读索引）：指向当前待消费帧的位置，表示队列中下一个将被取出处理的帧的索引。
    int windex;
    int size;
    int max_size;
    int keep_last; // shifou保留最后一帧注意
    int rindex_shown; // 已显示读索引：指向当前已显示帧的位置
    SDL_mutex *mutex;
    SDL_cond *cond;
    PacketQueue *pktq;
} FrameQueue;

enum {
    AV_SYNC_AUDIO_MASTER, /* default choice */
    AV_SYNC_VIDEO_MASTER,
    AV_SYNC_EXTERNAL_CLOCK, /* synchronize to an external clock */
};

typedef struct Decoder {
    AVPacket *pkt;
    PacketQueue *queue;
    AVCodecContext *avctx;
    int pkt_serial;
    int finished;
    int packet_pending; // 表示是否有未处理的数据包
    SDL_cond *empty_queue_cond;
    int64_t start_pts;
    AVRational start_pts_tb;
    int64_t next_pts;
    AVRational next_pts_tb;
    SDL_Thread *decoder_tid;
} Decoder;

typedef struct VideoState {
    SDL_Thread *read_tid;
    const AVInputFormat *iformat;
    int abort_request;
    int force_refresh;
    int paused;
    int last_paused;
    int queue_attachments_req;
    int seek_req;
    int seek_flags;
    int64_t seek_pos;
    int64_t seek_rel;
    int read_pause_return; // 是否请求附带图片（如MP3或AAC文件的专辑封面等）
    AVFormatContext *ic;
    int realtime;

    Clock audclk;
    Clock vidclk;
    Clock extclk;

    FrameQueue pictq;
    FrameQueue subpq;
    FrameQueue sampq;

    Decoder auddec;
    Decoder viddec;
    Decoder subdec;

    int audio_stream;

    int av_sync_type;

    double audio_clock;
    int audio_clock_serial;
    double audio_diff_cum; /* used for AV difference average computation */
    double audio_diff_avg_coef;
    double audio_diff_threshold;
    int audio_diff_avg_count;
    AVStream *audio_st;
    PacketQueue audioq;
    int audio_hw_buf_size;
    uint8_t *audio_buf;
    uint8_t *audio_buf1;
    unsigned int audio_buf_size; /* in bytes */ // 音频缓冲区
    unsigned int audio_buf1_size;  // 重采样音频缓冲区
    int audio_buf_index; /* in bytes */ // 音频缓冲区大小
    int audio_write_buf_size; // 重采样音频缓冲区大小
    int audio_volume;
    int muted;
    struct AudioParams audio_src;
    struct AudioParams audio_filter_src;
    struct AudioParams audio_tgt;
    struct SwrContext *swr_ctx;
    int frame_drops_early; // 解码器队列中由于同步问题而提前丢弃的帧
    int frame_drops_late; // 由于播放延迟而丢弃的帧

    enum ShowMode {
        SHOW_MODE_NONE = -1, SHOW_MODE_VIDEO = 0, SHOW_MODE_WAVES, SHOW_MODE_RDFT, SHOW_MODE_NB
    } show_mode;
    int16_t sample_array[SAMPLE_ARRAY_SIZE];
    int sample_array_index;
    int last_i_start;
    AVTXContext *rdft;
    av_tx_fn rdft_fn;
    int rdft_bits;
    float *real_data;
    AVComplexFloat *rdft_data;
    int xpos;
    double last_vis_time;
    SDL_Texture *vis_texture;
    SDL_Texture *sub_texture;
    SDL_Texture *vid_texture;

    int subtitle_stream;
    AVStream *subtitle_st;
    PacketQueue subtitleq;

    double frame_timer; // 帧定时器 下一帧应该显示的时间点 是累计时间
    double frame_last_returned_time; // 上一帧的显示时间戳
    double frame_last_filter_delay; // 上一帧的滤镜延迟
    int video_stream;
    AVStream *video_st;
    PacketQueue videoq;
    double max_frame_duration;      // maximum duration of a frame - above this, we consider the jump a timestamp discontinuity // 最大帧持续时间
    struct SwsContext *sub_convert_ctx;
    int eof;

    char *filename;
    int width, height, xleft, ytop;
    int step;

    int vfilter_idx;
    AVFilterContext *in_video_filter;   // the first filter in the video chain
    AVFilterContext *out_video_filter;  // the last filter in the video chain
    AVFilterContext *in_audio_filter;   // the first filter in the audio chain
    AVFilterContext *out_audio_filter;  // the last filter in the audio chain
    AVFilterGraph *agraph;              // audio filter graph

    int last_video_stream, last_audio_stream, last_subtitle_stream;

    SDL_cond *continue_read_thread;
} VideoState;

/* options specified by the user */
static const AVInputFormat *file_iformat;
static const char *input_filename;
static const char *window_title;
static int default_width  = 640;
static int default_height = 480;
static int screen_width  = 0;
static int screen_height = 0;
static int screen_left = SDL_WINDOWPOS_CENTERED;
static int screen_top = SDL_WINDOWPOS_CENTERED; // 让窗口自动处于屏幕的中心位置
static int audio_disable;
static int video_disable;
static int subtitle_disable;
static const char* wanted_stream_spec[AVMEDIA_TYPE_NB] = {0};
static int seek_by_bytes = -1; // 表示是否按字节进行跳转
static float seek_interval = 10; // 默认跳转间隔10s
static int display_disable;
static int borderless;  // 标志位，用于表示是否创建无边框窗口
static int alwaysontop; // 表示播放器窗口是否始终置顶
static int startup_volume = 100;  // ，代表播放器启动时的音量大小，取值范围通常是0到100
static int show_status = -1;
static int av_sync_type = AV_SYNC_AUDIO_MASTER;
static int64_t start_time = AV_NOPTS_VALUE;
static int64_t duration = AV_NOPTS_VALUE;
static int fast = 0;
static int genpts = 0;
static int lowres = 0;
static int decoder_reorder_pts = -1;
static int autoexit;
static int exit_on_keydown;
static int exit_on_mousedown;
static int loop = 1;
static int framedrop = -1;
static int infinite_buffer = -1;
static enum ShowMode show_mode = SHOW_MODE_NONE;
static const char *audio_codec_name;
static const char *subtitle_codec_name;
static const char *video_codec_name;
double rdftspeed = 0.02;
static int64_t cursor_last_shown; // 记录鼠标光标最后一次显示的时间
static int cursor_hidden = 0;
static const char **vfilters_list = NULL;
static int nb_vfilters = 0;
static char *afilters = NULL;
static int autorotate = 1;
static int find_stream_info = 1;
static int filter_nbthreads = 0;
static int enable_vulkan = 0;
static char *vulkan_params = NULL;
static const char *hwaccel = NULL;

/* current context */
static int is_full_screen;
static int64_t audio_callback_time;

#define FF_QUIT_EVENT    (SDL_USEREVENT + 2)

static SDL_Window *window;
static SDL_Renderer *renderer;
static SDL_RendererInfo renderer_info = {0};
static SDL_AudioDeviceID audio_dev;

static VkRenderer *vk_renderer;

// 建立FFmpeg像素格式（AVPixelFormat）和SDL像素格式之间的映射关系
static const struct TextureFormatEntry {
    enum AVPixelFormat format;
    int texture_fmt;
} sdl_texture_format_map[] = {
    { AV_PIX_FMT_RGB8,           SDL_PIXELFORMAT_RGB332 },
    { AV_PIX_FMT_RGB444,         SDL_PIXELFORMAT_RGB444 },
    { AV_PIX_FMT_RGB555,         SDL_PIXELFORMAT_RGB555 },
    { AV_PIX_FMT_BGR555,         SDL_PIXELFORMAT_BGR555 },
    { AV_PIX_FMT_RGB565,         SDL_PIXELFORMAT_RGB565 },
    { AV_PIX_FMT_BGR565,         SDL_PIXELFORMAT_BGR565 },
    { AV_PIX_FMT_RGB24,          SDL_PIXELFORMAT_RGB24 },
    { AV_PIX_FMT_BGR24,          SDL_PIXELFORMAT_BGR24 },
    { AV_PIX_FMT_0RGB32,         SDL_PIXELFORMAT_RGB888 },
    { AV_PIX_FMT_0BGR32,         SDL_PIXELFORMAT_BGR888 },
    { AV_PIX_FMT_NE(RGB0, 0BGR), SDL_PIXELFORMAT_RGBX8888 },
    { AV_PIX_FMT_NE(BGR0, 0RGB), SDL_PIXELFORMAT_BGRX8888 },
    { AV_PIX_FMT_RGB32,          SDL_PIXELFORMAT_ARGB8888 },
    { AV_PIX_FMT_RGB32_1,        SDL_PIXELFORMAT_RGBA8888 },
    { AV_PIX_FMT_BGR32,          SDL_PIXELFORMAT_ABGR8888 },
    { AV_PIX_FMT_BGR32_1,        SDL_PIXELFORMAT_BGRA8888 },
    { AV_PIX_FMT_YUV420P,        SDL_PIXELFORMAT_IYUV },
    { AV_PIX_FMT_YUYV422,        SDL_PIXELFORMAT_YUY2 },
    { AV_PIX_FMT_UYVY422,        SDL_PIXELFORMAT_UYVY },
};

// opt_add_vfilter 函数的主要功能是处理命令行选项，将用户指定的视频过滤器添加到 vfilters_list 数组中
static int opt_add_vfilter(void *optctx, const char *opt, const char *arg)
{
    // 宏，用于动态扩展数组的大小 重新分配内存
    int ret = GROW_ARRAY(vfilters_list, nb_vfilters);
    if (ret < 0)
        return ret;
    // 复制一个字符串到新分配的内存中
    vfilters_list[nb_vfilters - 1] = av_strdup(arg);
    if (!vfilters_list[nb_vfilters - 1])
        return AVERROR(ENOMEM);

    return 0;
}
// 通过区分单声道和多声道的情况，采用不同的比较逻辑来判断两个音频格式是否相同
// 返回0 则表示两个音频是一样的
static inline
int cmp_audio_fmts(enum AVSampleFormat fmt1, int64_t channel_count1,
                   enum AVSampleFormat fmt2, int64_t channel_count2)
{
    /* If channel count == 1, planar and non-planar formats are the same */
    if (channel_count1 == 1 && channel_count2 == 1)
        return av_get_packed_sample_fmt(fmt1) != av_get_packed_sample_fmt(fmt2);
    else
        return channel_count1 != channel_count2 || fmt1 != fmt2;
}
// 将pkt放入到队列中
static int packet_queue_put_private(PacketQueue *q, AVPacket *pkt)
{
    MyAVPacketList pkt1;
    int ret;

    if (q->abort_request)
       return -1;


    pkt1.pkt = pkt;
    pkt1.serial = q->serial;
    // int av_fifo_write(AVFifoBuffer *fifo, const void *src, int size);
    // 环形缓冲区（FIFO）
    ret = av_fifo_write(q->pkt_list, &pkt1, 1);
    if (ret < 0)
        return ret;
    q->nb_packets++;
    q->size += pkt1.pkt->size + sizeof(pkt1); // 总大小 累加
    q->duration += pkt1.pkt->duration;  // 总时长 累加
    /* XXX: should duplicate packet data in DV case */
    SDL_CondSignal(q->cond);
    return 0;
}

// 一个 AVPacket 数据包安全地放入 PacketQueue 队列中，使用零拷贝技术优化性能，
// 并在多线程环境下保证线程安全 转移数据所有权
static int packet_queue_put(PacketQueue *q, AVPacket *pkt)
{
    AVPacket *pkt1;
    int ret;

    pkt1 = av_packet_alloc();
    if (!pkt1) {
        av_packet_unref(pkt);
        return -1;
    }
    av_packet_move_ref(pkt1, pkt); // 零拷贝优化 转移引用计数，避免数据拷贝，适用于高频数据流场

    SDL_LockMutex(q->mutex);
    ret = packet_queue_put_private(q, pkt1);
    SDL_UnlockMutex(q->mutex);

    if (ret < 0)
        av_packet_free(&pkt1);

    return ret;
}

/*
向指定流（如音频、视频、字幕）的 PacketQueue 中放入一个空数据包（Null Packet），
主要用于触发解码器处理流的终止或刷新状态
空包的作用：空包（AVPacket 类型）的 data 和 size 字段通常为空，
用于通知解码器当前流已结束或需要立即刷新缓存帧（如 FLUSH 操作)或则拖动进度或切换流时，
需清空当前队列并发送空包，强制解码器重置状态
*/
static int packet_queue_put_nullpacket(PacketQueue *q, AVPacket *pkt, int stream_index)
{
    pkt->stream_index = stream_index;
    return packet_queue_put(q, pkt);
}

/* packet queue handling */
// 初始化线程安全的数据包队列结构 PacketQueue
static int packet_queue_init(PacketQueue *q)
{
    memset(q, 0, sizeof(PacketQueue));
    // 创建动态增长的 FIFO 队列，用于存储 MyAVPacketList 类型的数据包节点
    q->pkt_list = av_fifo_alloc2(1, sizeof(MyAVPacketList), AV_FIFO_FLAG_AUTO_GROW);
    if (!q->pkt_list)
        return AVERROR(ENOMEM);
    q->mutex = SDL_CreateMutex();
    if (!q->mutex) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    q->cond = SDL_CreateCond();
    if (!q->cond) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    q->abort_request = 1;
    return 0;
}

// 清空队列，在seek时
// 如果直接释放 1. 引发内存泄漏或引用计数错误
// 队列底层数据结构限制，AVFifo 特性：PacketQueue 使用 AVFifo 作为底层容器（基于循环数组实现），
// 其存储方式为二进制数据块而非指针数组4。直接释放整个 AVFifo 缓冲区会导致内存泄漏，因 AVPacket 对象需单独释放4。
// 元素独立性：队列中的每个 MyAVPacketList 元素均通过 av_fifo_write 写入，需通过 av_fifo_read 逐个取出以触发正确的内存释放逻辑。
static void packet_queue_flush(PacketQueue *q)
{
    MyAVPacketList pkt1;

    SDL_LockMutex(q->mutex);
    while (av_fifo_read(q->pkt_list, &pkt1, 1) >= 0) // 逐项读取元素
        av_packet_free(&pkt1.pkt);  // 逐项释放
    q->nb_packets = 0;
    q->size = 0;
    q->duration = 0;
    q->serial++; // 标识队列的序列号更新，用于区分清空前后的数据包
    SDL_UnlockMutex(q->mutex);
}

static void packet_queue_destroy(PacketQueue *q)
{
    packet_queue_flush(q);  // 先通过 flush 操作释放所有数据包引用计数，避免缓冲区残留导致内存泄漏
    av_fifo_freep2(&q->pkt_list); // 销毁存储容器 AVFifo 释放其内部数据缓冲区（buffer 字段）占用的内存
    SDL_DestroyMutex(q->mutex);
    SDL_DestroyCond(q->cond);
}

// 强制终止 PacketQueue 队列
// 播放终止,seek时先中止再清空
// 唤醒因等待数据而阻塞的线程（例如 packet_queue_get 中等待队列非空的线程），使其检测终止标志并退出循环
static void packet_queue_abort(PacketQueue *q)
{
    SDL_LockMutex(q->mutex);

    q->abort_request = 1; // 终止标志

    SDL_CondSignal(q->cond);

    SDL_UnlockMutex(q->mutex);
}

/*
播放初始化: 重置abort_request，递增serial,激活队列并标记数据流起始点
Seek后恢复: 清空队列，确保数据流与旧数据隔离
*/
static void packet_queue_start(PacketQueue *q)
{
    SDL_LockMutex(q->mutex);
    q->abort_request = 0;
    q->serial++;
    SDL_UnlockMutex(q->mutex);
}

/* return < 0 if aborted, 0 if no packet and > 0 if packet.  */
// 非阻塞式获取packet
static int packet_queue_get(PacketQueue *q, AVPacket *pkt, int block, int *serial)
{
    MyAVPacketList pkt1;
    int ret;

    SDL_LockMutex(q->mutex);

    for (;;) {
        if (q->abort_request) {
            ret = -1;
            break;
        }

        if (av_fifo_read(q->pkt_list, &pkt1, 1) >= 0) {
            q->nb_packets--;
            q->size -= pkt1.pkt->size + sizeof(pkt1);
            q->duration -= pkt1.pkt->duration;
            av_packet_move_ref(pkt, pkt1.pkt);
            if (serial)
                *serial = pkt1.serial;
            av_packet_free(&pkt1.pkt);
            ret = 1;
            break;
        } else if (!block) {
            ret = 0;
            break;
        } else {
            SDL_CondWait(q->cond, q->mutex);
        }
    }
    SDL_UnlockMutex(q->mutex);
    return ret;
}

static int decoder_init(Decoder *d, AVCodecContext *avctx, PacketQueue *queue, SDL_cond *empty_queue_cond) {
    memset(d, 0, sizeof(Decoder));
    d->pkt = av_packet_alloc();
    if (!d->pkt)
        return AVERROR(ENOMEM);
    d->avctx = avctx; // 解码器上下文
    d->queue = queue;
    d->empty_queue_cond = empty_queue_cond; // 队列空条件变量
    d->start_pts = AV_NOPTS_VALUE;
    d->pkt_serial = -1;
    return 0;
}

// 从数据包队列中获取数据并进行解码
/*
解决B帧导致的PTS/DTS不一致问题,统一不同媒体类型(音/视频)的时间基准,补偿解码器内部缓冲引入的延迟

启发式估算时间戳
通过分析帧顺序、解码延迟等上下文信息动态生成, 当原始PTS无效时自动启用（如AV_NOPTS_VALUE情况
适用于B帧导致的时间戳跳变修正

解码时间戳(DTS)
直接从压缩流数据包中提取
严格反映解码器处理帧的顺序
对不含B帧的流与PTS值相同

特性	   启发式估算时间戳	     解码时间戳(DTS)
可靠性	   可能包含误差但保证连续性  精确但可能不匹配显示顺序
B帧处理	   自动修正显示时序	     需配合PTS使用
容错性	   支持损坏流媒体	     依赖原始数据完整性
计算开销   需要运行时分析	     直接读取无额外计算
*/
static int decoder_decode_frame(Decoder *d, AVFrame *frame, AVSubtitle *sub) {
    int ret = AVERROR(EAGAIN); // 初始化返回值为EAGAIN

    for (;;) { // 1. 判断是否是同一个serial
        if (d->queue->serial == d->pkt_serial) {
            do {
                // 检查中止请求
                if (d->queue->abort_request)
                    return -1;

                switch (d->avctx->codec_type) {
                    case AVMEDIA_TYPE_VIDEO:
                        // 从已初始化的解码器(AVCodecContext)中获取解码后的帧数据(AVFrame)
                        ret = avcodec_receive_frame(d->avctx, frame);
                        if (ret >= 0) {
                            // 根据decoder_reorder_pts参数选择时间戳生成策略
                            if (decoder_reorder_pts == -1) {
                                frame->pts = frame->best_effort_timestamp; // 启发式估算
                            } else if (!decoder_reorder_pts) {
                                frame->pts = frame->pkt_dts; // 使用解码时间戳
                            }
                        }
                        break;
                    case AVMEDIA_TYPE_AUDIO:
                        ret = avcodec_receive_frame(d->avctx, frame);
                        if (ret >= 0) {
                            // 时间戳转换 解码后的帧需要转换为播放时间基
                            // 将解码后的音频帧的PTS转换为基于音频采样率的时间基（tb = {1, sample_rate}），确保时间戳单位统一，便于后续播放。
                            AVRational tb = (AVRational){1, frame->sample_rate}; // 音频帧的采样率
                            if (frame->pts != AV_NOPTS_VALUE)
                                // 完成音频帧时间基转换和连续时间戳生成
                                // 通过 av_rescale_q 函数将音频帧的 PTS 从编解码器包时间基（d->avctx->pkt_timebase）
                                // 转换为音频采样率时间基（tb），确保时间戳单位统一为音频播放所需的基准
                                // av_rescale_q(a,b,c)实现公式：a * b / c，完成时间单位的等比缩放
                                frame->pts = av_rescale_q(frame->pts, d->avctx->pkt_timebase, tb);
                            // 当帧无有效PTS时（如原始流未包含或解码错误），使用解码器缓存的预测值
                            else if (d->next_pts != AV_NOPTS_VALUE)
                                frame->pts = av_rescale_q(d->next_pts, d->next_pts_tb, tb);
                            // 更新下一帧的预期pts
                            if (frame->pts != AV_NOPTS_VALUE) {
                                // 而音频帧的PTS严格遵循采样点线性增长，因此可直接相加
                                // 直接相加结果的单位是采样点数量
                                d->next_pts = frame->pts + frame->nb_samples;
                                d->next_pts_tb = tb;
                            }
                        }
                        break;
                }
                if (ret == AVERROR_EOF) {
                    d->finished = d->pkt_serial; // 标记流结束状态
                    avcodec_flush_buffers(d->avctx); // 清空解码器内部缓存
                    return 0;
                }
                if (ret >= 0)
                    return 1;
            } while (ret != AVERROR(EAGAIN));
        }

        // 获取输入数据包
        // 数据包获取与序列号同步逻辑，主要用于解码线程从队列获取压缩数据包时的状态管理
        /*
          整体流程总结
          队列状态检测 → 空队列时唤醒生产者
          数据包获取   → 处理pending或从队列拉取新包
          序列号校验   → 变化时清空解码器并重置状态
          有效性过滤   → 仅处理序列号匹配的包，其余丢弃
        */
        do {
            if (d->queue->nb_packets == 0) // 如果队列是空的 通知线程进行补充
                SDL_CondSignal(d->empty_queue_cond);
            // 处理未处理完的数据包，通常是解码失败
            if (d->packet_pending) {
                d->packet_pending = 0;
            } else {
                int old_serial = d->pkt_serial;
                if (packet_queue_get(d->queue, d->pkt, 1, &d->pkt_serial) < 0)
                    return -1;
                // 验证数据包序列号一致性
                if (old_serial != d->pkt_serial) {
                    avcodec_flush_buffers(d->avctx);
                    d->finished = 0;
                    d->next_pts = d->start_pts;
                    d->next_pts_tb = d->start_pts_tb;
                }
            }
            if (d->queue->serial == d->pkt_serial)
                break;
            av_packet_unref(d->pkt);
        } while (1);

        // 字幕解码
        if (d->avctx->codec_type == AVMEDIA_TYPE_SUBTITLE) {
            int got_frame = 0;
            ret = avcodec_decode_subtitle2(d->avctx, sub, &got_frame, d->pkt);
            if (ret < 0) {
                ret = AVERROR(EAGAIN);
            } else {
                if (got_frame && !d->pkt->data) {
                    d->packet_pending = 1;
                }
                ret = got_frame ? 0 : (d->pkt->data ? AVERROR(EAGAIN) : AVERROR_EOF);
            }
            av_packet_unref(d->pkt);
        } else {
            if (d->pkt->buf && !d->pkt->opaque_ref) {
                FrameData *fd;
                // 分配帧数据缓冲区
                d->pkt->opaque_ref = av_buffer_allocz(sizeof(*fd));
                if (!d->pkt->opaque_ref)
                    return AVERROR(ENOMEM);
                fd = (FrameData*)d->pkt->opaque_ref->data;
                fd->pkt_pos = d->pkt->pos;
            }
            // 发送数据包到解码器
            if (avcodec_send_packet(d->avctx, d->pkt) == AVERROR(EAGAIN)) {
                av_log(d->avctx, AV_LOG_ERROR, "Receive_frame and send_packet both returned EAGAIN, which is an API violation.\n");
                d->packet_pending = 1;
            } else {
                av_packet_unref(d->pkt);
            }
        }
    }
}

// 销毁解码器上下文
static void decoder_destroy(Decoder *d) {
    av_packet_free(&d->pkt);  // 释放AVPacket内存
    avcodec_free_context(&d->avctx);  // 释放解码器上下文
}

// 清理帧队列元素
static void frame_queue_unref_item(Frame *vp)
{
    av_frame_unref(vp->frame);
    avsubtitle_free(&vp->sub);
}

// 帧队列(FrameQueue)的初始化逻辑，主要用于音视频同步和帧缓存管理
// 设置缓存帧的数量以及是否保留最后一帧
static int frame_queue_init(FrameQueue *f, PacketQueue *pktq, int max_size, int keep_last)
{
    int i;
    memset(f, 0, sizeof(FrameQueue));
    if (!(f->mutex = SDL_CreateMutex())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    if (!(f->cond = SDL_CreateCond())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    f->pktq = pktq;
    f->max_size = FFMIN(max_size, FRAME_QUEUE_SIZE); // 限制队列的最大帧缓存数量
    f->keep_last = !!keep_last; // 若keep_last=1，保留最后一帧避免黑屏
    for (i = 0; i < f->max_size; i++)
        if (!(f->queue[i].frame = av_frame_alloc())) // 初始化
            return AVERROR(ENOMEM);
    return 0;
}

/*
销毁FrameQueue结构体及其所有关联资源，包括：

释放队列中所有AVFrame对象
销毁线程同步原语（互斥锁和条件变量）

1. 代码可维护性优化
统一访问接口：frame_queue_unref_item是专门处理Frame类型资源的函数，通过指针参数传递队列元素，保持与队列其他操作（如push/pop）一致的访问方式13
隔离实现细节：避免在销毁函数中直接操作f->queue[i]的内部字段，降低队列数据结构变更带来的影响
2. 线程安全兼容性
指针传递的原子性：获取数组元素地址的操作是原子性的，而直接操作数组元素可能涉及多步内存访问（如计算偏移量），在未加锁环境下存在风险
与初始化对称：frame_queue_init中同样通过指针初始化队列元素，保持生命周期管理的一致性
*/
static void frame_queue_destroy(FrameQueue *f)
{
    int i;
    for (i = 0; i < f->max_size; i++) {
        Frame *vp = &f->queue[i];  // 可能误操作未初始化的元素
        frame_queue_unref_item(vp);
        av_frame_free(&vp->frame);
    }
    SDL_DestroyMutex(f->mutex);
    SDL_DestroyCond(f->cond);
}

// 通过条件变量（SDL_CondSignal）通知等待线程
// 发送信号，唤醒 一个 正在等待该条件变量的线程（如果有）。
/*
避免竞态条件：如果 SDL_CondSignal 调用时没有锁保护，可能会导致：

消费者线程在检查条件（如 queue->size > 0）和 SDL_CondWait 之间错过信号（lost wakeup problem）。
生产者线程可能在消费者线程进入等待之前发送信号，导致消费者线程永远阻塞。
确保原子性：SDL_CondSignal 必须与互斥锁配合使用，保证信号发送时，消费者线程的等待逻辑是同步的。
*/
static void frame_queue_signal(FrameQueue *f)
{
    SDL_LockMutex(f->mutex);
    SDL_CondSignal(f->cond); // 与SDL_CondWait配对使用 发送信号
    SDL_UnlockMutex(f->mutex);
}
// 获取队列中第一个待显示的帧 帧头
static Frame *frame_queue_peek(FrameQueue *f)
{
    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

// 预取下一个可读帧，作用:提前分析后续帧的问题，实现帧间差值计算或缓冲状态预判
static Frame *frame_queue_peek_next(FrameQueue *f)
{
    return &f->queue[(f->rindex + f->rindex_shown + 1) % f->max_size];
}

// 当前已显示,渲染的最新帧, 优先显示，确保最低延迟。
static Frame *frame_queue_peek_last(FrameQueue *f)
{
    return &f->queue[f->rindex];
}

static Frame *frame_queue_peek_writable(FrameQueue *f)
{
    /* wait until we have space to put a new frame */
    SDL_LockMutex(f->mutex);
    while (f->size >= f->max_size &&
           !f->pktq->abort_request) {
        SDL_CondWait(f->cond, f->mutex);
    }
    SDL_UnlockMutex(f->mutex);

    if (f->pktq->abort_request) // 它的修改都是在互斥锁之下的
        return NULL;

    return &f->queue[f->windex];
}

//  多线程环境下帧队列的可读帧获取接口，其核心作用是为消费者线程提供安全访问帧队列的机制
static Frame *frame_queue_peek_readable(FrameQueue *f)
{
    /* wait until we have a readable a new frame */
    SDL_LockMutex(f->mutex);
    while (f->size - f->rindex_shown <= 0 &&  // 检查是否有新的可读帧
           !f->pktq->abort_request) {
        SDL_CondWait(f->cond, f->mutex);
    }
    SDL_UnlockMutex(f->mutex);

    if (f->pktq->abort_request)
        return NULL;

    return &f->queue[(f->rindex + f->rindex_shown) % f->max_size];
}

// 向环形队列写入新帧
static void frame_queue_push(FrameQueue *f)
{
    if (++f->windex == f->max_size)
        f->windex = 0;
    SDL_LockMutex(f->mutex);
    f->size++;
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

// 在frame使用完后，释放frame且移动rindex
static void frame_queue_next(FrameQueue *f)
{
    // 首次读取且需保留最后一帧
    if (f->keep_last && !f->rindex_shown) {
        f->rindex_shown = 1; // 标记为已显示 但不移动rindex
        return;
    }
    // 释放单个帧资源。
    // 减少引用计数，若计数为 0 则释放 ，重置元数据pts等，清空serial等辅助信息
    frame_queue_unref_item(&f->queue[f->rindex]);
    if (++f->rindex == f->max_size)
        f->rindex = 0;
    SDL_LockMutex(f->mutex);
    f->size--;
    SDL_CondSignal(f->cond);
    SDL_UnlockMutex(f->mutex);
}

/* return the number of undisplayed frames in the queue */
// 计算未显示帧数量
static int frame_queue_nb_remaining(FrameQueue *f)
{
    return f->size - f->rindex_shown;
}

/* return last shown position */
// 返回的是当前帧在输入文件中的字节偏移量
static int64_t frame_queue_last_pos(FrameQueue *f)
{
    Frame *fp = &f->queue[f->rindex];
    if (f->rindex_shown && fp->serial == f->pktq->serial)
        return fp->pos;
    else
        return -1;
}

static void decoder_abort(Decoder *d, FrameQueue *fq)
{
    packet_queue_abort(d->queue); // 设置终止信号 唤醒等待中的帧队列
    frame_queue_signal(fq);  // 唤醒线程 唤醒等待中的包队列
    SDL_WaitThread(d->decoder_tid, NULL);  // 阻塞当前线程，等待指定目标线程执行完毕，然后释放相关内存
    d->decoder_tid = NULL;  // 避免悬空指针与重复操作风险
    packet_queue_flush(d->queue);
}

static inline void fill_rectangle(int x, int y, int w, int h)
{
    SDL_Rect rect;
    rect.x = x;
    rect.y = y;
    rect.w = w;
    rect.h = h;
    if (w && h) // w和h 非0
        SDL_RenderFillRect(renderer, &rect); // 填充矩形区域
}

// 动态调整或创建 SDL 纹理的核心逻辑
static int realloc_texture(SDL_Texture **texture, Uint32 new_format, int new_width, int new_height, SDL_BlendMode blendmode, int init_texture)
{
    Uint32 format;
    int access, w, h;
    // 若纹理不存在、查询失败或尺寸/格式不匹配，则触发重建逻辑
    if (!*texture || SDL_QueryTexture(*texture, &format, &access, &w, &h) < 0 || new_width != w || new_height != h || new_format != format) {
        void *pixels;
        int pitch;
        if (*texture)
            SDL_DestroyTexture(*texture);
            // SDL_TEXTUREACCESS_STREAMING 允许 CPU 频繁写入纹理数据
        if (!(*texture = SDL_CreateTexture(renderer, new_format, SDL_TEXTUREACCESS_STREAMING, new_width, new_height)))
            return -1;
        // 控制纹理与背景的混合效果（如透明度叠加）
        if (SDL_SetTextureBlendMode(*texture, blendmode) < 0)
            return -1;
        if (init_texture) { // 初始化纹理数据
            // 防止未初始化内存导致渲染异常（如残留图像），在分辨率切换时，纹理数据有可能造成污染，残留之前的数据，有的区域没有覆盖数据
            if (SDL_LockTexture(*texture, NULL, &pixels, &pitch) < 0)
                return -1;
            memset(pixels, 0, pitch * new_height);
            SDL_UnlockTexture(*texture);
        }
        av_log(NULL, AV_LOG_VERBOSE, "Created %dx%d texture with %s.\n", new_width, new_height, SDL_GetPixelFormatName(new_format));
    }
    return 0;
}

// 根据视频的样本宽高比(SAR)和图像尺寸，结合屏幕参数，计算视频在屏幕上居中显示时的位置和缩放尺寸，并确保宽高为偶数（兼容YUV格式）
static void calculate_display_rect(SDL_Rect *rect,
                                   int scr_xleft, int scr_ytop, int scr_width, int scr_height,
                                   int pic_width, int pic_height, AVRational pic_sar)
{
    AVRational aspect_ratio = pic_sar;
    // sar 样本宽高比，像素宽高比
    int64_t width, height, x, y;
    // 处理无效的 SAR 若sar是 0：1，则设置为1：1(方形)
    if (av_cmp_q(aspect_ratio, av_make_q(0, 1)) <= 0)
        aspect_ratio = av_make_q(1, 1);

    // 显示宽高比（DAR） = SAR × (图像宽度 / 图像高度)
    aspect_ratio = av_mul_q(aspect_ratio, av_make_q(pic_width, pic_height));

    /* XXX: we suppose the screen has a 1.0 pixel ratio */
    height = scr_height;
    // & ~1: 确保宽度为偶数（兼容 YUV 格式要求） YUV420 等格式要求宽高为偶数，& ~1 确保末位为 0
    width = av_rescale(height, aspect_ratio.num, aspect_ratio.den) & ~1;
    if (width > scr_width) { // 若计算的宽度超过屏幕宽度，则以 scr_width 为基准重新计算高度。
        width = scr_width;
        height = av_rescale(width, aspect_ratio.den, aspect_ratio.num) & ~1;
    }
    // 居中坐标计算
    x = (scr_width - width) / 2;
    y = (scr_height - height) / 2;
    // 输出显示矩形参数
    rect->x = scr_xleft + x;
    rect->y = scr_ytop  + y;
    rect->w = FFMAX((int)width,  1);
    rect->h = FFMAX((int)height, 1);
}

// 该函数用于将FFmpeg 像素格式（AV_PIX_FMT）转换为SDL纹理像素格式（SDL_PIXELFORMAT）并根据格式特性设置混合模式
// FFmpeg 与 SDL 渲染管线衔接的关键环节
static void get_sdl_pix_fmt_and_blendmode(int format, Uint32 *sdl_pix_fmt, SDL_BlendMode *sdl_blendmode)
{
    int i;
    *sdl_blendmode = SDL_BLENDMODE_NONE;  // 混合模式 // 不启用透明度混合
    *sdl_pix_fmt = SDL_PIXELFORMAT_UNKNOWN;
    if (format == AV_PIX_FMT_RGB32   ||
        format == AV_PIX_FMT_RGB32_1 ||
        format == AV_PIX_FMT_BGR32   ||
        format == AV_PIX_FMT_BGR32_1)
        *sdl_blendmode = SDL_BLENDMODE_BLEND;
    // 通过遍历静态数组 sdl_texture_format_map，匹配 FFmpeg 与 SDL 的像素格式对应关系
    for (i = 0; i < FF_ARRAY_ELEMS(sdl_texture_format_map); i++) {
        if (format == sdl_texture_format_map[i].format) {
            *sdl_pix_fmt = sdl_texture_format_map[i].texture_fmt;
            return;
        }
    }
}

// AVFrame 视频帧数据上传到SDL纹理（SDL_Texture)，支持多种像素格式（如 YUV、RGB）和内存布局（正向/逆向存储）
static int upload_texture(SDL_Texture **tex, AVFrame *frame)
{
    int ret = 0;
    Uint32 sdl_pix_fmt;
    SDL_BlendMode sdl_blendmode;
    get_sdl_pix_fmt_and_blendmode(frame->format, &sdl_pix_fmt, &sdl_blendmode); // 转换 FFmpeg 像素格式到 SDL 支持的格式，并确定混合模式
    // 重新分配纹理（若格式或尺寸变化）
    if (realloc_texture(tex, sdl_pix_fmt == SDL_PIXELFORMAT_UNKNOWN ? SDL_PIXELFORMAT_ARGB8888 : sdl_pix_fmt, frame->width, frame->height, sdl_blendmode, 0) < 0)
        return -1;
    // 根据像素格式和内存布局，调用 SDL API 更新纹理数据
    switch (sdl_pix_fmt) {
        case SDL_PIXELFORMAT_IYUV:
            // 内存布局处理
            // 正向存储（linesize > 0）：直接传递数据指针和步长。
            // 逆向存储（linesize < 0）
            if (frame->linesize[0] > 0 && frame->linesize[1] > 0 && frame->linesize[2] > 0) {
                ret = SDL_UpdateYUVTexture(*tex, NULL, frame->data[0], frame->linesize[0],
                                                       frame->data[1], frame->linesize[1],
                                                       frame->data[2], frame->linesize[2]);
            // 调整数据指针到内存末尾，并通过负步长逆向读取数据（兼容倒序存储的视频帧）
            } else if (frame->linesize[0] < 0 && frame->linesize[1] < 0 && frame->linesize[2] < 0) {
                // 计算 UV 平面的起始位置（YUV420 的 UV 平面高度为 Y 的一半）
                // AV_CEIL_RSHIFT(frame->height, 1)  // 等效于 (frame->height + 1) / 2
                ret = SDL_UpdateYUVTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height                    - 1), -frame->linesize[0],
                                                       frame->data[1] + frame->linesize[1] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[1],
                                                       frame->data[2] + frame->linesize[2] * (AV_CEIL_RSHIFT(frame->height, 1) - 1), -frame->linesize[2]);
            } else {
                av_log(NULL, AV_LOG_ERROR, "Mixed negative and positive linesizes are not supported.\n");
                return -1;
            }
            break;
        default:
            if (frame->linesize[0] < 0) {
                ret = SDL_UpdateTexture(*tex, NULL, frame->data[0] + frame->linesize[0] * (frame->height - 1), -frame->linesize[0]);
            } else {
                ret = SDL_UpdateTexture(*tex, NULL, frame->data[0], frame->linesize[0]);
            }
            break;
    }
    return ret;
}

static enum AVColorSpace sdl_supported_color_spaces[] = {
    AVCOL_SPC_BT709,
    AVCOL_SPC_BT470BG,
    AVCOL_SPC_SMPTE170M,
};

// 根据 AVFrame 的颜色空间属性动态设置 SDL 的 YUV 转换模式，确保 YUV 到 RGB 的渲染符合视频源的色彩标准
static void set_sdl_yuv_conversion_mode(AVFrame *frame)
{
#if SDL_VERSION_ATLEAST(2,0,8)
    SDL_YUV_CONVERSION_MODE mode = SDL_YUV_CONVERSION_AUTOMATIC;
    if (frame && (frame->format == AV_PIX_FMT_YUV420P || frame->format == AV_PIX_FMT_YUYV422 || frame->format == AV_PIX_FMT_UYVY422)) {
        if (frame->color_range == AVCOL_RANGE_JPEG)
            mode = SDL_YUV_CONVERSION_JPEG;
        else if (frame->colorspace == AVCOL_SPC_BT709)
            mode = SDL_YUV_CONVERSION_BT709;
        else if (frame->colorspace == AVCOL_SPC_BT470BG || frame->colorspace == AVCOL_SPC_SMPTE170M)
            mode = SDL_YUV_CONVERSION_BT601;
    }
    SDL_SetYUVConversionMode(mode); /* FIXME: no support for linear transfer */
#endif
}

// 视频播放器中负责渲染当前视频帧及叠加字幕的核心模块
static void video_image_display(VideoState *is)
{
    Frame *vp;
    Frame *sp = NULL;
    SDL_Rect rect;

    vp = frame_queue_peek_last(&is->pictq); // 获取最新的一帧 队列中最新/最后插入的帧（即当前应该显示的帧）
    if (vk_renderer) { // Vulkan渲染优先
        vk_renderer_display(vk_renderer, vp->frame);
        return;
    }

    if (is->subtitle_st) {
        if (frame_queue_nb_remaining(&is->subpq) > 0) {
            sp = frame_queue_peek(&is->subpq);
            // 校验其显示时间是否匹配视频帧       // 字幕内部偏移，是一个时间点，加起来就是显示的绝对时间点
            if (vp->pts >= sp->pts + ((float) sp->sub.start_display_time / 1000)) {
                // 纹理上传：通过SDL_LockTexture和sws_scale将字幕数据写入sub_texture，标记为已上传（sp->uploaded = 1
                if (!sp->uploaded) {
                    uint8_t* pixels[4];
                    int pitch[4];
                    int i;
                    // 纹理初始化与尺寸同步
                    if (!sp->width || !sp->height) {
                        sp->width = vp->width;
                        sp->height = vp->height;
                    }
                    if (realloc_texture(&is->sub_texture, SDL_PIXELFORMAT_ARGB8888, sp->width, sp->height, SDL_BLENDMODE_BLEND, 1) < 0)
                        return;

                    for (i = 0; i < sp->sub.num_rects; i++) {
                        AVSubtitleRect *sub_rect = sp->sub.rects[i];
                        // 边界安全处理,裁剪字幕区域以防止越界
                        sub_rect->x = av_clip(sub_rect->x, 0, sp->width );
                        sub_rect->y = av_clip(sub_rect->y, 0, sp->height);
                        sub_rect->w = av_clip(sub_rect->w, 0, sp->width  - sub_rect->x);
                        sub_rect->h = av_clip(sub_rect->h, 0, sp->height - sub_rect->y);
                        // 格式转换：使用sws_getCachedContext将字幕的调色板格式（AV_PIX_FMT_PAL8）转换为SDL支持的BGRA格式
                        is->sub_convert_ctx = sws_getCachedContext(is->sub_convert_ctx,
                            sub_rect->w, sub_rect->h, AV_PIX_FMT_PAL8,
                            sub_rect->w, sub_rect->h, AV_PIX_FMT_BGRA,
                            0, NULL, NULL, NULL);
                        if (!is->sub_convert_ctx) {
                            av_log(NULL, AV_LOG_FATAL, "Cannot initialize the conversion context\n");
                            return;
                        }
                        // 纹理数据高效更新，不锁定有可能导致数据污染，部分更新，线程竞争
                        if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)pixels, pitch)) {
                            // 将转换后的数据写入纹理中，避免中间缓存
                            sws_scale(is->sub_convert_ctx, (const uint8_t * const *)sub_rect->data, sub_rect->linesize,
                                      0, sub_rect->h, pixels, pitch);
                            SDL_UnlockTexture(is->sub_texture);
                        }
                    }
                    sp->uploaded = 1;
                }
            } else
                sp = NULL;
        }
    }
    // 3. 视频帧渲染,显示区域计算,适配宽高比
    calculate_display_rect(&rect, is->xleft, is->ytop, is->width, is->height, vp->width, vp->height, vp->sar);
    set_sdl_yuv_conversion_mode(vp->frame);

    // 视频纹理上传。若视频帧未上传（vp->uploaded == 0），调用upload_texture将YUV数据转换为SDL纹理（vid_texture
    if (!vp->uploaded) { // 转换为sdl纹理
        if (upload_texture(&is->vid_texture, vp->frame) < 0) {
            set_sdl_yuv_conversion_mode(NULL);
            return;
        }
        vp->uploaded = 1;
        vp->flip_v = vp->frame->linesize[0] < 0;
    }
    // 使用SDL_RenderCopyEx渲染视频纹理到目标区域
    SDL_RenderCopyEx(renderer, is->vid_texture, NULL, &rect, 0, NULL, vp->flip_v ? SDL_FLIP_VERTICAL : 0);
    set_sdl_yuv_conversion_mode(NULL);
    if (sp) {
    // 字幕叠加渲染，单次渲染模式（USE_ONEPASS_SUBTITLE_RENDER)，直接将整个字幕纹理铺满视频区域
#if USE_ONEPASS_SUBTITLE_RENDER
        SDL_RenderCopy(renderer, is->sub_texture, NULL, &rect);
#else
        //逐区域渲染模式（默认）‌，遍历字幕的每个矩形区域（sp->sub.num_rects），按视频缩放比例调整位置和尺寸后渲染
        int i;
        double xratio = (double)rect.w / (double)sp->width;
        double yratio = (double)rect.h / (double)sp->height;
        for (i = 0; i < sp->sub.num_rects; i++) {
            SDL_Rect *sub_rect = (SDL_Rect*)sp->sub.rects[i];
            SDL_Rect target = {.x = rect.x + sub_rect->x * xratio,
                               .y = rect.y + sub_rect->y * yratio,
                               .w = sub_rect->w * xratio,
                               .h = sub_rect->h * yratio};
            SDL_RenderCopy(renderer, is->sub_texture, sub_rect, &target);
        }
#endif
    }
}

static inline int compute_mod(int a, int b)
{
    return a < 0 ? a%b + b : a%b;
}

// 将音频数据以波形或者频谱的形式显示出来
static void video_audio_display(VideoState *s)
{
    // 定义多个变量用于循环计数、位置计算、延迟计算、通道数等
    int i, i_start, x, y1, y, ys, delay, n, nb_display_channels;
    int ch, channels, h, h2;
    int64_t time_diff;
    int rdft_bits, nb_freq;
    // 在显示的时候，需要一个特定的区域来绘制这些图形，s->height 就代表了这个显示区域在垂直方向上的像素数量
    // 计算合适的 rdft_bits，使得 2 的 rdft_bits 次方接近 2 倍的高度
    // RDFT用于将时域信号转换为频域，生成频谱数据以供显示。变换点数（N）决定了频率分辨率，点数越多，频率分辨率越高，但计算量也越大。
    for (rdft_bits = 1; (1 << rdft_bits) < 2 * s->height; rdft_bits++)
        ;
    // 计算频谱的频率数量
    // 在频谱显示模式下，会从音频样本数组中提取 2 * nb_freq 个样本作为输入数据进行实数离散傅里叶变换
    // 这些频率分量代表了音频信号在不同频率上的能量分布。nb_freq 越大，频谱的分辨率就越高，能够更细致地展示音频信号的频率成分；
    // 反之，nb_freq 越小，频谱的分辨率就越低，不同频率成分可能会相互重叠。
    nb_freq = 1 << (rdft_bits - 1);

    /* compute display index : center on currently output samples */
    // 获取音频目标通道布局中的通道数
    channels = s->audio_tgt.ch_layout.nb_channels;
    // 初始化要显示的通道数
    nb_display_channels = channels;
    if (!s->paused) {
        // 根据显示模式确定使用的数据量
        // 如果当前是波形显示模式，data_used 会被设置为显示区域的宽度，后续可能会根据这个宽度来遍历音频样本数组并绘制波形；
        // 如果是其他模式（如频谱显示模式），data_used 会被设置为 2 * nb_freq，
        int data_used = s->show_mode == SHOW_MODE_WAVES ? s->width : (2 * nb_freq);
        // 每个样本的字节数
        n = 2 * channels;
        // 音频写入缓冲区的大小
        delay = s->audio_write_buf_size;
        // 计算延迟的样本数
        delay /= n;

        /* to be more precise, we take into account the time spent since
           the last buffer computation */
        if (audio_callback_time) {
            // 计算自上次音频回调以来的时间差
            time_diff = av_gettime_relative() - audio_callback_time;
            // 根据时间差调整延迟
            delay -= (time_diff * s->audio_tgt.freq) / 1000000;
        }

        // 增加延迟以确保有足够的数据显示
        delay += 2 * data_used;
        if (delay < data_used)
            delay = data_used;

        // 计算起始索引
        i_start = x = compute_mod(s->sample_array_index - delay * channels, SAMPLE_ARRAY_SIZE);
        if (s->show_mode == SHOW_MODE_WAVES) {
            // 初始化最大得分
            h = INT_MIN;
            for (i = 0; i < 1000; i += channels) {
                // 计算样本数组的索引
                int idx = (SAMPLE_ARRAY_SIZE + x - i) % SAMPLE_ARRAY_SIZE;
                // 获取不同位置的样本值
                int a = s->sample_array[idx];
                int b = s->sample_array[(idx + 4 * channels) % SAMPLE_ARRAY_SIZE];
                int c = s->sample_array[(idx + 5 * channels) % SAMPLE_ARRAY_SIZE];
                int d = s->sample_array[(idx + 9 * channels) % SAMPLE_ARRAY_SIZE];
                // 计算得分
                int score = a - d;
                // 如果得分更高且满足条件，则更新最大得分和起始索引
                if (h < score && (b ^ c) < 0) {
                    h = score;
                    i_start = idx;
                }
            }
        }

        // 记录最后一次的起始索引
        s->last_i_start = i_start;
    } else {
        // 如果处于暂停状态，使用上次记录的起始索引
        i_start = s->last_i_start;
    }

    if (s->show_mode == SHOW_MODE_WAVES) {  // 波形模式：直接绘制原始音频波形
        // 设置渲染颜色为白色
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

        /* total height for one channel */
        // 计算每个通道的总高度
        h = s->height / nb_display_channels;
        /* graph height / 2 */
        // 计算波形图高度的一半
        h2 = (h * 9) / 20;
        for (ch = 0; ch < nb_display_channels; ch++) {
            // 初始化样本数组的索引
            i = i_start + ch;
            // 计算中心线的位置
            y1 = s->ytop + ch * h + (h / 2); /* position of center line */
            for (x = 0; x < s->width; x++) {
                // 计算波形的高度
                y = (s->sample_array[i] * h2) >> 15;
                if (y < 0) {
                    // 如果高度为负，取绝对值并调整绘制位置
                    y = -y;
                    ys = y1 - y;
                } else {
                    ys = y1;
                }
                // 绘制矩形表示波形
                fill_rectangle(s->xleft + x, ys, 1, y);
                // 更新样本数组的索引
                i += channels;
                if (i >= SAMPLE_ARRAY_SIZE)
                    i -= SAMPLE_ARRAY_SIZE;
            }
        }

        // 设置渲染颜色为蓝色
        SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);

        for (ch = 1; ch < nb_display_channels; ch++) {
            // 计算分割线的位置
            y = s->ytop + ch * h;
            // 绘制分割线
            fill_rectangle(s->xleft, y, s->width, 1);
        }
    } else { // 频谱模式：使用实数傅里叶变换(RDFT)生成频谱图
        int err = 0;
        // 重新分配纹理
        if (realloc_texture(&s->vis_texture, SDL_PIXELFORMAT_ARGB8888, s->width, s->height, SDL_BLENDMODE_NONE, 1) < 0)
            return;

        if (s->xpos >= s->width)
            // 如果 x 位置超出宽度，重置为 0
            s->xpos = 0;
        // 限制显示的通道数
        nb_display_channels = FFMIN(nb_display_channels, 2);
        if (rdft_bits != s->rdft_bits) {
            // RDFT 缩放因子
            const float rdft_scale = 1.0;
            // 释放之前的 RDFT 资源
            av_tx_uninit(&s->rdft);
            av_freep(&s->real_data);
            av_freep(&s->rdft_data);
            // 更新 rdft_bits
            s->rdft_bits = rdft_bits;
            // 分配新的实部数据缓冲区
            s->real_data = av_malloc_array(nb_freq, 4 * sizeof(*s->real_data));
            // 分配新的 RDFT 数据缓冲区
            s->rdft_data = av_malloc_array(nb_freq + 1, 2 * sizeof(*s->rdft_data));
            // 初始化 RDFT
            err = av_tx_init(&s->rdft, &s->rdft_fn, AV_TX_FLOAT_RDFT,
                             0, 1 << rdft_bits, &rdft_scale, 0);
        }
        if (err < 0 || !s->rdft_data) {
            // 如果初始化失败或缓冲区分配失败，切换到波形显示模式
            av_log(NULL, AV_LOG_ERROR, "Failed to allocate buffers for RDFT, switching to waves display\n");
            s->show_mode = SHOW_MODE_WAVES;
        } else {
            float *data_in[2];
            AVComplexFloat *data[2];
            // 定义要更新的矩形区域
            SDL_Rect rect = {.x = s->xpos, .y = 0, .w = 1, .h = s->height};
            uint32_t *pixels;
            int pitch;
            for (ch = 0; ch < nb_display_channels; ch++) {
                // 初始化输入数据指针
                data_in[ch] = s->real_data + 2 * nb_freq * ch;
                // 初始化输出数据指针
                data[ch] = s->rdft_data + nb_freq * ch;
                // 初始化样本数组的索引
                i = i_start + ch;
                for (x = 0; x < 2 * nb_freq; x++) {
                    // 应用窗函数
                    double w = (x - nb_freq) * (1.0 / nb_freq);
                    data_in[ch][x] = s->sample_array[i] * (1.0 - w * w);
                    // 更新样本数组的索引
                    i += channels;
                    if (i >= SAMPLE_ARRAY_SIZE)
                        i -= SAMPLE_ARRAY_SIZE;
                }
                // 执行 RDFT 变换
                s->rdft_fn(s->rdft, data[ch], data_in[ch], sizeof(float));
                // 处理 RDFT 结果
                data[ch][0].im = data[ch][nb_freq].re;
                data[ch][nb_freq].re = 0;
            }
            /* Least efficient way to do this, we should of course
             * directly access it but it is more than fast enough. */
            if (!SDL_LockTexture(s->vis_texture, &rect, (void **)&pixels, &pitch)) {
                // 计算像素间距
                pitch >>= 2;
                // 移动到纹理的最后一行
                pixels += pitch * s->height;
                for (y = 0; y < s->height; y++) {
                    // 计算幅度
                    double w = 1 / sqrt(nb_freq);
                    int a = sqrt(w * sqrt(data[0][y].re * data[0][y].re + data[0][y].im * data[0][y].im));
                    int b = (nb_display_channels == 2) ? sqrt(w * hypot(data[1][y].re, data[1][y].im))
                                                        : a;
                    // 限制幅度在 0 - 255 之间
                    a = FFMIN(a, 255);
                    b = FFMIN(b, 255);
                    // 移动到上一行
                    pixels -= pitch;
                    // 设置像素颜色
                    *pixels = (a << 16) + (b << 8) + ((a + b) >> 1);
                }
                // 解锁纹理
                SDL_UnlockTexture(s->vis_texture);
            }
            // 将纹理复制到渲染目标
            SDL_RenderCopy(renderer, s->vis_texture, NULL, NULL);
        }
        if (!s->paused)
            // 如果未暂停，更新 x 位置
            s->xpos++;
    }
}

// 此函数是多媒体播放器中流管理的核心组件，通过分类型资源释放和状态重置，确保系统资源的合理回收与播放状态的稳定性
static void stream_component_close(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;  // 输入媒体文件的格式上下文
    AVCodecParameters *codecpar;   // 流的编解码参数

    // 1. **参数有效性检查**：防止越界访问流数组
    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return;

    // 2. **获取流的编解码参数**
    codecpar = ic->streams[stream_index]->codecpar;

    // 3. **根据流类型释放资源**
    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_AUDIO:  // 处理音频流
        decoder_abort(&is->auddec, &is->sampq);  // 终止音频解码线程，清空采样队列
        SDL_CloseAudioDevice(audio_dev);         // 关闭SDL音频设备（停止播放）
        decoder_destroy(&is->auddec);            // 销毁音频解码器上下文
        swr_free(&is->swr_ctx);                  // 释放音频重采样器（SWResample）
        av_freep(&is->audio_buf1);               // 释放音频缓冲区1的内存
        is->audio_buf1_size = 0;                 // 重置缓冲区大小
        is->audio_buf = NULL;                    // 重置音频数据指针

        // 释放傅里叶变换相关资源（如音频频谱分析）
        if (is->rdft) {
            av_tx_uninit(&is->rdft);       // 销毁FFT变换器
            av_freep(&is->real_data);     // 释放实数数据缓冲区
            av_freep(&is->rdft_data);     // 释放FFT结果缓冲区
            is->rdft = NULL;              // 重置指针
            is->rdft_bits = 0;            // 重置FFT位数
        }
        break;

    case AVMEDIA_TYPE_VIDEO:  // 处理视频流
        decoder_abort(&is->viddec, &is->pictq);  // 终止视频解码线程，清空图像队列
        decoder_destroy(&is->viddec);            // 销毁视频解码器上下文
        break;

    case AVMEDIA_TYPE_SUBTITLE:  // 处理字幕流
        decoder_abort(&is->subdec, &is->subpq);  // 终止字幕解码线程，清空字幕队列
        decoder_destroy(&is->subdec);             // 销毁字幕解码器上下文
        break;

    default:
        break;
    }

    // 4. **标记流为丢弃状态**：通知FFmpeg后续忽略此流的所有数据包
    ic->streams[stream_index]->discard = AVDISCARD_ALL;

    // 5. **重置播放器的流状态**
    switch (codecpar->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        is->audio_st = NULL;       // 清空音频流指针
        is->audio_stream = -1;    // 重置音频流索引（-1表示未选择）
        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_st = NULL;      // 清空视频流指针
        is->video_stream = -1;    // 重置视频流索引
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_st = NULL;   // 清空字幕流指针
        is->subtitle_stream = -1; // 重置字幕流索引
        break;
    default:
        break;
    }
}
/* 此函数通过层级式资源释放和线程同步机制，确保多媒体播放器的安全退出。
   其核心设计思想是 反向初始化（逆序释放资源），避免依赖残留状态
*/
static void stream_close(VideoState *is)
{
    /* 1. 终止解复用线程（read_tid）*/
    is->abort_request = 1;                // 设置全局终止标志，通知线程退出
    SDL_WaitThread(is->read_tid, NULL);   // 等待解复用线程安全退出 阻塞当前线程 直到等待的线程结束

    /* 2. 关闭各类型流组件（音频、视频、字幕）*/
    if (is->audio_stream >= 0)
        stream_component_close(is, is->audio_stream);
    if (is->video_stream >= 0)
        stream_component_close(is, is->video_stream);
    if (is->subtitle_stream >= 0)
        stream_component_close(is, is->subtitle_stream);

    /* 3. 释放输入媒体文件的格式上下文 */
    avformat_close_input(&is->ic);        // 关闭输入流，释放 AVFormatContext 及相关资源

    /* 4. 销毁数据包队列 */
    packet_queue_destroy(&is->videoq);    // 清空并销毁视频包队列
    packet_queue_destroy(&is->audioq);    // 清空并销毁音频包队列
    packet_queue_destroy(&is->subtitleq); // 清空并销毁字幕包队列

    /* 5. 销毁帧队列 */
    frame_queue_destroy(&is->pictq);      // 释放视频帧队列内存
    frame_queue_destroy(&is->sampq);      // 释放音频采样队列内存
    frame_queue_destroy(&is->subpq);      // 释放字幕帧队列内存

    /* 6. 销毁同步对象与附加资源 */
    SDL_DestroyCond(is->continue_read_thread); // 销毁条件变量（用于线程通信）
    sws_freeContext(is->sub_convert_ctx);      // 释放字幕图像转换器（如缩放/色彩转换）

    /* 7. 释放文件名和纹理资源 */
    av_free(is->filename);                // 释放媒体文件名字符串内存
    if (is->vis_texture)
        SDL_DestroyTexture(is->vis_texture); // 销毁可视化纹理（如频谱图）
    if (is->vid_texture)
        SDL_DestroyTexture(is->vid_texture); // 销毁视频纹理
    if (is->sub_texture)
        SDL_DestroyTexture(is->sub_texture); // 销毁字幕纹理

    /* 8. 释放 VideoState 结构体自身 */
    av_free(is);                          // 最后释放播放器全局状态内存
}

/*
  do_exit 函数用于 安全终止程序并释放所有已分配的资源，包括媒体流、渲染器、窗口、
  编解码器参数等，确保无内存泄漏和资源残留
*/
static void do_exit(VideoState *is)
{
    // 1. 关闭媒体流及相关资源
    if (is) {
        stream_close(is); // 调用stream_close关闭视频状态（解复用器、解码器、队列等）
    }

    // 2. 销毁SDL图形渲染器
    if (renderer)
        SDL_DestroyRenderer(renderer); // 释放SDL的2D渲染器（如OpenGL/Direct3D后端）

    // 3. 销毁Vulkan渲染器（如果使用）
    if (vk_renderer)
        vk_renderer_destroy(vk_renderer); // 自定义Vulkan渲染器销毁逻辑（释放GPU资源）

    // 4. 销毁SDL窗口
    if (window)
        SDL_DestroyWindow(window); // 关闭应用程序窗口，释放关联的Surface/句柄

    // 5. 反初始化全局选项（如FFmpeg参数）
    uninit_opts(); // 假设该函数重置全局配置或释放选项相关内存

    // 6. 释放视频滤镜列表内存
    for (int i = 0; i < nb_vfilters; i++)
        av_freep(&vfilters_list[i]); // 释放每个滤镜字符串（av_freep避免悬空指针）
    av_freep(&vfilters_list);        // 释放滤镜列表数组

    // 7. 释放编解码器名称参数
    av_freep(&video_codec_name);    // 释放视频编解码器名称字符串
    av_freep(&audio_codec_name);    // 释放音频编解码器名称字符串
    av_freep(&subtitle_codec_name); // 释放字幕编解码器名称字符串

    // 8. 释放输入文件名
    av_freep(&input_filename); // 释放媒体文件路径字符串

    // 9. 反初始化网络模块（如HTTP、RTMP协议）
    avformat_network_deinit(); // 清理FFmpeg网络库的全局资源（若之前调用过avformat_network_init）

    // 10. 控制台状态显示处理
    if (show_status)
        printf("\n"); // 若启用了状态显示，打印换行符以美化输出（如进度条结束）

    // 11. 关闭SDL子系统
    SDL_Quit(); // 销毁所有SDL模块（音频、视频、事件等），必须调用以避免资源泄漏

    // 12. 静默FFmpeg日志输出
    av_log(NULL, AV_LOG_QUIET, "%s", ""); // 设置日志级别为QUIET，抑制退出时的无关日志

    // 13. 终止进程
    exit(0); // 立即退出程序，返回状态码0（正常退出）
}

/* SIGTERM 信号处理函数
 * - sig : 接收到的信号编号（虽未使用但保留参数以符合信号处理函数原型）
 * 功能：当进程收到 SIGTERM 信号时，以状态码 123 退出程序
 * 特性：
 *   1. static 限定符 - 限制函数作用域仅在当前编译单元可见
 *   2. 直接退出 - 适用于需要快速终止的场景，不执行额外清理
 *   3. 非标准退出码 - 123 可用于标识特定退出原因
 * 典型应用场景：
 *   - 容器化环境（如 Docker）发送 SIGTERM 时优雅退出
 *   - 进程监控系统发送终止指令后记录特殊状态
 * 注意事项：
 *   - 信号处理函数中应避免复杂操作（如 malloc 等非异步安全函数）
 *   - 若需要资源清理，应在调用 exit() 前完成
 */
static void sigterm_handler(int sig)
{
    exit(123); // 立即终止进程，返回状态码 123 给父进程
}

/**
 * 设置默认窗口尺寸（考虑屏幕约束和像素宽高比）
 *
 * @param width   原始内容宽度（像素）
 * @param height  原始内容高度（像素）
 * @param sar     像素宽高比（AVRational结构体，sar = 宽/高）
 *
 * 处理逻辑：
 * 1. 确定最大允许尺寸：优先使用全局设定的屏幕尺寸，无限制时使用INT_MAX
 * 2. 当屏幕尺寸完全无约束时，将最大高度设为原始高度作为参考基准
 * 3. 通过像素宽高比计算实际显示区域（自动适应屏幕约束）
 * 4. 将计算结果设为默认窗口尺寸
 *
 * 特性：
 * - static 限制函数作用域，属于模块内部实现
 * - 依赖全局变量 screen_width/screen_height 获取屏幕约束
 * - 调用 calculate_display_rect 实现核心布局计算
 */
static void set_default_window_size(int width, int height, AVRational sar)
{
    SDL_Rect rect;

    // 获取屏幕最大尺寸约束（未配置时使用INT_MAX表示无限制）
    int max_width  = screen_width  ? screen_width  : INT_MAX;
    int max_height = screen_height ? screen_height : INT_MAX;

    // 特殊处理：当屏幕尺寸完全无约束时，使用原始高度作为参考基准
    if (max_width == INT_MAX && max_height == INT_MAX)
        max_height = height; // 避免两个维度都无限大导致布局计算异常

    // 计算符合宽高比且适配屏幕约束的显示区域
    calculate_display_rect(&rect,
        0, 0,            // 显示位置起始坐标（左上角）
        max_width, max_height,  // 最大允许尺寸
        width, height,    // 原始内容尺寸
        sar);             // 像素宽高比

    // 设置默认窗口尺寸
    default_width  = rect.w;  // 计算后的显示宽度
    default_height = rect.h;  // 计算后的显示高度
}

static int video_open(VideoState *is)
{
    int w,h;

    w = screen_width ? screen_width : default_width;
    h = screen_height ? screen_height : default_height;

    if (!window_title)
        window_title = input_filename;
    SDL_SetWindowTitle(window, window_title);

    SDL_SetWindowSize(window, w, h);
    SDL_SetWindowPosition(window, screen_left, screen_top);
    if (is_full_screen)
        SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
    SDL_ShowWindow(window);

    is->width  = w;
    is->height = h;

    return 0;
}

/* display the current picture, if any */
static void video_display(VideoState *is)
{
    if (!is->width)
        video_open(is);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // 绘制颜色为黑色
    SDL_RenderClear(renderer);
    if (is->audio_st && is->show_mode != SHOW_MODE_VIDEO)
        video_audio_display(is);
    else if (is->video_st)
        video_image_display(is);
    SDL_RenderPresent(renderer);
}

/* 时钟结构体定义（推测） */
typedef struct Clock {
    double pts;          // 当前显示时间戳（Presentation Timestamp）
    double pts_drift;    // PTS与系统时间的差值（用于速度调整）
    double last_updated; // 最后一次更新时间（秒为单位）
    int *queue_serial;   // 指向数据包队列序列号的指针（用于同步校验）
    int serial;          // 当前时钟的序列号
    double speed;        // 播放速度（1.0=正常，2.0=2倍速）
    int paused;          // 暂停状态（1=暂停，0=运行）
} Clock;

/**
 * 获取时钟当前时间（考虑速度和暂停状态）
 * @param c Clock指针
 * @return 有效时返回当前时间（秒），否则返回NAN（表示不同步或无效）
 */
static double get_clock(Clock *c) {
    /* 检查序列号是否匹配（确保操作的是最新数据） */
    if (*c->queue_serial != c->serial)
        return NAN; // 数据队列已更新，当前时钟数据已过期

    /* 暂停状态直接返回最后记录的PTS */
    if (c->paused) {
        return c->pts;
    } else {
        /* 计算时间漂移后的实际时间：
         * 1. 获取当前相对时间（微秒转秒）
         * 2. 计算公式：c->pts_drift + time - (time - last_updated) * (1.0 - speed)
         *    等效于：last_pts + (current_time - last_updated) * speed
         *    实现变速播放时的时间线性插值
         */
        double time = av_gettime_relative() / 1000000.0;
        return c->pts_drift + time - (time - c->last_updated) * (1.0 - c->speed);
    }
}

/**
 * 在指定时间点设置时钟参数（内部函数）
 * @param c      Clock指针
 * @param pts    新的显示时间戳
 * @param serial 新序列号
 * @param time   设置时间点（系统时间，秒为单位） 存储的是系统绝对时间
 */
static void set_clock_at(Clock *c, double pts, int serial, double time) {
    c->pts = pts;                   // 记录原始PTS
    c->last_updated = time;         // 更新最后操作时间戳
    c->pts_drift = c->pts - time;   // 计算PTS与系统时间的初始差值
    c->serial = serial;             // 更新序列号（标识数据版本）
}

/**
 * 在当前时间设置时钟参数（外部接口）
 * @param c      Clock指针
 * @param pts    新的显示时间戳
 * @param serial 新序列号
 */
static void set_clock(Clock *c, double pts, int serial) {
    // 获取当前系统时间（秒级精度）并调用内部函数 系统启动或程序运行时间
    double time = av_gettime_relative() / 1000000.0; // 系统启动或程序运行时间
    set_clock_at(c, pts, serial, time);
}

/**
 * 调整时钟播放速度
 * @param c     Clock指针
 * @param speed 新速度值（必须>0）
 */
static void set_clock_speed(Clock *c, double speed) {
    /* 关键操作：
     * 1. 基于当前速度重新计算时钟参数
     * 2. 更新速度值（避免跳跃式变化）
     */
    set_clock(c, get_clock(c), c->serial); // 用当前时间重新锚定
    c->speed = speed;                      // 应用新速度
}

/**
 * 初始化时钟结构体
 * @param c             Clock指针
 * @param queue_serial  关联的数据包队列序列号指针
 */
static void init_clock(Clock *c, int *queue_serial) {
    c->speed = 1.0;             // 默认正常速度
    c->paused = 0;              // 初始非暂停状态
    c->queue_serial = queue_serial; // 绑定队列序列号
    set_clock(c, NAN, -1);      // 初始化为无效状态（时间=NAN，序列号=-1）
}

/**
 * 将主时钟同步到从时钟
 *
 * 当从时钟有效且满足以下任一条件时，强制将主时钟设置为从时钟的值：
 * 1. 主时钟当前无效（NaN）
 * 2. 主从时钟差值超过同步阈值（NOSYNC_THRESHOLD）
 *
 * @param c     主时钟（需要被同步的时钟）
 * @param slave 从时钟（作为参考源的时钟）
 */
static void sync_clock_to_slave(Clock *c, Clock *slave)
{
    // 获取主时钟当前值
    double clock = get_clock(c);
    // 获取从时钟当前值
    double slave_clock = get_clock(slave);

    // 同步条件：
    // 1. 从时钟值有效（非NaN）
    // 2. 且（主时钟无效 或 主从时钟差值超过阈值）
    if (!isnan(slave_clock) && (isnan(clock) || fabs(clock - slave_clock) > AV_NOSYNC_THRESHOLD))
        // 强制将主时钟设置为从时钟的值，并继承从时钟的序列号
        set_clock(c, slave_clock, slave->serial);
}

static int get_master_sync_type(VideoState *is) {
    if (is->av_sync_type == AV_SYNC_VIDEO_MASTER) {
        if (is->video_st)
            return AV_SYNC_VIDEO_MASTER;
        else
            return AV_SYNC_AUDIO_MASTER;
    } else if (is->av_sync_type == AV_SYNC_AUDIO_MASTER) {
        if (is->audio_st)
            return AV_SYNC_AUDIO_MASTER;
        else
            return AV_SYNC_EXTERNAL_CLOCK;
    } else {
        return AV_SYNC_EXTERNAL_CLOCK;
    }
}

/* get the current master clock value */
static double get_master_clock(VideoState *is)
{
    double val;

    switch (get_master_sync_type(is)) {
        case AV_SYNC_VIDEO_MASTER:
            val = get_clock(&is->vidclk);
            break;
        case AV_SYNC_AUDIO_MASTER:
            val = get_clock(&is->audclk);
            break;
        default:
            val = get_clock(&is->extclk);
            break;
    }
    return val;
}

// 播放器核心的同步控制机制，通过动态调整时钟速度，在保证流畅播放的前提下，
// 最大限度维持音视频同步和实时性 通过缓冲区状态反馈实现自适应速率控制
static void check_external_clock_speed(VideoState *is) {
   // 情况1：缓冲区不足时降速
   if (is->video_stream >= 0 && is->videoq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES ||
       is->audio_stream >= 0 && is->audioq.nb_packets <= EXTERNAL_CLOCK_MIN_FRAMES) {
       // 计算新速度 = max(最小速度, 当前速度-步长)
       set_clock_speed(&is->extclk, FFMAX(EXTERNAL_CLOCK_SPEED_MIN,
                                         is->extclk.speed - EXTERNAL_CLOCK_SPEED_STEP));
   }

   // 情况2：缓冲区充足时加速
   else if ((is->video_stream < 0 || is->videoq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES) &&
            (is->audio_stream < 0 || is->audioq.nb_packets > EXTERNAL_CLOCK_MAX_FRAMES)) {
       // 计算新速度 = min(最大速度, 当前速度+步长)
       set_clock_speed(&is->extclk, FFMIN(EXTERNAL_CLOCK_SPEED_MAX,
                                         is->extclk.speed + EXTERNAL_CLOCK_SPEED_STEP));
   }

   // 情况3：缓冲区正常时向基准速度回归
   else {
       double speed = is->extclk.speed;
       if (speed != 1.0) {
           // 智能回归公式：根据当前速度与1.0的差距方向调整
           set_clock_speed(&is->extclk,
                          speed + EXTERNAL_CLOCK_SPEED_STEP *
                          (1.0 - speed) / fabs(1.0 - speed));
       }
   }
}

/* seek in the stream */
static void stream_seek(VideoState *is, int64_t pos, int64_t rel, int by_bytes)
{
    // 检查是否已有未处理的跳转请求
    if (!is->seek_req) {
        // 设置跳转目标位置（绝对位置）
        is->seek_pos = pos;

        // 设置相对偏移量（相对于pos的偏移）
        is->seek_rel = rel;  // 实际跳转位置=pos+rel

        // 清除字节跳转标志（准备重新设置）
        is->seek_flags &= ~AVSEEK_FLAG_BYTE; // 还有向后寻找关键帧的方式 AVSEEK_FLAG_BACKWARD

        // 如果要求按字节跳转，设置相应标志位
        if (by_bytes)
            is->seek_flags |= AVSEEK_FLAG_BYTE;

        // 设置跳转请求标志（通知后台线程）
        is->seek_req = 1;

        // 唤醒阻塞的读取线程
        SDL_CondSignal(is->continue_read_thread);
    }
}

/* 暂停或恢复视频播放 */
static void stream_toggle_pause(VideoState *is)
{
    // 如果当前处于暂停状态（即将恢复播放）
    if (is->paused) {
        // 更新帧定时器：补偿暂停期间的时间差
        is->frame_timer += av_gettime_relative() / 1000000.0 - is->vidclk.last_updated;

        // 特殊处理：非系统级暂停时恢复视频时钟
        if (is->read_pause_return != AVERROR(ENOSYS)) {
            is->vidclk.paused = 0;
        }

        // 更新视频时钟为当前值（保持连续性）
        set_clock(&is->vidclk, get_clock(&is->vidclk), is->vidclk.serial);
    }

    // 更新外部时钟为当前值
    set_clock(&is->extclk, get_clock(&is->extclk), is->extclk.serial);

    // 切换所有时钟的暂停状态
    is->paused = is->audclk.paused = is->vidclk.paused = is->extclk.paused = !is->paused;
}

/* 切换暂停状态（外部接口） */
static void toggle_pause(VideoState *is)
{
    // 调用核心暂停/恢复逻辑
    stream_toggle_pause(is);

    // 重置单帧步进标志
    is->step = 0;
}

/* 切换静音状态 */
// 单行实现状态翻转
// 实际静音操作在音频播放线程处理
static void toggle_mute(VideoState *is)
{
    // 翻转静音状态标志
    is->muted = !is->muted;
}

/* 更新音频音量 */
static void update_volume(VideoState *is, int sign, double step)
{
    // 1. 计算当前音量对应的分贝值
    double volume_level = is->audio_volume ?
        (20 * log(is->audio_volume / (double)SDL_MIX_MAXVOLUME) / log(10)) :
        -1000.0;

    // 2. 计算新的线性音量值
    int new_volume = lrint(SDL_MIX_MAXVOLUME * pow(10.0, (volume_level + sign * step) / 20.0));

    // 3. 应用并裁剪音量值
    is->audio_volume = av_clip(
        is->audio_volume == new_volume ?
            (is->audio_volume + sign) :  // 特殊处理：防止卡在相同值
            new_volume,
        0,
        SDL_MIX_MAXVOLUME
    );
}

/* 步进到下一帧 */
static void step_to_next_frame(VideoState *is)
{
    // 如果当前处于暂停状态，则先解除暂停
    if (is->paused)
        stream_toggle_pause(is);

    // 设置单帧步进标志
    is->step = 1;
}

/**
 * 计算目标视频帧延迟时间
 * @param delay 当前视频帧的原始延迟时间（单位：秒）
 * @param is    视频状态对象指针
 * @return      调整后的目标延迟时间
 */
static double compute_target_delay(double delay, VideoState *is)
{
    double sync_threshold, diff = 0;

    /* 如果视频不是主同步源（即视频作为从属同步源） */
    if (get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER) {
        /* 计算视频时钟与主时钟的差值（视频落后主时钟时diff为正） 视频时间与音频时间的差值*/
        diff = get_clock(&is->vidclk) - get_master_clock(is);

        /* 动态计算同步阈值：
         * - 确保阈值在AV_SYNC_THRESHOLD_MIN和AV_SYNC_THRESHOLD_MAX之间
         * - 同时考虑当前帧延迟delay作为参考值
         */
        sync_threshold = FFMAX(AV_SYNC_THRESHOLD_MIN,
                              FFMIN(AV_SYNC_THRESHOLD_MAX, delay));

        /* 仅当差值有效且小于最大帧持续时间时才进行调整 */
        if (!isnan(diff) && fabs(diff) < is->max_frame_duration) {
            /* 情况1：视频落后超过阈值（diff为负） */
            if (diff <= -sync_threshold) {
                // 减少延迟：当前延迟 + 负差值（相当于减少延迟值）
                delay = FFMAX(0, delay + diff);
            }
            /* 情况2：视频超前超过阈值且当前延迟较大 */
            else if (diff >= sync_threshold && delay > AV_SYNC_FRAMEDUP_THRESHOLD) {
                // 直接增加延迟补偿差值
                delay = delay + diff;
            }
            /* 情况3：视频超前超过阈值但当前延迟较小 */
            else if (diff >= sync_threshold) {
                // 安全策略：将延迟加倍（避免过度跳帧）
                delay = 2 * delay;
            }
        }
    }

    /* 记录跟踪日志：显示最终延迟和音视频差值 */
    av_log(NULL, AV_LOG_TRACE, "video: delay=%0.3f A-V=%f\n", delay, -diff);

    return delay;
}

/**
 * 计算两个连续视频帧之间的有效持续时间
 *
 * @param is     视频状态管理器（包含全局播放信息）
 * @param vp     当前视频帧
 * @param nextvp 下一视频帧
 * @return       计算得到的帧间持续时间（秒）
 */
static double vp_duration(VideoState *is, Frame *vp, Frame *nextvp) {
    // 检查两帧是否属于同一个解码序列（serial变化说明发生了seek或流切换）
    if (vp->serial == nextvp->serial) {
        // 基于时间戳计算理论持续时间：下一帧PTS - 当前帧PTS
        double duration = nextvp->pts - vp->pts;

        /* 有效性检查（三重保护）：
         * 1. isnan(duration)   -> 时间戳值异常（非数字）
         * 2. duration <= 0      -> 时间戳不递增（错误或B帧乱序）
         * 3. duration > max_frame_duration -> 超过阈值（异常大间隔）
         */
        if (isnan(duration) || duration <= 0 || duration > is->max_frame_duration)
            // 使用当前帧自带的默认持续时间（通常基于帧率计算）
            return vp->duration;
        else
            // 返回计算出的有效时间间隔
            return duration;
    } else {
        // 序列号不匹配（如seek后），返回0表示非连续帧
        return 0.0;
    }
}

/**
 * 更新视频时钟系统
 *
 * @param is     视频状态管理器（包含时钟系统）
 * @param pts    新视频帧的显示时间戳（单位：秒）
 * @param serial 当前帧的序列号（用于seek检测）
 */
static void update_video_pts(VideoState *is, double pts, int serial)
{
    /* 更新主视频时钟 */
    set_clock(&is->vidclk, pts, serial);

    /* 将外部参考时钟同步到视频时钟 */
    sync_clock_to_slave(&is->extclk, &is->vidclk);
}

/* called to display each frame */
static void video_refresh(void *opaque, double *remaining_time)
{
    VideoState *is = opaque;
    double time;

    Frame *sp, *sp2;

    // is->realtime 实时播放模式
    if (!is->paused && get_master_sync_type(is) == AV_SYNC_EXTERNAL_CLOCK && is->realtime)
        check_external_clock_speed(is);

    // 非视频模式 仅显示音频
    if (!display_disable && is->show_mode != SHOW_MODE_VIDEO && is->audio_st) {
        time = av_gettime_relative() / 1000000.0;
        // 强制刷新标志(外部事件)触发立即渲染 和距离上次刷新已超过设定间隔
        // is->last_vis_time 记录了上次渲染的时间 rdftspeed 音频可视化的刷新速度
        if (is->force_refresh || is->last_vis_time + rdftspeed < time) {
            video_display(is);
            is->last_vis_time = time;
        }
        // 动态计算下次刷新的剩余时间
        *remaining_time = FFMIN(*remaining_time, is->last_vis_time + rdftspeed - time);
    }

    if (is->video_st) {
retry:
        if (frame_queue_nb_remaining(&is->pictq) == 0) {
            // nothing to do, no picture to display in the queue
        } else {
            double last_duration, duration, delay;
            Frame *vp, *lastvp;

            /* dequeue the picture */
            // 获取队列中已经显示的帧和当前帧
            lastvp = frame_queue_peek_last(&is->pictq);
            vp = frame_queue_peek(&is->pictq);

            if (vp->serial != is->videoq.serial) {
                frame_queue_next(&is->pictq);
                goto retry;
            }

            // 序列变化时重置帧计时器
            if (lastvp->serial != vp->serial)
                // PTS 帧在媒体时间轴（Media Timeline）上应该被显示的时刻
                // 记录当前帧在系统时间轴（Real-Time Clock）上的理论播放时刻
                // Seek操作 帧渲染调度 暂停恢复
                is->frame_timer = av_gettime_relative() / 1000000.0;

            if (is->paused)
                goto display;

            // === 帧同步计算 ===
            // 1. 计算上一帧的持续时间
            /* compute nominal last_duration */
            last_duration = vp_duration(is, lastvp, vp);
            delay = compute_target_delay(last_duration, is); // 经音视频同步修正后的实际渲染延迟

            time= av_gettime_relative()/1000000.0;
            if (time < is->frame_timer + delay) { // 未到达显示时间
                *remaining_time = FFMIN(is->frame_timer + delay - time, *remaining_time);
                goto display;
            }

            is->frame_timer += delay; // delay过大 则重置计时器显示时间
            if (delay > 0 && time - is->frame_timer > AV_SYNC_THRESHOLD_MAX)
                is->frame_timer = time;

            // 更新视频时钟
            SDL_LockMutex(is->pictq.mutex);
            if (!isnan(vp->pts))
                update_video_pts(is, vp->pts, vp->serial);
            SDL_UnlockMutex(is->pictq.mutex);

            if (frame_queue_nb_remaining(&is->pictq) > 1) {
                // 获取下一帧的指针（不移动队列指针
                Frame *nextvp = frame_queue_peek_next(&is->pictq);
                // 计算当前帧vp的理论显示时长（基于PTS差值
                duration = vp_duration(is, vp, nextvp);
                // 丢帧条件：非单步模式 强制丢帧 系统时间超过当前帧应结束的时刻
                if(!is->step && (framedrop>0 || (framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER)) && time > is->frame_timer + duration){
                    is->frame_drops_late++;  // 统计延迟丢帧次数
                    frame_queue_next(&is->pictq);  // 将当前帧移出队列
                    goto retry;  // 重新尝试处理下一帧
                }
            }

            // 检查是否存在字幕流
            if (is->subtitle_st) {
                // 遍历字幕帧队列（不移动队列指针）
                while (frame_queue_nb_remaining(&is->subpq) > 0) {
                    // 获取当前队首字幕帧（peek操作不移动指针）
                    sp = frame_queue_peek(&is->subpq);

                    // 检查队列中是否至少存在2帧字幕
                    if (frame_queue_nb_remaining(&is->subpq) > 1)
                        sp2 = frame_queue_peek_next(&is->subpq);  // 获取下一帧（用于预判显示时间）
                    else
                        sp2 = NULL;

                    /* 判断是否应丢弃当前字幕帧的条件：
                       1. 序列号不匹配（说明字幕流已重置）
                       2. 视频时钟超过当前字幕结束时间（sp->pts + end_display_time/1000）
                       3. 视频时钟超过下一字幕开始时间（预判丢弃当前帧以避免重叠）*/
                    if (sp->serial != is->subtitleq.serial
                            || (is->vidclk.pts > (sp->pts + ((float) sp->sub.end_display_time / 1000)))
                            || (sp2 && is->vidclk.pts > (sp2->pts + ((float) sp2->sub.start_display_time / 1000))))
                    {
                        // 如果字幕已上传到纹理（需清理显存）,避免残留渲染
                        if (sp->uploaded) {
                            // 遍历字幕的所有矩形区域
                            for (i = 0; i < sp->sub.num_rects; i++) {
                                AVSubtitleRect *sub_rect = sp->sub.rects[i];
                                uint8_t *pixels;
                                int pitch, j;

                                // 锁定SDL纹理并清空像素数据（透明化处理）
                                if (!SDL_LockTexture(is->sub_texture, (SDL_Rect *)sub_rect, (void **)&pixels, &pitch)) {
                                    for (j = 0; j < sub_rect->h; j++, pixels += pitch)
                                        memset(pixels, 0, sub_rect->w << 2);  // ARGB清0（透明色）
                                    SDL_UnlockTexture(is->sub_texture);
                                }
                            }
                        }
                        frame_queue_next(&is->subpq);  // 正式移出当前帧
                    } else {
                        break;  // 当前帧未过期，终止清理循环
                    }
                }
            }

            frame_queue_next(&is->pictq);
            is->force_refresh = 1; // 强制标记需要立即刷新视频画面，触发SDL渲染器重新绘制
            // 帧丢弃后需要快速显示下一帧 用户交互（如暂停/播放/跳转）后的画面更新

            if (is->step && !is->paused)
                stream_toggle_pause(is); /* 暂停或恢复视频播放 */
        }
display:
        /* display picture */
        if (!display_disable && is->force_refresh && is->show_mode == SHOW_MODE_VIDEO && is->pictq.rindex_shown)
            video_display(is);
    }
    // 周期性打印播放状态信息
    is->force_refresh = 0;  // 重置强制刷新标志

    if (show_status) {      // 检查是否需要显示状态信息
        AVBPrint buf;       // FFmpeg的动态字符串缓冲区
        static int64_t last_time; // 上次刷新时间（静态变量保持状态）
        int64_t cur_time;   // 当前时间
        int aqsize, vqsize, sqsize; // 音频/视频/字幕队列大小
        double av_diff;     // 音视频时钟差值

        // 获取当前相对时间（微秒）
        cur_time = av_gettime_relative();

        // 每30毫秒刷新一次状态（或首次刷新）
        if (!last_time || (cur_time - last_time) >= 30000) {
            // 初始化队列大小
            aqsize = 0;
            vqsize = 0;
            sqsize = 0;

            // 获取各队列的实际大小
            if (is->audio_st) aqsize = is->audioq.size;
            if (is->video_st) vqsize = is->videoq.size;
            if (is->subtitle_st) sqsize = is->subtitleq.size;

            // 计算时钟差值：
            av_diff = 0;
            // 情况1：同时存在音频和视频流 → 计算音频时钟-视频时钟
            if (is->audio_st && is->video_st)
                av_diff = get_clock(&is->audclk) - get_clock(&is->vidclk);
            // 情况2：只有视频流 → 主时钟-视频时钟
            else if (is->video_st)
                av_diff = get_master_clock(is) - get_clock(&is->vidclk);
            // 情况3：只有音频流 → 主时钟-音频时钟
            else if (is->audio_st)
                av_diff = get_master_clock(is) - get_clock(&is->audclk);

            // 初始化动态字符串缓冲区
            av_bprint_init(&buf, 0, AV_BPRINT_SIZE_AUTOMATIC);

            // 格式化状态行（使用回车符\r实现原地刷新）：
            av_bprintf(&buf,
                      "%7.2f %s:%7.3f fd=%4d aq=%5dKB vq=%5dKB sq=%5dB \r",
                      get_master_clock(is),  // 主时钟（播放位置，秒）
                      // 状态标识：
                      (is->audio_st && is->video_st) ? "A-V" :  // 音视频同步模式
                      (is->video_st ? "M-V" :  // 纯视频模式
                      (is->audio_st ? "M-A" :  // 纯音频模式
                      "   ")),  // 无媒体流
                      av_diff,  // 时钟差值
                      is->frame_drops_early + is->frame_drops_late, // 总丢帧数
                      aqsize / 1024,  // 音频队列大小(KB)
                      vqsize / 1024,  // 视频队列大小(KB)
                      sqsize);         // 字幕队列大小(字节)

            // 选择输出方式：
            if (show_status == 1 && AV_LOG_INFO > av_log_get_level())
                fprintf(stderr, "%s", buf.str);  // 直接输出到stderr
            else
                av_log(NULL, AV_LOG_INFO, "%s", buf.str); // 通过FFmpeg日志系统输出

            fflush(stderr);  // 强制刷新标准错误输出
            av_bprint_finalize(&buf, NULL);  // 释放缓冲区

            last_time = cur_time;  // 更新最后刷新时间
        }
    }
}

// 视频渲染管道的核心环节，连接解码线程与显示线程
// 将解码后的视频帧加入图像队列
static int queue_picture(VideoState *is, AVFrame *src_frame, double pts, double duration, int64_t pos, int serial)
{
    Frame *vp;  // 指向队列中可写帧的指针

    // 调试信息：输出帧类型和显示时间戳
#if defined(DEBUG_SYNC)
    printf("frame_type=%c pts=%0.3f\n",
           av_get_picture_type_char(src_frame->pict_type), pts);
#endif

    // 从图像队列获取一个可写帧位置（如果队列满则返回错误）
    if (!(vp = frame_queue_peek_writable(&is->pictq)))
        return -1;

    // 设置帧的宽高比（Sample Aspect Ratio）
    vp->sar = src_frame->sample_aspect_ratio;

    // 标记该帧尚未上传到显示设备（如GPU纹理）
    vp->uploaded = 0;

    // 设置帧的基本属性
    vp->width = src_frame->width;    // 视频宽度
    vp->height = src_frame->height;  // 视频高度
    vp->format = src_frame->format;  // 像素格式（如YUV420P）

    // 设置帧的时间信息
    vp->pts = pts;         // 显示时间戳（Presentation Time Stamp）
    vp->duration = duration; // 帧持续时间
    vp->pos = pos;          // 在媒体文件中的字节位置
    vp->serial = serial;    // 序列号（用于seek操作后识别新旧帧）

    // 根据视频尺寸和宽高比设置默认窗口大小
    set_default_window_size(vp->width, vp->height, vp->sar);

    // 将源帧数据转移到队列帧（避免复制，高效移动引用）
    av_frame_move_ref(vp->frame, src_frame);

    // 将帧推入队列（更新队列状态）
    frame_queue_push(&is->pictq);

    return 0;  // 成功返回0
}

// 从视频解码器获取一帧视频
static int get_video_frame(VideoState *is, AVFrame *frame)
{
    int got_picture;  // 是否成功获取帧的标志

    // 尝试从视频解码器解码一帧
    if ((got_picture = decoder_decode_frame(&is->viddec, frame, NULL)) < 0)
        return -1;  // 解码出错返回错误

    // 如果成功获取到帧
    if (got_picture) {
        double dpts = NAN;  // 初始化显示时间戳为NaN

        // 如果帧有有效的PTS，转换为秒单位
        if (frame->pts != AV_NOPTS_VALUE)
            dpts = av_q2d(is->video_st->time_base) * frame->pts;

        // 计算帧的宽高比（从容器/流/帧信息中推断）
        frame->sample_aspect_ratio = av_guess_sample_aspect_ratio(is->ic, is->video_st, frame);

        /* 帧丢弃逻辑：当需要主动丢帧时 */
        if (framedrop > 0 || (framedrop && get_master_sync_type(is) != AV_SYNC_VIDEO_MASTER)) {
            // 只在有有效PTS时进行丢帧判断
            if (frame->pts != AV_NOPTS_VALUE) {
                // 计算当前帧与主时钟的差值
                double diff = dpts - get_master_clock(is);

                /* 满足以下所有条件时丢弃当前帧： */
                if (!isnan(diff) &&                // 1. 差值是有效数字
                    fabs(diff) < AV_NOSYNC_THRESHOLD && // 2. 差值在同步阈值内（通常10秒）
                    diff - is->frame_last_filter_delay < 0 && // 3. 帧已经过时（考虑滤镜延迟）
                    is->viddec.pkt_serial == is->vidclk.serial && // 4. 序列号匹配（未发生seek）
                    is->videoq.nb_packets)         // 5. 视频队列中还有后续帧
                {
                    is->frame_drops_early++;  // 增加早期丢帧计数
                    av_frame_unref(frame);    // 释放当前帧
                    got_picture = 0;          // 标记为未获取有效帧
                }
            }
        }
    }

    return got_picture;  // 返回获取帧的状态
}

// FFmpeg 多媒体处理中用于配置滤镜图的核心函数。它负责连接源滤镜和接收滤镜，并根据需要解析自定义滤镜链
static int configure_filtergraph(AVFilterGraph *graph, const char *filtergraph,
                                 AVFilterContext *source_ctx, AVFilterContext *sink_ctx)
{
    int ret, i;
    int nb_filters = graph->nb_filters;
    AVFilterInOut *outputs = NULL, *inputs = NULL;

    if (filtergraph) {
        outputs = avfilter_inout_alloc();
        inputs  = avfilter_inout_alloc();
        if (!outputs || !inputs) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        outputs->name       = av_strdup("in");
        outputs->filter_ctx = source_ctx;
        outputs->pad_idx    = 0;
        outputs->next       = NULL;

        inputs->name        = av_strdup("out");
        inputs->filter_ctx  = sink_ctx;
        inputs->pad_idx     = 0;
        inputs->next        = NULL;

        if ((ret = avfilter_graph_parse_ptr(graph, filtergraph, &inputs, &outputs, NULL)) < 0)
            goto fail;
    } else {
        if ((ret = avfilter_link(source_ctx, 0, sink_ctx, 0)) < 0)
            goto fail;
    }

    /* Reorder the filters to ensure that inputs of the custom filters are merged first */
    for (i = 0; i < graph->nb_filters - nb_filters; i++)
        FFSWAP(AVFilterContext*, graph->filters[i], graph->filters[i + nb_filters]);

    ret = avfilter_graph_config(graph, NULL);
fail:
    avfilter_inout_free(&outputs);
    avfilter_inout_free(&inputs);
    return ret;
}

static int configure_video_filters(AVFilterGraph *graph, VideoState *is, const char *vfilters, AVFrame *frame)
{
    enum AVPixelFormat pix_fmts[FF_ARRAY_ELEMS(sdl_texture_format_map)];
    char sws_flags_str[512] = "";
    int ret;
    AVFilterContext *filt_src = NULL, *filt_out = NULL, *last_filter = NULL;
    AVCodecParameters *codecpar = is->video_st->codecpar;
    AVRational fr = av_guess_frame_rate(is->ic, is->video_st, NULL);
    const AVDictionaryEntry *e = NULL;
    int nb_pix_fmts = 0;
    int i, j;
    AVBufferSrcParameters *par = av_buffersrc_parameters_alloc();

    if (!par)
        return AVERROR(ENOMEM);

    for (i = 0; i < renderer_info.num_texture_formats; i++) {
        for (j = 0; j < FF_ARRAY_ELEMS(sdl_texture_format_map); j++) {
            if (renderer_info.texture_formats[i] == sdl_texture_format_map[j].texture_fmt) {
                pix_fmts[nb_pix_fmts++] = sdl_texture_format_map[j].format;
                break;
            }
        }
    }

    while ((e = av_dict_iterate(sws_dict, e))) {
        if (!strcmp(e->key, "sws_flags")) {
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", "flags", e->value);
        } else
            av_strlcatf(sws_flags_str, sizeof(sws_flags_str), "%s=%s:", e->key, e->value);
    }
    if (strlen(sws_flags_str))
        sws_flags_str[strlen(sws_flags_str)-1] = '\0';

    graph->scale_sws_opts = av_strdup(sws_flags_str);


    filt_src = avfilter_graph_alloc_filter(graph, avfilter_get_by_name("buffer"),
                                           "ffplay_buffer");
    if (!filt_src) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    par->format              = frame->format;
    par->time_base           = is->video_st->time_base;
    par->width               = frame->width;
    par->height              = frame->height;
    par->sample_aspect_ratio = codecpar->sample_aspect_ratio;
    par->color_space         = frame->colorspace;
    par->color_range         = frame->color_range;
    par->frame_rate          = fr;
    par->hw_frames_ctx = frame->hw_frames_ctx;
    ret = av_buffersrc_parameters_set(filt_src, par);
    if (ret < 0)
        goto fail;

    ret = avfilter_init_dict(filt_src, NULL);
    if (ret < 0)
        goto fail;

    filt_out = avfilter_graph_alloc_filter(graph, avfilter_get_by_name("buffersink"),
                                           "ffplay_buffersink");
    if (!filt_out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    if ((ret = av_opt_set_array(filt_out, "pixel_formats", AV_OPT_SEARCH_CHILDREN,
                                0, nb_pix_fmts, AV_OPT_TYPE_PIXEL_FMT, pix_fmts)) < 0)
        goto fail;
    if (!vk_renderer &&
        (ret = av_opt_set_array(filt_out, "colorspaces", AV_OPT_SEARCH_CHILDREN,
                                0, FF_ARRAY_ELEMS(sdl_supported_color_spaces),
                                AV_OPT_TYPE_INT, sdl_supported_color_spaces)) < 0)
        goto fail;

    ret = avfilter_init_dict(filt_out, NULL);
    if (ret < 0)
        goto fail;

    last_filter = filt_out;

/* Note: this macro adds a filter before the lastly added filter, so the
 * processing order of the filters is in reverse */
#define INSERT_FILT(name, arg) do {                                          \
    AVFilterContext *filt_ctx;                                               \
                                                                             \
    ret = avfilter_graph_create_filter(&filt_ctx,                            \
                                       avfilter_get_by_name(name),           \
                                       "ffplay_" name, arg, NULL, graph);    \
    if (ret < 0)                                                             \
        goto fail;                                                           \
                                                                             \
    ret = avfilter_link(filt_ctx, 0, last_filter, 0);                        \
    if (ret < 0)                                                             \
        goto fail;                                                           \
                                                                             \
    last_filter = filt_ctx;                                                  \
} while (0)

    if (autorotate) {
        double theta = 0.0;
        int32_t *displaymatrix = NULL;
        AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_DISPLAYMATRIX);
        if (sd)
            displaymatrix = (int32_t *)sd->data;
        if (!displaymatrix) {
            const AVPacketSideData *psd = av_packet_side_data_get(is->video_st->codecpar->coded_side_data,
                                                                  is->video_st->codecpar->nb_coded_side_data,
                                                                  AV_PKT_DATA_DISPLAYMATRIX);
            if (psd)
                displaymatrix = (int32_t *)psd->data;
        }
        theta = get_rotation(displaymatrix);

        if (fabs(theta - 90) < 1.0) {
            INSERT_FILT("transpose", displaymatrix[3] > 0 ? "cclock_flip" : "clock");
        } else if (fabs(theta - 180) < 1.0) {
            if (displaymatrix[0] < 0)
                INSERT_FILT("hflip", NULL);
            if (displaymatrix[4] < 0)
                INSERT_FILT("vflip", NULL);
        } else if (fabs(theta - 270) < 1.0) {
            INSERT_FILT("transpose", displaymatrix[3] < 0 ? "clock_flip" : "cclock");
        } else if (fabs(theta) > 1.0) {
            char rotate_buf[64];
            snprintf(rotate_buf, sizeof(rotate_buf), "%f*PI/180", theta);
            INSERT_FILT("rotate", rotate_buf);
        } else {
            if (displaymatrix && displaymatrix[4] < 0)
                INSERT_FILT("vflip", NULL);
        }
    }

    if ((ret = configure_filtergraph(graph, vfilters, filt_src, last_filter)) < 0)
        goto fail;

    is->in_video_filter  = filt_src;
    is->out_video_filter = filt_out;

fail:
    av_freep(&par);
    return ret;
}

static int configure_audio_filters(VideoState *is, const char *afilters, int force_output_format)
{
    AVFilterContext *filt_asrc = NULL, *filt_asink = NULL;
    char aresample_swr_opts[512] = "";
    const AVDictionaryEntry *e = NULL;
    AVBPrint bp;
    char asrc_args[256];
    int ret;

    avfilter_graph_free(&is->agraph);
    if (!(is->agraph = avfilter_graph_alloc()))
        return AVERROR(ENOMEM);
    is->agraph->nb_threads = filter_nbthreads;

    av_bprint_init(&bp, 0, AV_BPRINT_SIZE_AUTOMATIC);

    while ((e = av_dict_iterate(swr_opts, e)))
        av_strlcatf(aresample_swr_opts, sizeof(aresample_swr_opts), "%s=%s:", e->key, e->value);
    if (strlen(aresample_swr_opts))
        aresample_swr_opts[strlen(aresample_swr_opts)-1] = '\0';
    av_opt_set(is->agraph, "aresample_swr_opts", aresample_swr_opts, 0);

    av_channel_layout_describe_bprint(&is->audio_filter_src.ch_layout, &bp);

    ret = snprintf(asrc_args, sizeof(asrc_args),
                   "sample_rate=%d:sample_fmt=%s:time_base=%d/%d:channel_layout=%s",
                   is->audio_filter_src.freq, av_get_sample_fmt_name(is->audio_filter_src.fmt),
                   1, is->audio_filter_src.freq, bp.str);

    ret = avfilter_graph_create_filter(&filt_asrc,
                                       avfilter_get_by_name("abuffer"), "ffplay_abuffer",
                                       asrc_args, NULL, is->agraph);
    if (ret < 0)
        goto end;

    filt_asink = avfilter_graph_alloc_filter(is->agraph, avfilter_get_by_name("abuffersink"),
                                             "ffplay_abuffersink");
    if (!filt_asink) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    if ((ret = av_opt_set(filt_asink, "sample_formats", "s16", AV_OPT_SEARCH_CHILDREN)) < 0)
        goto end;

    if (force_output_format) {
        if ((ret = av_opt_set_array(filt_asink, "channel_layouts", AV_OPT_SEARCH_CHILDREN,
                                    0, 1, AV_OPT_TYPE_CHLAYOUT, &is->audio_tgt.ch_layout)) < 0)
            goto end;
        if ((ret = av_opt_set_array(filt_asink, "samplerates", AV_OPT_SEARCH_CHILDREN,
                                    0, 1, AV_OPT_TYPE_INT, &is->audio_tgt.freq)) < 0)
            goto end;
    }

    ret = avfilter_init_dict(filt_asink, NULL);
    if (ret < 0)
        goto end;

    if ((ret = configure_filtergraph(is->agraph, afilters, filt_asrc, filt_asink)) < 0)
        goto end;

    is->in_audio_filter  = filt_asrc;
    is->out_audio_filter = filt_asink;

end:
    if (ret < 0)
        avfilter_graph_free(&is->agraph);
    av_bprint_finalize(&bp, NULL);

    return ret;
}

// 这个函数是音频解码线程的核心实现，负责从音频流中解码数据、应用滤镜并将处理后的音频帧送入播放队列。
static int audio_thread(void *arg)
{
    VideoState *is = arg;          // 获取播放器状态对象
    AVFrame *frame = av_frame_alloc(); // 分配帧对象用于存储解码后的音频
    Frame *af;                     // 指向音频帧队列的帧
    int last_serial = -1;          // 跟踪上一个包的序列号（用于检测seek操作）
    int reconfigure;               // 是否需要重新配置滤镜的标志
    int got_frame = 0;             // 是否成功解码帧的标志
    AVRational tb;                 // 时间基（用于时间戳计算）
    int ret = 0;                   // 函数返回值和错误码

    // 内存分配检查
    if (!frame)
        return AVERROR(ENOMEM);

    // 主解码循环
    do {
        // 尝试从解码器获取音频帧
        if ((got_frame = decoder_decode_frame(&is->auddec, frame, NULL)) < 0)
            goto the_end;  // 解码失败则跳转到清理

        if (got_frame) {  // 成功获取到音频帧
            tb = (AVRational){1, frame->sample_rate};  // 设置时间基（基于采样率）

            /* 检查是否需要重新配置音频滤镜 */
            reconfigure =
                // 1. 格式变化检测：比较源格式和当前帧格式
                cmp_audio_fmts(is->audio_filter_src.fmt, is->audio_filter_src.ch_layout.nb_channels,
                               frame->format, frame->ch_layout.nb_channels) ||
                // 2. 声道布局变化检测
                av_channel_layout_compare(&is->audio_filter_src.ch_layout, &frame->ch_layout) ||
                // 3. 采样率变化检测
                is->audio_filter_src.freq != frame->sample_rate ||
                // 4. 序列号变化检测（表示seek操作）
                is->auddec.pkt_serial != last_serial;

            // 如果需要重新配置滤镜
            if (reconfigure) {
                char buf1[1024], buf2[1024];
                // 获取声道布局描述（用于日志）
                av_channel_layout_describe(&is->audio_filter_src.ch_layout, buf1, sizeof(buf1));
                av_channel_layout_describe(&frame->ch_layout, buf2, sizeof(buf2));

                // 输出详细的格式变更日志
                av_log(NULL, AV_LOG_DEBUG,
                       "Audio frame changed from rate:%d ch:%d fmt:%s layout:%s serial:%d to rate:%d ch:%d fmt:%s layout:%s serial:%d\n",
                       is->audio_filter_src.freq, is->audio_filter_src.ch_layout.nb_channels,
                       av_get_sample_fmt_name(is->audio_filter_src.fmt), buf1, last_serial,
                       frame->sample_rate, frame->ch_layout.nb_channels,
                       av_get_sample_fmt_name(frame->format), buf2, is->auddec.pkt_serial);

                // 更新源格式信息
                is->audio_filter_src.fmt = frame->format;
                ret = av_channel_layout_copy(&is->audio_filter_src.ch_layout, &frame->ch_layout);
                if (ret < 0)
                    goto the_end;
                is->audio_filter_src.freq = frame->sample_rate;
                last_serial = is->auddec.pkt_serial;  // 更新序列号

                // 重新配置音频滤镜链
                if ((ret = configure_audio_filters(is, afilters, 1)) < 0)
                    goto the_end;
            }

            // 将解码帧添加到滤镜图输入
            if ((ret = av_buffersrc_add_frame(is->in_audio_filter, frame)) < 0)
                goto the_end;

            /* 从滤镜图输出获取处理后的帧 */
            while ((ret = av_buffersink_get_frame_flags(is->out_audio_filter, frame, 0)) >= 0) {
                // 获取帧的附加数据（如原始包位置）
                FrameData *fd = frame->opaque_ref ? (FrameData*)frame->opaque_ref->data : NULL;
                tb = av_buffersink_get_time_base(is->out_audio_filter);  // 获取滤镜输出的时间基

                // 从帧队列获取可写位置
                if (!(af = frame_queue_peek_writable(&is->sampq)))
                    goto the_end;

                // 设置帧信息
                af->pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb);
                af->pos = fd ? fd->pkt_pos : -1;      // 原始包位置（用于seek）
                af->serial = is->auddec.pkt_serial;   // 当前序列号
                af->duration = av_q2d((AVRational){frame->nb_samples, frame->sample_rate}); // 帧持续时间

                // 转移帧数据到队列（避免复制）
                av_frame_move_ref(af->frame, frame);

                // 将帧推入播放队列
                frame_queue_push(&is->sampq);

                // 检查序列号是否变化（seek操作）
                if (is->audioq.serial != is->auddec.pkt_serial)
                    break;  // 序列变化则停止处理当前帧
            }

            // 处理滤镜图结束（EOF）
            if (ret == AVERROR_EOF)
                is->auddec.finished = is->auddec.pkt_serial;
        }
    // 循环条件：正常、需重试或结束状态
    } while (ret >= 0 || ret == AVERROR(EAGAIN) || ret == AVERROR_EOF);

 the_end:  // 清理资源
    avfilter_graph_free(&is->agraph);  // 释放滤镜图
    av_frame_free(&frame);             // 释放帧内存
    return ret;                        // 返回状态
}

static int decoder_start(Decoder *d, int (*fn)(void *), const char *thread_name, void* arg)
{
    packet_queue_start(d->queue);
    d->decoder_tid = SDL_CreateThread(fn, thread_name, arg);
    if (!d->decoder_tid) {
        av_log(NULL, AV_LOG_ERROR, "SDL_CreateThread(): %s\n", SDL_GetError());
        return AVERROR(ENOMEM);
    }
    return 0;
}

// 获取帧→检查参数→配置滤镜→处理帧→入队显示
// 视频处理线程函数
static int video_thread(void *arg)
{
    VideoState *is = arg; // 获取播放器状态
    AVFrame *frame = av_frame_alloc(); // 分配帧内存
    double pts; // 显示时间戳
    double duration; // 帧持续时间
    int ret; // 返回值
    AVRational tb = is->video_st->time_base; // 视频流时间基
    AVRational frame_rate = av_guess_frame_rate(is->ic, is->video_st, NULL); // 获取帧率

    // 滤镜相关变量
    AVFilterGraph *graph = NULL;
    AVFilterContext *filt_out = NULL, *filt_in = NULL;
    int last_w = 0; // 缓存上次帧宽度
    int last_h = 0; // 缓存上次帧高度
    enum AVPixelFormat last_format = -2; // 缓存上次像素格式
    int last_serial = -1; // 缓存上次包序列号
    int last_vfilter_idx = 0; // 缓存上次滤镜索引

    if (!frame)
        return AVERROR(ENOMEM); // 内存分配失败处理

    for (;;) { // 主处理循环
        // 获取视频帧
        ret = get_video_frame(is, frame);
        if (ret < 0)
            goto the_end; // 错误处理
        if (!ret)
            continue; // 无数据则继续循环

        /* 检测视频帧属性变化（分辨率/格式/序列号/滤镜）*/
        if (   last_w != frame->width
            || last_h != frame->height
            || last_format != frame->format
            || last_serial != is->viddec.pkt_serial
            || last_vfilter_idx != is->vfilter_idx) {

            // 打印调试信息
            av_log(NULL, AV_LOG_DEBUG,
                   "Video frame changed from size:%dx%d format:%s serial:%d to size:%dx%d format:%s serial:%d\n",
                   last_w, last_h,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(last_format), "none"), last_serial,
                   frame->width, frame->height,
                   (const char *)av_x_if_null(av_get_pix_fmt_name(frame->format), "none"), is->viddec.pkt_serial);

            // 释放旧滤镜图并创建新图
            avfilter_graph_free(&graph);
            graph = avfilter_graph_alloc();
            if (!graph) {
                ret = AVERROR(ENOMEM);
                goto the_end;
            }
            graph->nb_threads = filter_nbthreads; // 设置滤镜线程数

            // 配置新视频滤镜
            if ((ret = configure_video_filters(graph, is, vfilters_list ? vfilters_list[is->vfilter_idx] : NULL, frame)) < 0) {
                // 配置失败时发送退出事件
                SDL_Event event;
                event.type = FF_QUIT_EVENT;
                event.user.data1 = is;
                SDL_PushEvent(&event);
                goto the_end;
            }

            // 更新滤镜上下文和缓存属性
            filt_in  = is->in_video_filter;
            filt_out = is->out_video_filter;
            last_w = frame->width;
            last_h = frame->height;
            last_format = frame->format;
            last_serial = is->viddec.pkt_serial;
            last_vfilter_idx = is->vfilter_idx;
            frame_rate = av_buffersink_get_frame_rate(filt_out); // 更新帧率
        }

        // 将帧送入滤镜输入源
        ret = av_buffersrc_add_frame(filt_in, frame);
        if (ret < 0)
            goto the_end;

        // 从滤镜输出接收处理后的帧
        while (ret >= 0) {
            FrameData *fd; // 帧元数据

            is->frame_last_returned_time = av_gettime_relative() / 1000000.0; // 记录处理开始时间

            // 获取滤镜处理后的帧
            ret = av_buffersink_get_frame_flags(filt_out, frame, 0);
            if (ret < 0) {
                if (ret == AVERROR_EOF) // 流结束处理
                    is->viddec.finished = is->viddec.pkt_serial;
                ret = 0;
                break;
            }

            fd = frame->opaque_ref ? (FrameData*)frame->opaque_ref->data : NULL; // 获取附加数据

            // 计算滤镜处理延迟
            is->frame_last_filter_delay = av_gettime_relative() / 1000000.0 - is->frame_last_returned_time;
            if (fabs(is->frame_last_filter_delay) > AV_NOSYNC_THRESHOLD / 10.0)
                is->frame_last_filter_delay = 0; // 超阈值则重置延迟

            tb = av_buffersink_get_time_base(filt_out); // 更新输出时间基
            duration = (frame_rate.num && frame_rate.den ?
                       av_q2d((AVRational){frame_rate.den, frame_rate.num}) : 0); // 计算帧持续时间 一帧应该在屏幕上显示多长时间
            pts = (frame->pts == AV_NOPTS_VALUE) ? NAN : frame->pts * av_q2d(tb); // 计算显示时间戳

            // 将处理后的帧加入显示队列
            ret = queue_picture(is, frame, pts, duration, fd ? fd->pkt_pos : -1, is->viddec.pkt_serial);
            av_frame_unref(frame); // 释放帧引用

            // 检查序列号是否变化（如发生seek）
            if (is->videoq.serial != is->viddec.pkt_serial)
                break;
        }

        if (ret < 0)
            goto the_end;
    }
 the_end: // 清理资源
    avfilter_graph_free(&graph);
    av_frame_free(&frame);
    return 0;
}

static int subtitle_thread(void *arg)
{
    VideoState *is = arg;
    Frame *sp;
    int got_subtitle;
    double pts;

    for (;;) {
        if (!(sp = frame_queue_peek_writable(&is->subpq)))
            return 0;

        if ((got_subtitle = decoder_decode_frame(&is->subdec, NULL, &sp->sub)) < 0)
            break;

        pts = 0;

        if (got_subtitle && sp->sub.format == 0) {
            if (sp->sub.pts != AV_NOPTS_VALUE)
                pts = sp->sub.pts / (double)AV_TIME_BASE;
            sp->pts = pts;
            sp->serial = is->subdec.pkt_serial;
            sp->width = is->subdec.avctx->width;
            sp->height = is->subdec.avctx->height;
            sp->uploaded = 0;

            /* now we can update the picture count */
            frame_queue_push(&is->subpq);
        } else if (got_subtitle) {
            avsubtitle_free(&sp->sub);
        }
    }
    return 0;
}

/* 
 * 更新样本显示 - 将音频样本数据复制到显示缓冲区
 * 目的：为音频波形可视化提供数据（通常在播放器界面显示）
 * 
 * 参数：
 *   is - 播放器全局状态对象
 *   samples - 音频样本数据数组（PCM数据）
 *   samples_size - 样本数据总字节数
 */
static void update_sample_display(VideoState *is, short *samples, int samples_size)
{
    int size, len;
    
    // 计算实际样本数量（每个样本占2字节）
    size = samples_size / sizeof(short);
    
    // 循环处理所有样本
    while (size > 0) {
        // 计算环形缓冲区剩余空间
        len = SAMPLE_ARRAY_SIZE - is->sample_array_index;
        
        // 确保不超出待复制数据范围
        if (len > size)
            len = size;
        
        // 复制样本到显示缓冲区
        memcpy(is->sample_array + is->sample_array_index, samples, len * sizeof(short));
        
        // 更新源数据指针和目标索引
        samples += len;
        is->sample_array_index += len;
        
        // 环形缓冲区处理：到达末尾时回到起始位置
        if (is->sample_array_index >= SAMPLE_ARRAY_SIZE)
            is->sample_array_index = 0;
        
        // 更新剩余待处理样本数
        size -= len;
    }
}

/* 
 * 音频同步处理 - 调整音频样本数以实现音视频同步
 * 核心功能：当主同步时钟不是音频时钟时，通过增减样本数实现同步
 * 
 * 参数：
 *   is - 播放器全局状态对象
 *   nb_samples - 当前音频帧的原始样本数
 * 
 * 返回值：调整后的目标样本数
 */
static int synchronize_audio(VideoState *is, int nb_samples)
{
    int wanted_nb_samples = nb_samples;  // 默认返回原始样本数

    // 仅当主同步时钟不是音频时钟时才需要调整
    if (get_master_sync_type(is) != AV_SYNC_AUDIO_MASTER) {
        double diff, avg_diff;
        int min_nb_samples, max_nb_samples;

        // 计算音频时钟与主时钟的差值
        diff = get_clock(&is->audclk) - get_master_clock(is);

        // 检查差值是否有效且在可接受范围内
        if (!isnan(diff) && fabs(diff) < AV_NOSYNC_THRESHOLD) {
            // 使用指数平滑累积时钟差值
            is->audio_diff_cum = diff + is->audio_diff_avg_coef * is->audio_diff_cum;
            
            // 确保有足够的测量值才进行估算
            if (is->audio_diff_avg_count < AUDIO_DIFF_AVG_NB) {
                is->audio_diff_avg_count++;
            } else {
                // 计算平均时钟偏差
                avg_diff = is->audio_diff_cum * (1.0 - is->audio_diff_avg_coef);
                
                // 如果平均偏差超过阈值，则调整样本数
                if (fabs(avg_diff) >= is->audio_diff_threshold) {
                    // 计算理论需要的样本数（基于采样率转换）
                    wanted_nb_samples = nb_samples + (int)(diff * is->audio_src.freq);
                    
                    // 计算允许的样本数调整范围（百分比限制）
                    min_nb_samples = ((nb_samples * (100 - SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    max_nb_samples = ((nb_samples * (100 + SAMPLE_CORRECTION_PERCENT_MAX) / 100));
                    
                    // 将调整后的样本数限制在合理范围内
                    wanted_nb_samples = av_clip(wanted_nb_samples, min_nb_samples, max_nb_samples);
                }
                
                // 跟踪日志：记录同步调试信息
                av_log(NULL, AV_LOG_TRACE, "diff=%f adiff=%f sample_diff=%d apts=%0.3f %f\n",
                      diff, avg_diff, wanted_nb_samples - nb_samples,
                      is->audio_clock, is->audio_diff_threshold);
            }
        } else {
            // 时钟偏差过大时重置同步状态
            is->audio_diff_avg_count = 0;
            is->audio_diff_cum       = 0;
        }
    }

    return wanted_nb_samples;
}

/**
 * 解码一帧音频数据并返回未压缩数据大小
 * 
 * 功能：
 * 1. 从采样队列获取音频帧
 * 2. 进行音视频同步计算
 * 3. 处理音频重采样（格式/采样率转换）
 * 4. 更新音频时钟
 * 
 * 返回：未压缩音频数据大小（字节数），出错返回-1
 */
static int audio_decode_frame(VideoState *is)
{
    int data_size, resampled_data_size;
    av_unused double audio_clock0;  // 调试用时钟值
    int wanted_nb_samples;          // 同步调整后的样本数
    Frame *af;                      // 音频帧指针

    // 检查播放状态（暂停直接返回）
    if (is->paused)
        return -1;

    // 从采样队列获取有效音频帧
    do {
        // Windows平台特殊处理：避免长时间阻塞
#if defined(_WIN32)
        while (frame_queue_nb_remaining(&is->sampq) == 0) {
            // 计算超时时间（半缓冲区时长）
            if ((av_gettime_relative() - audio_callback_time) > 
                1000000LL * is->audio_hw_buf_size / is->audio_tgt.bytes_per_sec / 2)
                return -1;
            av_usleep(1000);  // 短暂休眠避免CPU忙等
        }
#endif
        // 获取可读帧（非阻塞）
        if (!(af = frame_queue_peek_readable(&is->sampq)))
            return -1;
        frame_queue_next(&is->sampq);  // 移动队列指针
    } while (af->serial != is->audioq.serial);  // 确保序列号匹配（处理seek操作）

    // 计算原始音频帧数据大小
    data_size = av_samples_get_buffer_size(
        NULL, 
        af->frame->ch_layout.nb_channels,  // 声道数
        af->frame->nb_samples,             // 样本数
        af->frame->format,                 // 采样格式
        1                                  // 对齐
    );

    // 音视频同步：计算调整后的样本数
    wanted_nb_samples = synchronize_audio(is, af->frame->nb_samples);

    // 检查是否需要初始化/重建重采样器
    if (af->frame->format != is->audio_src.fmt ||             // 采样格式变化
        av_channel_layout_compare(&af->frame->ch_layout, &is->audio_src.ch_layout) || // 声道布局变化
        af->frame->sample_rate != is->audio_src.freq ||       // 采样率变化
        (wanted_nb_samples != af->frame->nb_samples && !is->swr_ctx) // 样本数调整且无重采样器
    ) {
        int ret;
        swr_free(&is->swr_ctx);  // 释放现有重采样器
        
        // 创建新的重采样器配置
        ret = swr_alloc_set_opts2(
            &is->swr_ctx,
            &is->audio_tgt.ch_layout,   // 目标声道布局
            is->audio_tgt.fmt,          // 目标采样格式
            is->audio_tgt.freq,         // 目标采样率
            &af->frame->ch_layout,      // 源声道布局
            af->frame->format,          // 源采样格式
            af->frame->sample_rate,     // 源采样率
            0,                          // 日志偏移
            NULL                        // 日志上下文
        );
        
        // 初始化重采样器
        if (ret < 0 || swr_init(is->swr_ctx) < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "无法创建采样率转换器：从 %d Hz %s %d 声道 到 %d Hz %s %d 声道\n",
                   af->frame->sample_rate, 
                   av_get_sample_fmt_name(af->frame->format),
                   af->frame->ch_layout.nb_channels,
                   is->audio_tgt.freq,
                   av_get_sample_fmt_name(is->audio_tgt.fmt),
                   is->audio_tgt.ch_layout.nb_channels);
            swr_free(&is->swr_ctx);
            return -1;
        }
        
        // 更新音频源参数
        if (av_channel_layout_copy(&is->audio_src.ch_layout, &af->frame->ch_layout) < 0)
            return -1;
        is->audio_src.freq = af->frame->sample_rate;
        is->audio_src.fmt = af->frame->format;
    }

    // 音频重采样处理
    if (is->swr_ctx) {
        const uint8_t **in = (const uint8_t **)af->frame->extended_data;
        uint8_t **out = &is->audio_buf1;
        
        // 计算重采样输出缓冲区大小（包含安全余量）
        int out_count = (int64_t)wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate + 256;
        int out_size = av_samples_get_buffer_size(
            NULL, 
            is->audio_tgt.ch_layout.nb_channels,
            out_count,
            is->audio_tgt.fmt,
            0
        );
        
        int len2;
        if (out_size < 0) {
            av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size() 失败\n");
            return -1;
        }
        
        // 设置样本数补偿（用于同步调整）
        if (wanted_nb_samples != af->frame->nb_samples) {
            if (swr_set_compensation(
                is->swr_ctx, 
                (wanted_nb_samples - af->frame->nb_samples) * is->audio_tgt.freq / af->frame->sample_rate,
                wanted_nb_samples * is->audio_tgt.freq / af->frame->sample_rate) < 0
            ) {
                av_log(NULL, AV_LOG_ERROR, "swr_set_compensation() 失败\n");
                return -1;
            }
        }
        
        // 动态分配重采样缓冲区
        av_fast_malloc(&is->audio_buf1, &is->audio_buf1_size, out_size);
        if (!is->audio_buf1)
            return AVERROR(ENOMEM);
        
        // 执行重采样
        len2 = swr_convert(
            is->swr_ctx, 
            out,       // 输出缓冲区
            out_count, // 输出容量
            in,        // 输入数据
            af->frame->nb_samples // 输入样本数
        );
        
        if (len2 < 0) {
            av_log(NULL, AV_LOG_ERROR, "swr_convert() 失败\n");
            return -1;
        }
        
        // 缓冲区不足警告处理
        if (len2 == out_count) {
            av_log(NULL, AV_LOG_WARNING, "音频缓冲区可能太小\n");
            if (swr_init(is->swr_ctx) < 0)
                swr_free(&is->swr_ctx);
        }
        
        // 设置最终音频缓冲区
        is->audio_buf = is->audio_buf1;
        // 计算重采样后数据大小
        resampled_data_size = len2 * is->audio_tgt.ch_layout.nb_channels * 
                              av_get_bytes_per_sample(is->audio_tgt.fmt);
    } else {
        // 无需重采样：直接使用原始数据
        is->audio_buf = af->frame->data[0];
        resampled_data_size = data_size;
    }

    // 更新音频时钟（用于同步）
    audio_clock0 = is->audio_clock;
    if (!isnan(af->pts)) {
        // 基于时间戳计算新时钟：PTS + 本帧持续时间
        is->audio_clock = af->pts + (double)af->frame->nb_samples / af->frame->sample_rate;
    } else {
        is->audio_clock = NAN;  // 无效时间戳
    }
    is->audio_clock_serial = af->serial;  // 更新序列号

// 调试信息
#ifdef DEBUG
    {
        static double last_clock;
        printf("audio: delay=%0.3f clock=%0.3f clock0=%0.3f\n",
               is->audio_clock - last_clock,
               is->audio_clock, audio_clock0);
        last_clock = is->audio_clock;
    }
#endif

    return resampled_data_size;
}

/* prepare a new audio buffer */
static void sdl_audio_callback(void *opaque, Uint8 *stream, int len)
{
    VideoState *is = opaque;
    int audio_size, len1;

    audio_callback_time = av_gettime_relative();

    while (len > 0) {
        if (is->audio_buf_index >= is->audio_buf_size) {
           audio_size = audio_decode_frame(is);
           if (audio_size < 0) {
                /* if error, just output silence */
               is->audio_buf = NULL;
               is->audio_buf_size = SDL_AUDIO_MIN_BUFFER_SIZE / is->audio_tgt.frame_size * is->audio_tgt.frame_size;
           } else {
               if (is->show_mode != SHOW_MODE_VIDEO)
                   update_sample_display(is, (int16_t *)is->audio_buf, audio_size);
               is->audio_buf_size = audio_size;
           }
           is->audio_buf_index = 0;
        }
        len1 = is->audio_buf_size - is->audio_buf_index;
        if (len1 > len)
            len1 = len;
        if (!is->muted && is->audio_buf && is->audio_volume == SDL_MIX_MAXVOLUME)
            memcpy(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, len1);
        else {
            memset(stream, 0, len1);
            if (!is->muted && is->audio_buf)
                SDL_MixAudioFormat(stream, (uint8_t *)is->audio_buf + is->audio_buf_index, AUDIO_S16SYS, len1, is->audio_volume);
        }
        len -= len1;
        stream += len1;
        is->audio_buf_index += len1;
    }
    is->audio_write_buf_size = is->audio_buf_size - is->audio_buf_index;
    /* Let's assume the audio driver that is used by SDL has two periods. */
    if (!isnan(is->audio_clock)) {
        set_clock_at(&is->audclk, is->audio_clock - (double)(2 * is->audio_hw_buf_size + is->audio_write_buf_size) / is->audio_tgt.bytes_per_sec, is->audio_clock_serial, audio_callback_time / 1000000.0);
        sync_clock_to_slave(&is->extclk, &is->audclk);
    }
}

static int audio_open(void *opaque, AVChannelLayout *wanted_channel_layout, int wanted_sample_rate, struct AudioParams *audio_hw_params)
{
    SDL_AudioSpec wanted_spec, spec;
    const char *env;
    static const int next_nb_channels[] = {0, 0, 1, 6, 2, 6, 4, 6};
    static const int next_sample_rates[] = {0, 44100, 48000, 96000, 192000};
    int next_sample_rate_idx = FF_ARRAY_ELEMS(next_sample_rates) - 1;
    int wanted_nb_channels = wanted_channel_layout->nb_channels;

    env = SDL_getenv("SDL_AUDIO_CHANNELS");
    if (env) {
        wanted_nb_channels = atoi(env);
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, wanted_nb_channels);
    }
    if (wanted_channel_layout->order != AV_CHANNEL_ORDER_NATIVE) {
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, wanted_nb_channels);
    }
    wanted_nb_channels = wanted_channel_layout->nb_channels;
    wanted_spec.channels = wanted_nb_channels;
    wanted_spec.freq = wanted_sample_rate;
    if (wanted_spec.freq <= 0 || wanted_spec.channels <= 0) {
        av_log(NULL, AV_LOG_ERROR, "Invalid sample rate or channel count!\n");
        return -1;
    }
    while (next_sample_rate_idx && next_sample_rates[next_sample_rate_idx] >= wanted_spec.freq)
        next_sample_rate_idx--;
    wanted_spec.format = AUDIO_S16SYS;
    wanted_spec.silence = 0;
    wanted_spec.samples = FFMAX(SDL_AUDIO_MIN_BUFFER_SIZE, 2 << av_log2(wanted_spec.freq / SDL_AUDIO_MAX_CALLBACKS_PER_SEC));
    wanted_spec.callback = sdl_audio_callback;
    wanted_spec.userdata = opaque;
    while (!(audio_dev = SDL_OpenAudioDevice(NULL, 0, &wanted_spec, &spec, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE | SDL_AUDIO_ALLOW_CHANNELS_CHANGE))) {
        av_log(NULL, AV_LOG_WARNING, "SDL_OpenAudio (%d channels, %d Hz): %s\n",
               wanted_spec.channels, wanted_spec.freq, SDL_GetError());
        wanted_spec.channels = next_nb_channels[FFMIN(7, wanted_spec.channels)];
        if (!wanted_spec.channels) {
            wanted_spec.freq = next_sample_rates[next_sample_rate_idx--];
            wanted_spec.channels = wanted_nb_channels;
            if (!wanted_spec.freq) {
                av_log(NULL, AV_LOG_ERROR,
                       "No more combinations to try, audio open failed\n");
                return -1;
            }
        }
        av_channel_layout_default(wanted_channel_layout, wanted_spec.channels);
    }
    if (spec.format != AUDIO_S16SYS) {
        av_log(NULL, AV_LOG_ERROR,
               "SDL advised audio format %d is not supported!\n", spec.format);
        return -1;
    }
    if (spec.channels != wanted_spec.channels) {
        av_channel_layout_uninit(wanted_channel_layout);
        av_channel_layout_default(wanted_channel_layout, spec.channels);
        if (wanted_channel_layout->order != AV_CHANNEL_ORDER_NATIVE) {
            av_log(NULL, AV_LOG_ERROR,
                   "SDL advised channel count %d is not supported!\n", spec.channels);
            return -1;
        }
    }

    audio_hw_params->fmt = AV_SAMPLE_FMT_S16;
    audio_hw_params->freq = spec.freq;
    if (av_channel_layout_copy(&audio_hw_params->ch_layout, wanted_channel_layout) < 0)
        return -1;
    audio_hw_params->frame_size = av_samples_get_buffer_size(NULL, audio_hw_params->ch_layout.nb_channels, 1, audio_hw_params->fmt, 1);
    audio_hw_params->bytes_per_sec = av_samples_get_buffer_size(NULL, audio_hw_params->ch_layout.nb_channels, audio_hw_params->freq, audio_hw_params->fmt, 1);
    if (audio_hw_params->bytes_per_sec <= 0 || audio_hw_params->frame_size <= 0) {
        av_log(NULL, AV_LOG_ERROR, "av_samples_get_buffer_size failed\n");
        return -1;
    }
    return spec.size;
}

static int create_hwaccel(AVBufferRef **device_ctx)
{
    enum AVHWDeviceType type;
    int ret;
    AVBufferRef *vk_dev;

    *device_ctx = NULL;

    if (!hwaccel)
        return 0;

    type = av_hwdevice_find_type_by_name(hwaccel);
    if (type == AV_HWDEVICE_TYPE_NONE)
        return AVERROR(ENOTSUP);

    if (!vk_renderer) {
        av_log(NULL, AV_LOG_ERROR, "Vulkan renderer is not available\n");
        return AVERROR(ENOTSUP);
    }

    ret = vk_renderer_get_hw_dev(vk_renderer, &vk_dev);
    if (ret < 0)
        return ret;

    ret = av_hwdevice_ctx_create_derived(device_ctx, type, vk_dev, 0);
    if (!ret)
        return 0;

    if (ret != AVERROR(ENOSYS))
        return ret;

    av_log(NULL, AV_LOG_WARNING, "Derive %s from vulkan not supported.\n", hwaccel);
    ret = av_hwdevice_ctx_create(device_ctx, type, NULL, NULL, 0);
    return ret;
}

/* open a given stream. Return 0 if OK */
static int stream_component_open(VideoState *is, int stream_index)
{
    AVFormatContext *ic = is->ic;
    AVCodecContext *avctx;
    const AVCodec *codec;
    const char *forced_codec_name = NULL;
    AVDictionary *opts = NULL;
    int sample_rate;
    AVChannelLayout ch_layout = { 0 };
    int ret = 0;
    int stream_lowres = lowres;

    if (stream_index < 0 || stream_index >= ic->nb_streams)
        return -1;

    avctx = avcodec_alloc_context3(NULL);
    if (!avctx)
        return AVERROR(ENOMEM);

    ret = avcodec_parameters_to_context(avctx, ic->streams[stream_index]->codecpar);
    if (ret < 0)
        goto fail;
    avctx->pkt_timebase = ic->streams[stream_index]->time_base;

    codec = avcodec_find_decoder(avctx->codec_id);

    switch(avctx->codec_type){
        case AVMEDIA_TYPE_AUDIO   : is->last_audio_stream    = stream_index; forced_codec_name =    audio_codec_name; break;
        case AVMEDIA_TYPE_SUBTITLE: is->last_subtitle_stream = stream_index; forced_codec_name = subtitle_codec_name; break;
        case AVMEDIA_TYPE_VIDEO   : is->last_video_stream    = stream_index; forced_codec_name =    video_codec_name; break;
    }
    if (forced_codec_name)
        codec = avcodec_find_decoder_by_name(forced_codec_name);
    if (!codec) {
        if (forced_codec_name) av_log(NULL, AV_LOG_WARNING,
                                      "No codec could be found with name '%s'\n", forced_codec_name);
        else                   av_log(NULL, AV_LOG_WARNING,
                                      "No decoder could be found for codec %s\n", avcodec_get_name(avctx->codec_id));
        ret = AVERROR(EINVAL);
        goto fail;
    }

    avctx->codec_id = codec->id;
    if (stream_lowres > codec->max_lowres) {
        av_log(avctx, AV_LOG_WARNING, "The maximum value for lowres supported by the decoder is %d\n",
                codec->max_lowres);
        stream_lowres = codec->max_lowres;
    }
    avctx->lowres = stream_lowres;

    if (fast)
        avctx->flags2 |= AV_CODEC_FLAG2_FAST;

    ret = filter_codec_opts(codec_opts, avctx->codec_id, ic,
                            ic->streams[stream_index], codec, &opts, NULL);
    if (ret < 0)
        goto fail;

    if (!av_dict_get(opts, "threads", NULL, 0))
        av_dict_set(&opts, "threads", "auto", 0);
    if (stream_lowres)
        av_dict_set_int(&opts, "lowres", stream_lowres, 0);

    av_dict_set(&opts, "flags", "+copy_opaque", AV_DICT_MULTIKEY);

    if (avctx->codec_type == AVMEDIA_TYPE_VIDEO) {
        ret = create_hwaccel(&avctx->hw_device_ctx);
        if (ret < 0)
            goto fail;
    }

    if ((ret = avcodec_open2(avctx, codec, &opts)) < 0) {
        goto fail;
    }
    ret = check_avoptions(opts);
    if (ret < 0)
        goto fail;

    is->eof = 0;
    ic->streams[stream_index]->discard = AVDISCARD_DEFAULT;
    switch (avctx->codec_type) {
    case AVMEDIA_TYPE_AUDIO:
        {
            AVFilterContext *sink;

            is->audio_filter_src.freq           = avctx->sample_rate;
            ret = av_channel_layout_copy(&is->audio_filter_src.ch_layout, &avctx->ch_layout);
            if (ret < 0)
                goto fail;
            is->audio_filter_src.fmt            = avctx->sample_fmt;
            if ((ret = configure_audio_filters(is, afilters, 0)) < 0)
                goto fail;
            sink = is->out_audio_filter;
            sample_rate    = av_buffersink_get_sample_rate(sink);
            ret = av_buffersink_get_ch_layout(sink, &ch_layout);
            if (ret < 0)
                goto fail;
        }

        /* prepare audio output */
        if ((ret = audio_open(is, &ch_layout, sample_rate, &is->audio_tgt)) < 0)
            goto fail;
        is->audio_hw_buf_size = ret;
        is->audio_src = is->audio_tgt;
        is->audio_buf_size  = 0;
        is->audio_buf_index = 0;

        /* init averaging filter */
        is->audio_diff_avg_coef  = exp(log(0.01) / AUDIO_DIFF_AVG_NB);
        is->audio_diff_avg_count = 0;
        /* since we do not have a precise anough audio FIFO fullness,
           we correct audio sync only if larger than this threshold */
        is->audio_diff_threshold = (double)(is->audio_hw_buf_size) / is->audio_tgt.bytes_per_sec;

        is->audio_stream = stream_index;
        is->audio_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->auddec, avctx, &is->audioq, is->continue_read_thread)) < 0)
            goto fail;
        if (is->ic->iformat->flags & AVFMT_NOTIMESTAMPS) {
            is->auddec.start_pts = is->audio_st->start_time;
            is->auddec.start_pts_tb = is->audio_st->time_base;
        }
        if ((ret = decoder_start(&is->auddec, audio_thread, "audio_decoder", is)) < 0)
            goto out;
        SDL_PauseAudioDevice(audio_dev, 0);
        break;
    case AVMEDIA_TYPE_VIDEO:
        is->video_stream = stream_index;
        is->video_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->viddec, avctx, &is->videoq, is->continue_read_thread)) < 0)
            goto fail;
        if ((ret = decoder_start(&is->viddec, video_thread, "video_decoder", is)) < 0)
            goto out;
        is->queue_attachments_req = 1;
        break;
    case AVMEDIA_TYPE_SUBTITLE:
        is->subtitle_stream = stream_index;
        is->subtitle_st = ic->streams[stream_index];

        if ((ret = decoder_init(&is->subdec, avctx, &is->subtitleq, is->continue_read_thread)) < 0)
            goto fail;
        if ((ret = decoder_start(&is->subdec, subtitle_thread, "subtitle_decoder", is)) < 0)
            goto out;
        break;
    default:
        break;
    }
    goto out;

fail:
    avcodec_free_context(&avctx);
out:
    av_channel_layout_uninit(&ch_layout);
    av_dict_free(&opts);

    return ret;
}

static int decode_interrupt_cb(void *ctx)
{
    VideoState *is = ctx;
    return is->abort_request;
}

static int stream_has_enough_packets(AVStream *st, int stream_id, PacketQueue *queue) {
    return stream_id < 0 ||
           queue->abort_request ||
           (st->disposition & AV_DISPOSITION_ATTACHED_PIC) ||
           queue->nb_packets > MIN_FRAMES && (!queue->duration || av_q2d(st->time_base) * queue->duration > 1.0);
}

static int is_realtime(AVFormatContext *s)
{
    if(   !strcmp(s->iformat->name, "rtp")
       || !strcmp(s->iformat->name, "rtsp")
       || !strcmp(s->iformat->name, "sdp")
    )
        return 1;

    if(s->pb && (   !strncmp(s->url, "rtp:", 4)
                 || !strncmp(s->url, "udp:", 4)
                )
    )
        return 1;
    return 0;
}

/* this thread gets the stream from the disk or the network */
static int read_thread(void *arg)
{
    VideoState *is = arg;
    AVFormatContext *ic = NULL;
    int err, i, ret;
    int st_index[AVMEDIA_TYPE_NB];
    AVPacket *pkt = NULL;
    int64_t stream_start_time;
    int pkt_in_play_range = 0;
    const AVDictionaryEntry *t;
    SDL_mutex *wait_mutex = SDL_CreateMutex();
    int scan_all_pmts_set = 0;
    int64_t pkt_ts;

    if (!wait_mutex) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateMutex(): %s\n", SDL_GetError());
        ret = AVERROR(ENOMEM);
        goto fail;
    }

    memset(st_index, -1, sizeof(st_index));
    is->eof = 0;

    pkt = av_packet_alloc();
    if (!pkt) {
        av_log(NULL, AV_LOG_FATAL, "Could not allocate packet.\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ic = avformat_alloc_context();
    if (!ic) {
        av_log(NULL, AV_LOG_FATAL, "Could not allocate context.\n");
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    ic->interrupt_callback.callback = decode_interrupt_cb;
    ic->interrupt_callback.opaque = is;
    if (!av_dict_get(format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE)) {
        av_dict_set(&format_opts, "scan_all_pmts", "1", AV_DICT_DONT_OVERWRITE);
        scan_all_pmts_set = 1;
    }
    err = avformat_open_input(&ic, is->filename, is->iformat, &format_opts);
    if (err < 0) {
        print_error(is->filename, err);
        ret = -1;
        goto fail;
    }
    if (scan_all_pmts_set)
        av_dict_set(&format_opts, "scan_all_pmts", NULL, AV_DICT_MATCH_CASE);
    remove_avoptions(&format_opts, codec_opts);

    ret = check_avoptions(format_opts);
    if (ret < 0)
        goto fail;
    is->ic = ic;

    if (genpts)
        ic->flags |= AVFMT_FLAG_GENPTS;

    if (find_stream_info) {
        AVDictionary **opts;
        int orig_nb_streams = ic->nb_streams;

        err = setup_find_stream_info_opts(ic, codec_opts, &opts);
        if (err < 0) {
            av_log(NULL, AV_LOG_ERROR,
                   "Error setting up avformat_find_stream_info() options\n");
            ret = err;
            goto fail;
        }

        err = avformat_find_stream_info(ic, opts);

        for (i = 0; i < orig_nb_streams; i++)
            av_dict_free(&opts[i]);
        av_freep(&opts);

        if (err < 0) {
            av_log(NULL, AV_LOG_WARNING,
                   "%s: could not find codec parameters\n", is->filename);
            ret = -1;
            goto fail;
        }
    }

    if (ic->pb)
        ic->pb->eof_reached = 0; // FIXME hack, ffplay maybe should not use avio_feof() to test for the end

    if (seek_by_bytes < 0)
        seek_by_bytes = !(ic->iformat->flags & AVFMT_NO_BYTE_SEEK) &&
                        !!(ic->iformat->flags & AVFMT_TS_DISCONT) &&
                        strcmp("ogg", ic->iformat->name);

    is->max_frame_duration = (ic->iformat->flags & AVFMT_TS_DISCONT) ? 10.0 : 3600.0;

    if (!window_title && (t = av_dict_get(ic->metadata, "title", NULL, 0)))
        window_title = av_asprintf("%s - %s", t->value, input_filename);

    /* if seeking requested, we execute it */
    if (start_time != AV_NOPTS_VALUE) {
        int64_t timestamp;

        timestamp = start_time;
        /* add the stream start time */
        if (ic->start_time != AV_NOPTS_VALUE)
            timestamp += ic->start_time;
        ret = avformat_seek_file(ic, -1, INT64_MIN, timestamp, INT64_MAX, 0);
        if (ret < 0) {
            av_log(NULL, AV_LOG_WARNING, "%s: could not seek to position %0.3f\n",
                    is->filename, (double)timestamp / AV_TIME_BASE);
        }
    }

    is->realtime = is_realtime(ic);

    if (show_status)
        av_dump_format(ic, 0, is->filename, 0);

    for (i = 0; i < ic->nb_streams; i++) {
        AVStream *st = ic->streams[i];
        enum AVMediaType type = st->codecpar->codec_type;
        st->discard = AVDISCARD_ALL;
        if (type >= 0 && wanted_stream_spec[type] && st_index[type] == -1)
            if (avformat_match_stream_specifier(ic, st, wanted_stream_spec[type]) > 0)
                st_index[type] = i;
    }
    for (i = 0; i < AVMEDIA_TYPE_NB; i++) {
        if (wanted_stream_spec[i] && st_index[i] == -1) {
            av_log(NULL, AV_LOG_ERROR, "Stream specifier %s does not match any %s stream\n", wanted_stream_spec[i], av_get_media_type_string(i));
            st_index[i] = INT_MAX;
        }
    }

    if (!video_disable)
        st_index[AVMEDIA_TYPE_VIDEO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO,
                                st_index[AVMEDIA_TYPE_VIDEO], -1, NULL, 0);
    if (!audio_disable)
        st_index[AVMEDIA_TYPE_AUDIO] =
            av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO,
                                st_index[AVMEDIA_TYPE_AUDIO],
                                st_index[AVMEDIA_TYPE_VIDEO],
                                NULL, 0);
    if (!video_disable && !subtitle_disable)
        st_index[AVMEDIA_TYPE_SUBTITLE] =
            av_find_best_stream(ic, AVMEDIA_TYPE_SUBTITLE,
                                st_index[AVMEDIA_TYPE_SUBTITLE],
                                (st_index[AVMEDIA_TYPE_AUDIO] >= 0 ?
                                 st_index[AVMEDIA_TYPE_AUDIO] :
                                 st_index[AVMEDIA_TYPE_VIDEO]),
                                NULL, 0);

    is->show_mode = show_mode;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        AVStream *st = ic->streams[st_index[AVMEDIA_TYPE_VIDEO]];
        AVCodecParameters *codecpar = st->codecpar;
        AVRational sar = av_guess_sample_aspect_ratio(ic, st, NULL);
        if (codecpar->width)
            set_default_window_size(codecpar->width, codecpar->height, sar);
    }

    /* open the streams */
    if (st_index[AVMEDIA_TYPE_AUDIO] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_AUDIO]);
    }

    ret = -1;
    if (st_index[AVMEDIA_TYPE_VIDEO] >= 0) {
        ret = stream_component_open(is, st_index[AVMEDIA_TYPE_VIDEO]);
    }
    if (is->show_mode == SHOW_MODE_NONE)
        is->show_mode = ret >= 0 ? SHOW_MODE_VIDEO : SHOW_MODE_RDFT;

    if (st_index[AVMEDIA_TYPE_SUBTITLE] >= 0) {
        stream_component_open(is, st_index[AVMEDIA_TYPE_SUBTITLE]);
    }

    if (is->video_stream < 0 && is->audio_stream < 0) {
        av_log(NULL, AV_LOG_FATAL, "Failed to open file '%s' or configure filtergraph\n",
               is->filename);
        ret = -1;
        goto fail;
    }

    if (infinite_buffer < 0 && is->realtime)
        infinite_buffer = 1;

    for (;;) {
        if (is->abort_request)
            break;
        if (is->paused != is->last_paused) {
            is->last_paused = is->paused;
            if (is->paused)
                is->read_pause_return = av_read_pause(ic);
            else
                av_read_play(ic);
        }
#if CONFIG_RTSP_DEMUXER || CONFIG_MMSH_PROTOCOL
        if (is->paused &&
                (!strcmp(ic->iformat->name, "rtsp") ||
                 (ic->pb && !strncmp(input_filename, "mmsh:", 5)))) {
            /* wait 10 ms to avoid trying to get another packet */
            /* XXX: horrible */
            SDL_Delay(10);
            continue;
        }
#endif
        if (is->seek_req) {
            int64_t seek_target = is->seek_pos;
            int64_t seek_min    = is->seek_rel > 0 ? seek_target - is->seek_rel + 2: INT64_MIN;
            int64_t seek_max    = is->seek_rel < 0 ? seek_target - is->seek_rel - 2: INT64_MAX;
// FIXME the +-2 is due to rounding being not done in the correct direction in generation
//      of the seek_pos/seek_rel variables

            ret = avformat_seek_file(is->ic, -1, seek_min, seek_target, seek_max, is->seek_flags);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR,
                       "%s: error while seeking\n", is->ic->url);
            } else {
                if (is->audio_stream >= 0)
                    packet_queue_flush(&is->audioq);
                if (is->subtitle_stream >= 0)
                    packet_queue_flush(&is->subtitleq);
                if (is->video_stream >= 0)
                    packet_queue_flush(&is->videoq);
                if (is->seek_flags & AVSEEK_FLAG_BYTE) {
                   set_clock(&is->extclk, NAN, 0);
                } else {
                   set_clock(&is->extclk, seek_target / (double)AV_TIME_BASE, 0);
                }
            }
            is->seek_req = 0;
            is->queue_attachments_req = 1;
            is->eof = 0;
            if (is->paused)
                step_to_next_frame(is);
        }
        if (is->queue_attachments_req) {
            if (is->video_st && is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC) {
                if ((ret = av_packet_ref(pkt, &is->video_st->attached_pic)) < 0)
                    goto fail;
                packet_queue_put(&is->videoq, pkt);
                packet_queue_put_nullpacket(&is->videoq, pkt, is->video_stream);
            }
            is->queue_attachments_req = 0;
        }

        /* if the queue are full, no need to read more */
        if (infinite_buffer<1 &&
              (is->audioq.size + is->videoq.size + is->subtitleq.size > MAX_QUEUE_SIZE
            || (stream_has_enough_packets(is->audio_st, is->audio_stream, &is->audioq) &&
                stream_has_enough_packets(is->video_st, is->video_stream, &is->videoq) &&
                stream_has_enough_packets(is->subtitle_st, is->subtitle_stream, &is->subtitleq)))) {
            /* wait 10 ms */
            SDL_LockMutex(wait_mutex);
            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            SDL_UnlockMutex(wait_mutex);
            continue;
        }
        if (!is->paused &&
            (!is->audio_st || (is->auddec.finished == is->audioq.serial && frame_queue_nb_remaining(&is->sampq) == 0)) &&
            (!is->video_st || (is->viddec.finished == is->videoq.serial && frame_queue_nb_remaining(&is->pictq) == 0))) {
            if (loop != 1 && (!loop || --loop)) {
                stream_seek(is, start_time != AV_NOPTS_VALUE ? start_time : 0, 0, 0);
            } else if (autoexit) {
                ret = AVERROR_EOF;
                goto fail;
            }
        }
        ret = av_read_frame(ic, pkt);
        if (ret < 0) {
            if ((ret == AVERROR_EOF || avio_feof(ic->pb)) && !is->eof) {
                if (is->video_stream >= 0)
                    packet_queue_put_nullpacket(&is->videoq, pkt, is->video_stream);
                if (is->audio_stream >= 0)
                    packet_queue_put_nullpacket(&is->audioq, pkt, is->audio_stream);
                if (is->subtitle_stream >= 0)
                    packet_queue_put_nullpacket(&is->subtitleq, pkt, is->subtitle_stream);
                is->eof = 1;
            }
            if (ic->pb && ic->pb->error) {
                if (autoexit)
                    goto fail;
                else
                    break;
            }
            SDL_LockMutex(wait_mutex);
            SDL_CondWaitTimeout(is->continue_read_thread, wait_mutex, 10);
            SDL_UnlockMutex(wait_mutex);
            continue;
        } else {
            is->eof = 0;
        }
        /* check if packet is in play range specified by user, then queue, otherwise discard */
        stream_start_time = ic->streams[pkt->stream_index]->start_time;
        pkt_ts = pkt->pts == AV_NOPTS_VALUE ? pkt->dts : pkt->pts;
        pkt_in_play_range = duration == AV_NOPTS_VALUE ||
                (pkt_ts - (stream_start_time != AV_NOPTS_VALUE ? stream_start_time : 0)) *
                av_q2d(ic->streams[pkt->stream_index]->time_base) -
                (double)(start_time != AV_NOPTS_VALUE ? start_time : 0) / 1000000
                <= ((double)duration / 1000000);
        if (pkt->stream_index == is->audio_stream && pkt_in_play_range) {
            packet_queue_put(&is->audioq, pkt);
        } else if (pkt->stream_index == is->video_stream && pkt_in_play_range
                   && !(is->video_st->disposition & AV_DISPOSITION_ATTACHED_PIC)) {
            packet_queue_put(&is->videoq, pkt);
        } else if (pkt->stream_index == is->subtitle_stream && pkt_in_play_range) {
            packet_queue_put(&is->subtitleq, pkt);
        } else {
            av_packet_unref(pkt);
        }
    }

    ret = 0;
 fail:
    if (ic && !is->ic)
        avformat_close_input(&ic);

    av_packet_free(&pkt);
    if (ret != 0) {
        SDL_Event event;

        event.type = FF_QUIT_EVENT;
        event.user.data1 = is;
        SDL_PushEvent(&event);
    }
    SDL_DestroyMutex(wait_mutex);
    return 0;
}

static VideoState *stream_open(const char *filename,
                               const AVInputFormat *iformat)
{
    VideoState *is;

    is = av_mallocz(sizeof(VideoState));
    if (!is)
        return NULL;
    is->last_video_stream = is->video_stream = -1;
    is->last_audio_stream = is->audio_stream = -1;
    is->last_subtitle_stream = is->subtitle_stream = -1;
    is->filename = av_strdup(filename);
    if (!is->filename)
        goto fail;
    is->iformat = iformat;
    is->ytop    = 0;
    is->xleft   = 0;

    /* start video display */
    if (frame_queue_init(&is->pictq, &is->videoq, VIDEO_PICTURE_QUEUE_SIZE, 1) < 0)
        goto fail;
    if (frame_queue_init(&is->subpq, &is->subtitleq, SUBPICTURE_QUEUE_SIZE, 0) < 0)
        goto fail;
    if (frame_queue_init(&is->sampq, &is->audioq, SAMPLE_QUEUE_SIZE, 1) < 0)
        goto fail;

    if (packet_queue_init(&is->videoq) < 0 ||
        packet_queue_init(&is->audioq) < 0 ||
        packet_queue_init(&is->subtitleq) < 0)
        goto fail;

    if (!(is->continue_read_thread = SDL_CreateCond())) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateCond(): %s\n", SDL_GetError());
        goto fail;
    }

    init_clock(&is->vidclk, &is->videoq.serial);
    init_clock(&is->audclk, &is->audioq.serial);
    init_clock(&is->extclk, &is->extclk.serial);
    is->audio_clock_serial = -1;
    if (startup_volume < 0)
        av_log(NULL, AV_LOG_WARNING, "-volume=%d < 0, setting to 0\n", startup_volume);
    if (startup_volume > 100)
        av_log(NULL, AV_LOG_WARNING, "-volume=%d > 100, setting to 100\n", startup_volume);
    startup_volume = av_clip(startup_volume, 0, 100);
    startup_volume = av_clip(SDL_MIX_MAXVOLUME * startup_volume / 100, 0, SDL_MIX_MAXVOLUME);
    is->audio_volume = startup_volume;
    is->muted = 0;
    is->av_sync_type = av_sync_type;
    is->read_tid     = SDL_CreateThread(read_thread, "read_thread", is);
    if (!is->read_tid) {
        av_log(NULL, AV_LOG_FATAL, "SDL_CreateThread(): %s\n", SDL_GetError());
fail:
        stream_close(is);
        return NULL;
    }
    return is;
}

static void stream_cycle_channel(VideoState *is, int codec_type)
{
    AVFormatContext *ic = is->ic;
    int start_index, stream_index;
    int old_index;
    AVStream *st;
    AVProgram *p = NULL;
    int nb_streams = is->ic->nb_streams;

    if (codec_type == AVMEDIA_TYPE_VIDEO) {
        start_index = is->last_video_stream;
        old_index = is->video_stream;
    } else if (codec_type == AVMEDIA_TYPE_AUDIO) {
        start_index = is->last_audio_stream;
        old_index = is->audio_stream;
    } else {
        start_index = is->last_subtitle_stream;
        old_index = is->subtitle_stream;
    }
    stream_index = start_index;

    if (codec_type != AVMEDIA_TYPE_VIDEO && is->video_stream != -1) {
        p = av_find_program_from_stream(ic, NULL, is->video_stream);
        if (p) {
            nb_streams = p->nb_stream_indexes;
            for (start_index = 0; start_index < nb_streams; start_index++)
                if (p->stream_index[start_index] == stream_index)
                    break;
            if (start_index == nb_streams)
                start_index = -1;
            stream_index = start_index;
        }
    }

    for (;;) {
        if (++stream_index >= nb_streams)
        {
            if (codec_type == AVMEDIA_TYPE_SUBTITLE)
            {
                stream_index = -1;
                is->last_subtitle_stream = -1;
                goto the_end;
            }
            if (start_index == -1)
                return;
            stream_index = 0;
        }
        if (stream_index == start_index)
            return;
        st = is->ic->streams[p ? p->stream_index[stream_index] : stream_index];
        if (st->codecpar->codec_type == codec_type) {
            /* check that parameters are OK */
            switch (codec_type) {
            case AVMEDIA_TYPE_AUDIO:
                if (st->codecpar->sample_rate != 0 &&
                    st->codecpar->ch_layout.nb_channels != 0)
                    goto the_end;
                break;
            case AVMEDIA_TYPE_VIDEO:
            case AVMEDIA_TYPE_SUBTITLE:
                goto the_end;
            default:
                break;
            }
        }
    }
 the_end:
    if (p && stream_index != -1)
        stream_index = p->stream_index[stream_index];
    av_log(NULL, AV_LOG_INFO, "Switch %s stream from #%d to #%d\n",
           av_get_media_type_string(codec_type),
           old_index,
           stream_index);

    stream_component_close(is, old_index);
    stream_component_open(is, stream_index);
}


static void toggle_full_screen(VideoState *is)
{
    is_full_screen = !is_full_screen;
    SDL_SetWindowFullscreen(window, is_full_screen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
}

static void toggle_audio_display(VideoState *is)
{
    int next = is->show_mode;
    do {
        next = (next + 1) % SHOW_MODE_NB;
    } while (next != is->show_mode && (next == SHOW_MODE_VIDEO && !is->video_st || next != SHOW_MODE_VIDEO && !is->audio_st));
    if (is->show_mode != next) {
        is->force_refresh = 1;
        is->show_mode = next;
    }
}

static void refresh_loop_wait_event(VideoState *is, SDL_Event *event) {
    double remaining_time = 0.0;
    SDL_PumpEvents();
    while (!SDL_PeepEvents(event, 1, SDL_GETEVENT, SDL_FIRSTEVENT, SDL_LASTEVENT)) {
        if (!cursor_hidden && av_gettime_relative() - cursor_last_shown > CURSOR_HIDE_DELAY) {
            SDL_ShowCursor(0);
            cursor_hidden = 1;
        }
        if (remaining_time > 0.0)
            av_usleep((int64_t)(remaining_time * 1000000.0));
        remaining_time = REFRESH_RATE;
        if (is->show_mode != SHOW_MODE_NONE && (!is->paused || is->force_refresh))
            video_refresh(is, &remaining_time);
        SDL_PumpEvents();
    }
}

static void seek_chapter(VideoState *is, int incr)
{
    int64_t pos = get_master_clock(is) * AV_TIME_BASE;
    int i;

    if (!is->ic->nb_chapters)
        return;

    /* find the current chapter */
    for (i = 0; i < is->ic->nb_chapters; i++) {
        AVChapter *ch = is->ic->chapters[i];
        if (av_compare_ts(pos, AV_TIME_BASE_Q, ch->start, ch->time_base) < 0) {
            i--;
            break;
        }
    }

    i += incr;
    i = FFMAX(i, 0);
    if (i >= is->ic->nb_chapters)
        return;

    av_log(NULL, AV_LOG_VERBOSE, "Seeking to chapter %d.\n", i);
    stream_seek(is, av_rescale_q(is->ic->chapters[i]->start, is->ic->chapters[i]->time_base,
                                 AV_TIME_BASE_Q), 0, 0);
}

/* handle an event sent by the GUI */
static void event_loop(VideoState *cur_stream)
{
    SDL_Event event;
    double incr, pos, frac;

    for (;;) {
        double x;
        refresh_loop_wait_event(cur_stream, &event);
        switch (event.type) {
        case SDL_KEYDOWN:
            if (exit_on_keydown || event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q) {
                do_exit(cur_stream);
                break;
            }
            // If we don't yet have a window, skip all key events, because read_thread might still be initializing...
            if (!cur_stream->width)
                continue;
            switch (event.key.keysym.sym) {
            case SDLK_f:
                toggle_full_screen(cur_stream);
                cur_stream->force_refresh = 1;
                break;
            case SDLK_p:
            case SDLK_SPACE:
                toggle_pause(cur_stream);
                break;
            case SDLK_m:
                toggle_mute(cur_stream);
                break;
            case SDLK_KP_MULTIPLY:
            case SDLK_0:
                update_volume(cur_stream, 1, SDL_VOLUME_STEP);
                break;
            case SDLK_KP_DIVIDE:
            case SDLK_9:
                update_volume(cur_stream, -1, SDL_VOLUME_STEP);
                break;
            case SDLK_s: // S: Step to next frame
                step_to_next_frame(cur_stream);
                break;
            case SDLK_a:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                break;
            case SDLK_v:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                break;
            case SDLK_c:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_VIDEO);
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_AUDIO);
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                break;
            case SDLK_t:
                stream_cycle_channel(cur_stream, AVMEDIA_TYPE_SUBTITLE);
                break;
            case SDLK_w:
                if (cur_stream->show_mode == SHOW_MODE_VIDEO && cur_stream->vfilter_idx < nb_vfilters - 1) {
                    if (++cur_stream->vfilter_idx >= nb_vfilters)
                        cur_stream->vfilter_idx = 0;
                } else {
                    cur_stream->vfilter_idx = 0;
                    toggle_audio_display(cur_stream);
                }
                break;
            case SDLK_PAGEUP:
                if (cur_stream->ic->nb_chapters <= 1) {
                    incr = 600.0;
                    goto do_seek;
                }
                seek_chapter(cur_stream, 1);
                break;
            case SDLK_PAGEDOWN:
                if (cur_stream->ic->nb_chapters <= 1) {
                    incr = -600.0;
                    goto do_seek;
                }
                seek_chapter(cur_stream, -1);
                break;
            case SDLK_LEFT:
                incr = seek_interval ? -seek_interval : -10.0;
                goto do_seek;
            case SDLK_RIGHT:
                incr = seek_interval ? seek_interval : 10.0;
                goto do_seek;
            case SDLK_UP:
                incr = 60.0;
                goto do_seek;
            case SDLK_DOWN:
                incr = -60.0;
            do_seek:
                    if (seek_by_bytes) {
                        pos = -1;
                        if (pos < 0 && cur_stream->video_stream >= 0)
                            pos = frame_queue_last_pos(&cur_stream->pictq);
                        if (pos < 0 && cur_stream->audio_stream >= 0)
                            pos = frame_queue_last_pos(&cur_stream->sampq);
                        if (pos < 0)
                            pos = avio_tell(cur_stream->ic->pb);
                        if (cur_stream->ic->bit_rate)
                            incr *= cur_stream->ic->bit_rate / 8.0;
                        else
                            incr *= 180000.0;
                        pos += incr;
                        stream_seek(cur_stream, pos, incr, 1);
                    } else {
                        pos = get_master_clock(cur_stream);
                        if (isnan(pos))
                            pos = (double)cur_stream->seek_pos / AV_TIME_BASE;
                        pos += incr;
                        if (cur_stream->ic->start_time != AV_NOPTS_VALUE && pos < cur_stream->ic->start_time / (double)AV_TIME_BASE)
                            pos = cur_stream->ic->start_time / (double)AV_TIME_BASE;
                        stream_seek(cur_stream, (int64_t)(pos * AV_TIME_BASE), (int64_t)(incr * AV_TIME_BASE), 0);
                    }
                break;
            default:
                break;
            }
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (exit_on_mousedown) {
                do_exit(cur_stream);
                break;
            }
            if (event.button.button == SDL_BUTTON_LEFT) {
                static int64_t last_mouse_left_click = 0;
                if (av_gettime_relative() - last_mouse_left_click <= 500000) {
                    toggle_full_screen(cur_stream);
                    cur_stream->force_refresh = 1;
                    last_mouse_left_click = 0;
                } else {
                    last_mouse_left_click = av_gettime_relative();
                }
            }
        case SDL_MOUSEMOTION:
            if (cursor_hidden) {
                SDL_ShowCursor(1);
                cursor_hidden = 0;
            }
            cursor_last_shown = av_gettime_relative();
            if (event.type == SDL_MOUSEBUTTONDOWN) {
                if (event.button.button != SDL_BUTTON_RIGHT)
                    break;
                x = event.button.x;
            } else {
                if (!(event.motion.state & SDL_BUTTON_RMASK))
                    break;
                x = event.motion.x;
            }
                if (seek_by_bytes || cur_stream->ic->duration <= 0) {
                    uint64_t size =  avio_size(cur_stream->ic->pb);
                    stream_seek(cur_stream, size*x/cur_stream->width, 0, 1);
                } else {
                    int64_t ts;
                    int ns, hh, mm, ss;
                    int tns, thh, tmm, tss;
                    tns  = cur_stream->ic->duration / 1000000LL;
                    thh  = tns / 3600;
                    tmm  = (tns % 3600) / 60;
                    tss  = (tns % 60);
                    frac = x / cur_stream->width;
                    ns   = frac * tns;
                    hh   = ns / 3600;
                    mm   = (ns % 3600) / 60;
                    ss   = (ns % 60);
                    av_log(NULL, AV_LOG_INFO,
                           "Seek to %2.0f%% (%2d:%02d:%02d) of total duration (%2d:%02d:%02d)       \n", frac*100,
                            hh, mm, ss, thh, tmm, tss);
                    ts = frac * cur_stream->ic->duration;
                    if (cur_stream->ic->start_time != AV_NOPTS_VALUE)
                        ts += cur_stream->ic->start_time;
                    stream_seek(cur_stream, ts, 0, 0);
                }
            break;
        case SDL_WINDOWEVENT:
            switch (event.window.event) {
                case SDL_WINDOWEVENT_SIZE_CHANGED:
                    screen_width  = cur_stream->width  = event.window.data1;
                    screen_height = cur_stream->height = event.window.data2;
                    if (cur_stream->vis_texture) {
                        SDL_DestroyTexture(cur_stream->vis_texture);
                        cur_stream->vis_texture = NULL;
                    }
                    if (vk_renderer)
                        vk_renderer_resize(vk_renderer, screen_width, screen_height);
                case SDL_WINDOWEVENT_EXPOSED:
                    cur_stream->force_refresh = 1;
            }
            break;
        case SDL_QUIT:
        case FF_QUIT_EVENT:
            do_exit(cur_stream);
            break;
        default:
            break;
        }
    }
}

static int opt_width(void *optctx, const char *opt, const char *arg)
{
    double num;
    int ret = parse_number(opt, arg, OPT_TYPE_INT64, 1, INT_MAX, &num);
    if (ret < 0)
        return ret;

    screen_width = num;
    return 0;
}

static int opt_height(void *optctx, const char *opt, const char *arg)
{
    double num;
    int ret = parse_number(opt, arg, OPT_TYPE_INT64, 1, INT_MAX, &num);
    if (ret < 0)
        return ret;

    screen_height = num;
    return 0;
}

static int opt_format(void *optctx, const char *opt, const char *arg)
{
    file_iformat = av_find_input_format(arg);
    if (!file_iformat) {
        av_log(NULL, AV_LOG_FATAL, "Unknown input format: %s\n", arg);
        return AVERROR(EINVAL);
    }
    return 0;
}

static int opt_sync(void *optctx, const char *opt, const char *arg)
{
    if (!strcmp(arg, "audio"))
        av_sync_type = AV_SYNC_AUDIO_MASTER;
    else if (!strcmp(arg, "video"))
        av_sync_type = AV_SYNC_VIDEO_MASTER;
    else if (!strcmp(arg, "ext"))
        av_sync_type = AV_SYNC_EXTERNAL_CLOCK;
    else {
        av_log(NULL, AV_LOG_ERROR, "Unknown value for %s: %s\n", opt, arg);
        exit(1);
    }
    return 0;
}

static int opt_show_mode(void *optctx, const char *opt, const char *arg)
{
    show_mode = !strcmp(arg, "video") ? SHOW_MODE_VIDEO :
                !strcmp(arg, "waves") ? SHOW_MODE_WAVES :
                !strcmp(arg, "rdft" ) ? SHOW_MODE_RDFT  : SHOW_MODE_NONE;

    if (show_mode == SHOW_MODE_NONE) {
        double num;
        int ret = parse_number(opt, arg, OPT_TYPE_INT, 0, SHOW_MODE_NB-1, &num);
        if (ret < 0)
            return ret;
        show_mode = num;
    }
    return 0;
}

static int opt_input_file(void *optctx, const char *filename)
{
    if (input_filename) {
        av_log(NULL, AV_LOG_FATAL,
               "Argument '%s' provided as input filename, but '%s' was already specified.\n",
                filename, input_filename);
        return AVERROR(EINVAL);
    }
    if (!strcmp(filename, "-"))
        filename = "fd:";
    input_filename = av_strdup(filename);
    if (!input_filename)
        return AVERROR(ENOMEM);

    return 0;
}

static int opt_codec(void *optctx, const char *opt, const char *arg)
{
   const char *spec = strchr(opt, ':');
   const char **name;
   if (!spec) {
       av_log(NULL, AV_LOG_ERROR,
              "No media specifier was specified in '%s' in option '%s'\n",
               arg, opt);
       return AVERROR(EINVAL);
   }
   spec++;

   switch (spec[0]) {
   case 'a' : name = &audio_codec_name;    break;
   case 's' : name = &subtitle_codec_name; break;
   case 'v' : name = &video_codec_name;    break;
   default:
       av_log(NULL, AV_LOG_ERROR,
              "Invalid media specifier '%s' in option '%s'\n", spec, opt);
       return AVERROR(EINVAL);
   }

   av_freep(name);
   *name = av_strdup(arg);
   return *name ? 0 : AVERROR(ENOMEM);
}

static int dummy;

static const OptionDef options[] = {
    CMDUTILS_COMMON_OPTIONS
    { "x",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_width }, "force displayed width", "width" },
    { "y",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_height }, "force displayed height", "height" },
    { "fs",                 OPT_TYPE_BOOL,            0, { &is_full_screen }, "force full screen" },
    { "an",                 OPT_TYPE_BOOL,            0, { &audio_disable }, "disable audio" },
    { "vn",                 OPT_TYPE_BOOL,            0, { &video_disable }, "disable video" },
    { "sn",                 OPT_TYPE_BOOL,            0, { &subtitle_disable }, "disable subtitling" },
    { "ast",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_AUDIO] }, "select desired audio stream", "stream_specifier" },
    { "vst",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_VIDEO] }, "select desired video stream", "stream_specifier" },
    { "sst",                OPT_TYPE_STRING, OPT_EXPERT, { &wanted_stream_spec[AVMEDIA_TYPE_SUBTITLE] }, "select desired subtitle stream", "stream_specifier" },
    { "ss",                 OPT_TYPE_TIME,            0, { &start_time }, "seek to a given position in seconds", "pos" },
    { "t",                  OPT_TYPE_TIME,            0, { &duration }, "play  \"duration\" seconds of audio/video", "duration" },
    { "bytes",              OPT_TYPE_INT,             0, { &seek_by_bytes }, "seek by bytes 0=off 1=on -1=auto", "val" },
    { "seek_interval",      OPT_TYPE_FLOAT,           0, { &seek_interval }, "set seek interval for left/right keys, in seconds", "seconds" },
    { "nodisp",             OPT_TYPE_BOOL,            0, { &display_disable }, "disable graphical display" },
    { "noborder",           OPT_TYPE_BOOL,            0, { &borderless }, "borderless window" },
    { "alwaysontop",        OPT_TYPE_BOOL,            0, { &alwaysontop }, "window always on top" },
    { "volume",             OPT_TYPE_INT,             0, { &startup_volume}, "set startup volume 0=min 100=max", "volume" },
    { "f",                  OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_format }, "force format", "fmt" },
    { "stats",              OPT_TYPE_BOOL,   OPT_EXPERT, { &show_status }, "show status", "" },
    { "fast",               OPT_TYPE_BOOL,   OPT_EXPERT, { &fast }, "non spec compliant optimizations", "" },
    { "genpts",             OPT_TYPE_BOOL,   OPT_EXPERT, { &genpts }, "generate pts", "" },
    { "drp",                OPT_TYPE_INT,    OPT_EXPERT, { &decoder_reorder_pts }, "let decoder reorder pts 0=off 1=on -1=auto", ""},
    { "lowres",             OPT_TYPE_INT,    OPT_EXPERT, { &lowres }, "", "" },
    { "sync",               OPT_TYPE_FUNC, OPT_FUNC_ARG | OPT_EXPERT, { .func_arg = opt_sync }, "set audio-video sync. type (type=audio/video/ext)", "type" },
    { "autoexit",           OPT_TYPE_BOOL,   OPT_EXPERT, { &autoexit }, "exit at the end", "" },
    { "exitonkeydown",      OPT_TYPE_BOOL,   OPT_EXPERT, { &exit_on_keydown }, "exit on key down", "" },
    { "exitonmousedown",    OPT_TYPE_BOOL,   OPT_EXPERT, { &exit_on_mousedown }, "exit on mouse down", "" },
    { "loop",               OPT_TYPE_INT,    OPT_EXPERT, { &loop }, "set number of times the playback shall be looped", "loop count" },
    { "framedrop",          OPT_TYPE_BOOL,   OPT_EXPERT, { &framedrop }, "drop frames when cpu is too slow", "" },
    { "infbuf",             OPT_TYPE_BOOL,   OPT_EXPERT, { &infinite_buffer }, "don't limit the input buffer size (useful with realtime streams)", "" },
    { "window_title",       OPT_TYPE_STRING,          0, { &window_title }, "set window title", "window title" },
    { "left",               OPT_TYPE_INT,    OPT_EXPERT, { &screen_left }, "set the x position for the left of the window", "x pos" },
    { "top",                OPT_TYPE_INT,    OPT_EXPERT, { &screen_top }, "set the y position for the top of the window", "y pos" },
    { "vf",                 OPT_TYPE_FUNC, OPT_FUNC_ARG | OPT_EXPERT, { .func_arg = opt_add_vfilter }, "set video filters", "filter_graph" },
    { "af",                 OPT_TYPE_STRING,          0, { &afilters }, "set audio filters", "filter_graph" },
    { "rdftspeed",          OPT_TYPE_INT, OPT_AUDIO | OPT_EXPERT, { &rdftspeed }, "rdft speed", "msecs" },
    { "showmode",           OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_show_mode}, "select show mode (0 = video, 1 = waves, 2 = RDFT)", "mode" },
    { "i",                  OPT_TYPE_BOOL,            0, { &dummy}, "read specified file", "input_file"},
    { "codec",              OPT_TYPE_FUNC, OPT_FUNC_ARG, { .func_arg = opt_codec}, "force decoder", "decoder_name" },
    { "acodec",             OPT_TYPE_STRING, OPT_EXPERT, {    &audio_codec_name }, "force audio decoder",    "decoder_name" },
    { "scodec",             OPT_TYPE_STRING, OPT_EXPERT, { &subtitle_codec_name }, "force subtitle decoder", "decoder_name" },
    { "vcodec",             OPT_TYPE_STRING, OPT_EXPERT, {    &video_codec_name }, "force video decoder",    "decoder_name" },
    { "autorotate",         OPT_TYPE_BOOL,            0, { &autorotate }, "automatically rotate video", "" },
    { "find_stream_info",   OPT_TYPE_BOOL, OPT_INPUT | OPT_EXPERT, { &find_stream_info },
        "read and decode the streams to fill missing information with heuristics" },
    { "filter_threads",     OPT_TYPE_INT,    OPT_EXPERT, { &filter_nbthreads }, "number of filter threads per graph" },
    { "enable_vulkan",      OPT_TYPE_BOOL,            0, { &enable_vulkan }, "enable vulkan renderer" },
    { "vulkan_params",      OPT_TYPE_STRING, OPT_EXPERT, { &vulkan_params }, "vulkan configuration using a list of key=value pairs separated by ':'" },
    { "hwaccel",            OPT_TYPE_STRING, OPT_EXPERT, { &hwaccel }, "use HW accelerated decoding" },
    { NULL, },
};

static void show_usage(void)
{
    av_log(NULL, AV_LOG_INFO, "Simple media player\n");
    av_log(NULL, AV_LOG_INFO, "usage: %s [options] input_file\n", program_name);
    av_log(NULL, AV_LOG_INFO, "\n");
}

void show_help_default(const char *opt, const char *arg)
{
    av_log_set_callback(log_callback_help);
    show_usage();
    show_help_options(options, "Main options:", 0, OPT_EXPERT);
    show_help_options(options, "Advanced options:", OPT_EXPERT, 0);
    printf("\n");
    show_help_children(avcodec_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avformat_get_class(), AV_OPT_FLAG_DECODING_PARAM);
    show_help_children(avfilter_get_class(), AV_OPT_FLAG_FILTERING_PARAM);
    printf("\nWhile playing:\n"
           "q, ESC              quit\n"
           "f                   toggle full screen\n"
           "p, SPC              pause\n"
           "m                   toggle mute\n"
           "9, 0                decrease and increase volume respectively\n"
           "/, *                decrease and increase volume respectively\n"
           "a                   cycle audio channel in the current program\n"
           "v                   cycle video channel\n"
           "t                   cycle subtitle channel in the current program\n"
           "c                   cycle program\n"
           "w                   cycle video filters or show modes\n"
           "s                   activate frame-step mode\n"
           "left/right          seek backward/forward 10 seconds or to custom interval if -seek_interval is set\n"
           "down/up             seek backward/forward 1 minute\n"
           "page down/page up   seek backward/forward 10 minutes\n"
           "right mouse click   seek to percentage in file corresponding to fraction of width\n"
           "left double-click   toggle full screen\n"
           );
}

/* Called from the main */
int main(int argc, char **argv)
{
    int flags, ret;
    VideoState *is;

    init_dynload();

    av_log_set_flags(AV_LOG_SKIP_REPEATED);
    parse_loglevel(argc, argv, options);

    /* register all codecs, demux and protocols */
#if CONFIG_AVDEVICE
    avdevice_register_all();
#endif
    avformat_network_init();

    signal(SIGINT , sigterm_handler); /* Interrupt (ANSI).    */
    signal(SIGTERM, sigterm_handler); /* Termination (ANSI).  */

    show_banner(argc, argv, options);

    ret = parse_options(NULL, argc, argv, options, opt_input_file);
    if (ret < 0)
        exit(ret == AVERROR_EXIT ? 0 : 1);

    if (!input_filename) {
        show_usage();
        av_log(NULL, AV_LOG_FATAL, "An input file must be specified\n");
        av_log(NULL, AV_LOG_FATAL,
               "Use -h to get full help or, even better, run 'man %s'\n", program_name);
        exit(1);
    }

    if (display_disable) {
        video_disable = 1;
    }
    flags = SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER;
    if (audio_disable)
        flags &= ~SDL_INIT_AUDIO;
    else {
        /* Try to work around an occasional ALSA buffer underflow issue when the
         * period size is NPOT due to ALSA resampling by forcing the buffer size. */
        if (!SDL_getenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE"))
            SDL_setenv("SDL_AUDIO_ALSA_SET_BUFFER_SIZE","1", 1);
    }
    if (display_disable)
        flags &= ~SDL_INIT_VIDEO;
    if (SDL_Init (flags)) {
        av_log(NULL, AV_LOG_FATAL, "Could not initialize SDL - %s\n", SDL_GetError());
        av_log(NULL, AV_LOG_FATAL, "(Did you set the DISPLAY variable?)\n");
        exit(1);
    }

    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);

    if (!display_disable) {
        int flags = SDL_WINDOW_HIDDEN;
        if (alwaysontop)
#if SDL_VERSION_ATLEAST(2,0,5)
            flags |= SDL_WINDOW_ALWAYS_ON_TOP;
#else
            av_log(NULL, AV_LOG_WARNING, "Your SDL version doesn't support SDL_WINDOW_ALWAYS_ON_TOP. Feature will be inactive.\n");
#endif
        if (borderless)
            flags |= SDL_WINDOW_BORDERLESS;
        else
            flags |= SDL_WINDOW_RESIZABLE;

#ifdef SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR
        SDL_SetHint(SDL_HINT_VIDEO_X11_NET_WM_BYPASS_COMPOSITOR, "0");
#endif
        if (hwaccel && !enable_vulkan) {
            av_log(NULL, AV_LOG_INFO, "Enable vulkan renderer to support hwaccel %s\n", hwaccel);
            enable_vulkan = 1;
        }
        if (enable_vulkan) {
            vk_renderer = vk_get_renderer();
            if (vk_renderer) {
#if SDL_VERSION_ATLEAST(2, 0, 6)
                flags |= SDL_WINDOW_VULKAN;
#endif
            } else {
                av_log(NULL, AV_LOG_WARNING, "Doesn't support vulkan renderer, fallback to SDL renderer\n");
                enable_vulkan = 0;
            }
        }
        window = SDL_CreateWindow(program_name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, default_width, default_height, flags);
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        if (!window) {
            av_log(NULL, AV_LOG_FATAL, "Failed to create window: %s", SDL_GetError());
            do_exit(NULL);
        }

        if (vk_renderer) {
            AVDictionary *dict = NULL;

            if (vulkan_params) {
                int ret = av_dict_parse_string(&dict, vulkan_params, "=", ":", 0);
                if (ret < 0) {
                    av_log(NULL, AV_LOG_FATAL, "Failed to parse, %s\n", vulkan_params);
                    do_exit(NULL);
                }
            }
            ret = vk_renderer_create(vk_renderer, window, dict);
            av_dict_free(&dict);
            if (ret < 0) {
                av_log(NULL, AV_LOG_FATAL, "Failed to create vulkan renderer, %s\n", av_err2str(ret));
                do_exit(NULL);
            }
        } else {
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if (!renderer) {
                av_log(NULL, AV_LOG_WARNING, "Failed to initialize a hardware accelerated renderer: %s\n", SDL_GetError());
                renderer = SDL_CreateRenderer(window, -1, 0);
            }
            if (renderer) {
                if (!SDL_GetRendererInfo(renderer, &renderer_info))
                    av_log(NULL, AV_LOG_VERBOSE, "Initialized %s renderer.\n", renderer_info.name);
            }
            if (!renderer || !renderer_info.num_texture_formats) {
                av_log(NULL, AV_LOG_FATAL, "Failed to create window or renderer: %s", SDL_GetError());
                do_exit(NULL);
            }
        }
    }

    is = stream_open(input_filename, file_iformat);
    if (!is) {
        av_log(NULL, AV_LOG_FATAL, "Failed to initialize VideoState!\n");
        do_exit(NULL);
    }

    event_loop(is);

    /* never returns */

    return 0;
}
