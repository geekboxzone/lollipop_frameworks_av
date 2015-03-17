/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ES_QUEUE_H_

#define ES_QUEUE_H_

#include <media/stagefright/foundation/ABase.h>
#include <utils/Errors.h>
#include <media/stagefright/MediaBuffer.h>
#include <utils/List.h>
#include <utils/RefBase.h>
#include "AnotherPacketSource.h"

namespace android {

struct ABuffer;
struct MetaData;

#define SEQUENCE_HEADER_CODE	0x1b3
#define GROUP_START_CODE		0x1b8
#define PICTURE_START_CODE		0x100
#define XMEDIA_BITSTREAM_START_CODE         (0x42564b52) /* RKVB, rockchip video bitstream */
#define AC3SYNCWORD 	0x0B77
#define MPASYNCWORD		0xFFE0
#define DTSSYNCWORD 	0x7FFE8001
#define SYNC_LATM   0x2b7       // 11 bits
#define MAX_SIZE    16*1024
#define ADTS_HEADER_LEN    0x7
#define AAC_TIMESTAMP_LEN   0x4//0x0
#define LATM_HEADER_LEN  0x3
#define MPGA_HEADER_LEN 0x4
#define AAC_FRAME_CACHED_LEN	64*1024
#define min(a,b) ((a)<(b) ? (a) : (b))
#define AV_CH_FRONT_LEFT             0x00000001
#define AV_CH_FRONT_RIGHT            0x00000002
#define AV_CH_FRONT_CENTER           0x00000004
#define AV_CH_LOW_FREQUENCY          0x00000008
#define AV_CH_BACK_LEFT              0x00000010
#define AV_CH_BACK_RIGHT             0x00000020
#define AV_CH_FRONT_LEFT_OF_CENTER   0x00000040
#define AV_CH_FRONT_RIGHT_OF_CENTER  0x00000080
#define AV_CH_BACK_CENTER            0x00000100
#define AV_CH_SIDE_LEFT              0x00000200
#define AV_CH_SIDE_RIGHT             0x00000400
#define AV_CH_TOP_CENTER             0x00000800
#define AV_CH_TOP_FRONT_LEFT         0x00001000
#define AV_CH_TOP_FRONT_CENTER       0x00002000
#define AV_CH_TOP_FRONT_RIGHT        0x00004000
#define AV_CH_TOP_BACK_LEFT          0x00008000
#define AV_CH_TOP_BACK_CENTER        0x00010000
#define AV_CH_TOP_BACK_RIGHT         0x00020000
#define AV_CH_STEREO_LEFT            0x20000000  ///< Stereo downmix.
#define AV_CH_STEREO_RIGHT           0x40000000  ///< See AV_CH_STEREO_LEFT.
#define AV_CH_WIDE_LEFT              0x0000000080000000ULL
#define AV_CH_WIDE_RIGHT             0x0000000100000000ULL
#define AV_CH_SURROUND_DIRECT_LEFT   0x0000000200000000ULL
#define AV_CH_SURROUND_DIRECT_RIGHT  0x0000000400000000ULL
#define AV_CH_LAYOUT_NATIVE          0x8000000000000000ULL
#define AV_CH_LAYOUT_MONO              (AV_CH_FRONT_CENTER)
#define AV_CH_LAYOUT_STEREO            (AV_CH_FRONT_LEFT|AV_CH_FRONT_RIGHT)
#define AV_CH_LAYOUT_2POINT1           (AV_CH_LAYOUT_STEREO|AV_CH_LOW_FREQUENCY)
#define AV_CH_LAYOUT_2_1               (AV_CH_LAYOUT_STEREO|AV_CH_BACK_CENTER)
#define AV_CH_LAYOUT_SURROUND          (AV_CH_LAYOUT_STEREO|AV_CH_FRONT_CENTER)
#define AV_CH_LAYOUT_3POINT1           (AV_CH_LAYOUT_SURROUND|AV_CH_LOW_FREQUENCY)
#define AV_CH_LAYOUT_4POINT0           (AV_CH_LAYOUT_SURROUND|AV_CH_BACK_CENTER)
#define AV_CH_LAYOUT_4POINT1           (AV_CH_LAYOUT_4POINT0|AV_CH_LOW_FREQUENCY)
#define AV_CH_LAYOUT_2_2               (AV_CH_LAYOUT_STEREO|AV_CH_SIDE_LEFT|AV_CH_SIDE_RIGHT)
#define AV_CH_LAYOUT_QUAD              (AV_CH_LAYOUT_STEREO|AV_CH_BACK_LEFT|AV_CH_BACK_RIGHT)
#define AV_CH_LAYOUT_5POINT0           (AV_CH_LAYOUT_SURROUND|AV_CH_SIDE_LEFT|AV_CH_SIDE_RIGHT)
#define AV_CH_LAYOUT_5POINT1           (AV_CH_LAYOUT_5POINT0|AV_CH_LOW_FREQUENCY)
#define AV_CH_LAYOUT_5POINT0_BACK      (AV_CH_LAYOUT_SURROUND|AV_CH_BACK_LEFT|AV_CH_BACK_RIGHT)
#define AV_CH_LAYOUT_5POINT1_BACK      (AV_CH_LAYOUT_5POINT0_BACK|AV_CH_LOW_FREQUENCY)
#define AV_CH_LAYOUT_6POINT0           (AV_CH_LAYOUT_5POINT0|AV_CH_BACK_CENTER)
#define AV_CH_LAYOUT_6POINT0_FRONT     (AV_CH_LAYOUT_2_2|AV_CH_FRONT_LEFT_OF_CENTER|AV_CH_FRONT_RIGHT_OF_CENTER)
#define AV_CH_LAYOUT_HEXAGONAL         (AV_CH_LAYOUT_5POINT0_BACK|AV_CH_BACK_CENTER)
#define AV_CH_LAYOUT_6POINT1           (AV_CH_LAYOUT_5POINT1|AV_CH_BACK_CENTER)
#define AV_CH_LAYOUT_6POINT1_BACK      (AV_CH_LAYOUT_5POINT1_BACK|AV_CH_BACK_CENTER)
#define AV_CH_LAYOUT_6POINT1_FRONT     (AV_CH_LAYOUT_6POINT0_FRONT|AV_CH_LOW_FREQUENCY)
#define AV_CH_LAYOUT_7POINT0           (AV_CH_LAYOUT_5POINT0|AV_CH_BACK_LEFT|AV_CH_BACK_RIGHT)
#define AV_CH_LAYOUT_7POINT0_FRONT     (AV_CH_LAYOUT_5POINT0|AV_CH_FRONT_LEFT_OF_CENTER|AV_CH_FRONT_RIGHT_OF_CENTER)
#define AV_CH_LAYOUT_7POINT1           (AV_CH_LAYOUT_5POINT1|AV_CH_BACK_LEFT|AV_CH_BACK_RIGHT)
#define AV_CH_LAYOUT_7POINT1_WIDE      (AV_CH_LAYOUT_5POINT1|AV_CH_FRONT_LEFT_OF_CENTER|AV_CH_FRONT_RIGHT_OF_CENTER)
#define AV_CH_LAYOUT_7POINT1_WIDE_BACK (AV_CH_LAYOUT_5POINT1_BACK|AV_CH_FRONT_LEFT_OF_CENTER|AV_CH_FRONT_RIGHT_OF_CENTER)
#define AV_CH_LAYOUT_OCTAGONAL         (AV_CH_LAYOUT_5POINT0|AV_CH_BACK_LEFT|AV_CH_BACK_CENTER|AV_CH_BACK_RIGHT)
#define AV_CH_LAYOUT_STEREO_DOWNMIX    (AV_CH_STEREO_LEFT|AV_CH_STEREO_RIGHT)

typedef struct RK_HEVC_PARSER {
    void* (*init)();
    int  (*parser)(void* hevcparserHandle,void *packet,void *outpacket);
    void (*close)(void*  hevcparserHandle);
    void (*flush)(void*  hevcparserHandle);
}RK_HEVC_PAESER_S;

typedef struct tsTimeStamp
{
    uint32_t low_part;    /** low bits of the signed 64 bit tick value */
    uint32_t high_part;   /** high bits of the signed 64 bit tick value */
} tsTimeStamp;
typedef struct TsBitsHeader
{
    uint32_t start_code;
    uint32_t size;
    tsTimeStamp time;
    uint32_t type;
    uint32_t pic_num;
    uint32_t reserved[2];
} TsBitsHeader;
struct LatmAacExtConfig;
struct ElementaryStreamQueue {
    enum Mode {
        H264,
        MPEG2,
        VC1,
        HEVC,
        AC3,
        AAC_LATM,
        AAC_ADTS,
        MP3,
        DTS,
        MPEG4_VIDEO,
        PCM_AUDIO,
    };

    enum Flags {
        // Data appended to the queue is always at access unit boundaries.
        kFlag_AlignedData = 1,
    };
    ElementaryStreamQueue(Mode mode, uint32_t flags = 0);
    void set_player_type(int type);
    status_t appendData(const void *data, size_t size, int64_t timeUs);
    void clear(bool clearFormat);
    void seekflush();

    MediaBuffer * dequeueAccessUnit();

    sp<AnotherPacketSource> getSource();
    sp<MetaData> getFormat();

    virtual ~ElementaryStreamQueue();
private:
    int player_type;
    struct RangeInfo {
        int64_t mTimestampUs;
        size_t mLength;
    };

    Mode mMode;
    uint32_t mFlags;

    sp<ABuffer> mBuffer;
    List<RangeInfo> mRangeInfos;
	List<int64_t> mTimestamps;
    sp<MetaData> mFormat;
    int mFormat_flag ;
    bool seekFlag;
    bool FirstIDRFlag;
    bool Vc1InterlaceFlag;
    int pktStart ;
	int startoffset;
    uint32_t Nextsize;
    int64_t lastTimeus;
    int64_t appendlastTimeus;
    bool spsFlag;
    bool ppsFlag;
    uint8_t *SpsPpsBuf;
    uint32_t spsSize;
	LatmAacExtConfig * mLatmAacExtConfig;
    MediaBuffer * dequeueAccessUnitH264_Wireless();
    MediaBuffer * dequeueAccessUnitH264();
    MediaBuffer * dequeueAccessUnitHEVC();
    MediaBuffer * dequeueAccessUnitMPEG2();
    MediaBuffer * dequeueAccessUnitAC3();
    MediaBuffer * dequeueAccessUnitVC1();
    MediaBuffer * dequeueAccessUnitDTS();
    MediaBuffer * dequeueAccessUnitPCMAudio();
    MediaBuffer * dequeueAccessUnitAAC_ADTS();
    MediaBuffer * dequeueAccessUnitAAC_LATM();
    MediaBuffer * dequeueAccessUnitMP3();
    sp<AnotherPacketSource> mSource;
    RK_HEVC_PAESER_S *HevcParser_api;
    void *hevcparser_handle;
#ifdef ES_DEBUG
    FILE *fp;
#endif

    // consume a logical (compressed) access unit of size "size",
    // returns its timestamp in us (or -1 if no time information).
    int64_t fetchTimestamp(size_t size);
    int64_t fetchTimestampAAC(size_t size);


    DISALLOW_EVIL_CONSTRUCTORS(ElementaryStreamQueue);
};

}  // namespace android

#endif  // ES_QUEUE_H_
