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

//#define LOG_NDEBUG 0
#define LOG_TAG "ESQueue"
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/MediaBuffer.h>

#include "ESQueue.h"
#include "bitstream.h"
#include <media/stagefright/foundation/hexdump.h>
#include <media/stagefright/foundation/ABitReader.h>
#include <media/stagefright/foundation/ABuffer.h>
#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/MediaErrors.h>
#include <media/stagefright/MediaDefs.h>
#include <media/stagefright/MetaData.h>
#include <media/stagefright/Utils.h>

#include "include/avc_utils.h"

#include <netinet/in.h>
#include <dlfcn.h>  // for dlopen/dlclose
#include"vpu_api.h"
static void *gHevcParserLibHandle = NULL;

#include "mpeg4audio.h"
#define LATM_AAC_DEBUG 0
#if LATM_AAC_DEBUG
static FILE *fp = NULL;
void* point = NULL;
#endif
namespace android {
int TS_MPASampleRateTable[4][4] = {{11025, 12000,  8000,   0},     /* MPEG2.5 */
                                   {0,     0,      0,      0},     /* Reserved */
                                   {22050, 24000,  16000,  0},     /* MPEG2 */
                                   {44100, 48000,  32000,  0}};    /* MPEG1 */

int TS_MPABitsRateTable[16][6] = {{0   , 0   , 0   , 0   , 0   , 0  },  //   0000
                                  {32  , 32  , 32  , 32  , 32  , 8  },  //   0001
                                  {64  , 48  , 40  , 64  , 48  , 16 },  //   0010
                                  {96  , 56  , 48  , 96  , 56  , 24 },  //   0011
                                  {128 , 64  , 56  , 128 , 64  , 32 },  //   0100
                                  {160 , 80  , 64  , 160 , 80  , 64 },  //   0101
                                  {192 , 96  , 80  , 192 , 96  , 80 },  //   0110
                                  {224 , 112 , 96  , 224 , 112 , 56 },  //   0111
                                  {256 , 128 , 112 , 256 , 128 , 64 },  //   1000
                                  {288 , 160 , 128 , 288 , 160 , 128},  //   1001
                                  {320 , 192 , 160 , 320 , 192 , 160},  //   1010
                                  {352 , 224 , 192 , 352 , 224 , 112},  //   1011
                                  {384 , 256 , 224 , 384 , 256 , 128},  //   1100
                                  {416 , 320 , 256 , 416 , 320 , 256},  //   1101
                                  {448 , 384 , 320 , 448 , 384 , 320},  //   1110
                                  {-1  , -1  , -1  , -1  , -1  , -1 }}; //   1111
int samplingFreqTable[] =
{
    96000, 88200, 64000, 48000, 44100,
    32000, 24000, 22050, 16000, 12000,
    11025, 8000, 7350
};
int mvl[4/*version*/][4/*layer*/] = {
    {255, 255, 255, 255},
    {255, 255, 255, 255},
    {255, 4, 4, 3},
    {255, 2, 1, 0}
};
int mbitrate[16/*bitrate index*/][5/*mvl*/] = {
    {0, 0, 0, 0, 0},
    {32, 32, 32, 32, 8},
    {64, 48, 40, 48, 16},
    {96, 56, 48, 56, 24},
    {128, 64, 56, 64, 32},
    {160, 80, 64, 80, 40},
    {192, 96, 80, 96, 48},
    {224, 112, 96, 112, 56},
    {256, 128, 112, 128, 64},
    {288, 160, 128, 144, 80},
    {320, 192, 160, 160, 96},
    {352, 224, 192, 176, 112},
    {384, 256, 224, 192, 128},
    {416, 320, 256, 224, 144},
    {448, 384, 320, 256, 160},
    {255, 255, 255, 255, 255}
};
int msamplerate[4/*samplerate index*/][4/*version*/] = {
    {11205, 255, 22050, 44100},
    {12000, 255, 24000, 48000},
    {8000, 255, 16000, 32000},
    {255, 255, 255, 255}
};
//only for latm frame parse
typedef struct AACStruct
{
	uint8_t 		initialized;
	uint8_t 		audio_mux_version_A;
	uint8_t 		frameLengthType;
	int32_t 		frameLength; // faad(may be useful)
} AACStruct;
typedef struct LatmAacExtConfig
{
	//intput buffer point
	uint8_t * pInputBuffer;
	//intput buffer input length
	int32_t   inputBufferCurrentLength;
	//output buffer point
	uint8_t * pOutputBuffer;
	//output buffer output length
	int32_t   outputBufferLength;

	//the follow para from TS stream send the audioconfig data
	MPEG4AudioConfig aacConfig;


	//for decoder latm package
	AACStruct  aacStruct;
}LatmAacExtConfig;
ElementaryStreamQueue::ElementaryStreamQueue(Mode mode, uint32_t flags)
    : mMode(mode),
	 mBuffer(NULL),
     mFormat(NULL),
     HevcParser_api(NULL),
     hevcparser_handle(NULL),
     mFlags(flags) {

	mFormat_flag = 0;
    lastTimeus = 0;
    appendlastTimeus = 0;
    seekFlag = false;
    FirstIDRFlag = false;
    pktStart = 0;
	startoffset = -1;
    Nextsize = 0;
    Vc1InterlaceFlag = false;
    player_type = 0;
    spsFlag = false;
    ppsFlag = false;
    spsSize = 0;
    SpsPpsBuf = NULL;
	mLatmAacExtConfig = NULL;

	if(mMode == AAC_LATM)
	{
		mLatmAacExtConfig = new LatmAacExtConfig;
		if(mLatmAacExtConfig)
		{
			ALOGI("--> mLatmAacExtConfig %p",mLatmAacExtConfig);
			memset(mLatmAacExtConfig, 0 , sizeof(LatmAacExtConfig));
			mLatmAacExtConfig->aacConfig.object_type= -1;
			mLatmAacExtConfig->aacConfig.chan_config= -1;
			mLatmAacExtConfig->aacConfig.sampling_index= -1;
		}
		else
			ALOGI("--> mLatmAacExtConfig new failed");
	}
    if(mMode == HEVC){
        if(gHevcParserLibHandle == NULL){
            gHevcParserLibHandle = dlopen("/system/lib/librk_hevcdec.so", RTLD_LAZY);
            if (gHevcParserLibHandle == NULL) {
                ALOGI("dlopen hevc_hwdec library fail\n");
            }
        }
        if(gHevcParserLibHandle != NULL){
            HevcParser_api = (RK_HEVC_PAESER_S*)malloc(sizeof(RK_HEVC_PAESER_S));
            if(HevcParser_api != NULL){
                HevcParser_api->init = (void* (*)())dlsym(gHevcParserLibHandle, "libHevcParserInit");

                HevcParser_api->parser = (int (*)(void *hevcparserHandle,
                                void *packet,void *outpacket))dlsym(gHevcParserLibHandle, "libHevcParser");

                HevcParser_api->close = (void (*)(void *hevcparserHandle))dlsym(gHevcParserLibHandle, "libHevcParserClose");
                HevcParser_api->flush = (void (*)(void *hevcparserHandle))dlsym(gHevcParserLibHandle, "libHevcParserflush");
           }
           if((HevcParser_api != NULL) && (HevcParser_api->init != NULL)){
                hevcparser_handle = HevcParser_api->init();
           }

        }

    }

#ifdef ES_DEBUG
    fp = NULL;
   switch (mMode) {
        case H264:
        {
            fp = fopen("/sdcard/264es.data","wb+");
            break;
        }
        case HEVC:{
            fp = fopen("/data/video/hevc_ts.bin","wb+");
            break;
        }
        case AAC_LATM:
        {


			break;
        }

        case AAC_ADTS:
        {
            break;
        }

        case MP3:
        {
            break;
        }
        case MPEG2:
        {
             break;
        }
        case DTS:
        {
            break;
        }
        case AC3:
        {

            break;
        }
		case VC1:
        {
            break;
        }
        case PCM:
        {
            break;
        }
        default:
            break;
       }
#endif
}

sp<MetaData> ElementaryStreamQueue::getFormat() {
    return mFormat;
}
sp<AnotherPacketSource> ElementaryStreamQueue::getSource() {
    return mSource;
}
void ElementaryStreamQueue::set_player_type(int type)
{
	ALOGV("ElementaryStreamQueue::set_player_type %d",type);
	player_type = type;
	return;
}

void ElementaryStreamQueue::clear(bool clearFormat) {
    if (mBuffer != NULL) {
        mBuffer->setRange(0, 0);
    }
	mTimestamps.clear();
    mRangeInfos.clear();

    if (clearFormat) {
        mFormat.clear();
        if(mMode == H264){
        spsFlag = false;
        ppsFlag = false;
        spsSize = 0;
        }
    }
}
void ElementaryStreamQueue::seekflush() {
    if(mBuffer != NULL)
        mBuffer->setRange(0, 0);
    mTimestamps.clear();
    mRangeInfos.clear();
    seekFlag = true;
    Nextsize  = 0;
    startoffset = -1;
    pktStart = 0;
    lastTimeus = 0;
	appendlastTimeus = 0;
    if(hevcparser_handle != NULL){
        HevcParser_api->flush(hevcparser_handle);
    }
}

static bool IsSeeminglyValidADTSHeader(
        const uint8_t *ptr, size_t size, size_t *frameLength) {
    if (size < 3) {
        // Not enough data to verify header.
        return false;
    }

    if (ptr[0] != 0xff || (ptr[1] >> 4) != 0x0f) {
        return false;
    }

    unsigned layer = (ptr[1] >> 1) & 3;

    if (layer != 0) {
        return false;
    }

    unsigned ID = (ptr[1] >> 3) & 1;
    unsigned profile_ObjectType = ptr[2] >> 6;

    if (ID == 1 && profile_ObjectType == 3) {
        // MPEG-2 profile 3 is reserved.
        return false;
    }

    size_t frameLengthInHeader =
            ((ptr[3] & 3) << 11) + (ptr[4] << 3) + ((ptr[5] >> 5) & 7);
    if (frameLengthInHeader > size) {
        return false;
    }

    *frameLength = frameLengthInHeader;
    return true;
}

static bool IsSeeminglyValidMPEGAudioHeader(const uint8_t *ptr, size_t size) {
    if (size < 7) {
        // Not enough data to verify header.
        return false;
    }

    if (ptr[0] != 0xff || (ptr[1] >> 5) != 0x07) {
        return false;
    }

    unsigned ID = (ptr[1] >> 3) & 3;

    if (ID == 1) {
        return false;  // reserved
    }

    unsigned layer = (ptr[1] >> 1) & 3;

    if (layer == 0) {
        return false;  // reserved
    }

    unsigned bitrateIndex = (ptr[2] >> 4);

    if (bitrateIndex == 0x0f) {
        return false;  // reserved
    }

    unsigned samplingRateIndex = (ptr[2] >> 2) & 3;

    if (samplingRateIndex == 3) {
        return false;  // reserved
    }

    return true;
}

status_t ElementaryStreamQueue::appendData(
        const void *data, size_t size, int64_t timeUs) {
    if (mBuffer == NULL || mBuffer->size() == 0) {
        switch (mMode) {
            case H264:
            {

                break;
            }
            case HEVC:
            {
                break;
            }

            case AAC_LATM:
            {

                ALOGV("AAC_LATM append the data \n");
                uint8_t *ptr = (uint8_t *)data;

                break;
            }

            case AAC_ADTS:
            {
                uint8_t *ptr = (uint8_t *)data;

#if 0
                if (size < 2 || ptr[0] != 0xff || (ptr[1] >> 4) != 0x0f) {
                    return ERROR_MALFORMED;
                }
#else
                ssize_t startOffset = -1;
                size_t frameLength;
                for (size_t i = 0; i < size; ++i) {
                    if (IsSeeminglyValidADTSHeader(
                            &ptr[i], size - i, &frameLength)) {
                        startOffset = i;
                        break;
                    }
                }

                if (startOffset < 0) {
                    return ERROR_MALFORMED;
                }

                if (startOffset > 0) {
                    ALOGI("found something resembling an AAC syncword at offset %ld",
                         startOffset);
                }

                if (frameLength != size - startOffset) {
                    ALOGD("First ADTS AAC frame length is %zd bytes, "
                          "while the buffer size is %zd bytes.",
                          frameLength, size - startOffset);
                }

                data = &ptr[startOffset];
                size -= startOffset;
#endif
                break;
            }

            case MP3:
            {
                break;
            }
            case MPEG2:
            {
                ALOGV("MPEG2 append the data \n");
                 break;
                }
            case DTS:
            {
                ALOGV("DTS append the data \n");
                break;
            }

            case AC3:
            {
                ALOGV("AC3 append the data \n");

                        break;
            }
			case VC1:
            {
                ALOGV(" VC1 append the data size %d\n",size);
                break;
            }
            case PCM_AUDIO:
            {
                break;
            }

            default:
                return OK;
                break;
        }
    }

    size_t neededSize = (mBuffer == NULL ? 0 : mBuffer->size()) + size;
    if (mBuffer == NULL || neededSize > mBuffer->capacity()) {
        neededSize = (neededSize + 65535) & ~65535;

        ALOGV("resizing buffer to size %d", neededSize);

        sp<ABuffer> buffer = new ABuffer(neededSize);
        if (mBuffer != NULL) {
            memcpy(buffer->data(), mBuffer->data(), mBuffer->size());
            buffer->setRange(0, mBuffer->size());
        } else {
            buffer->setRange(0, 0);
        }

        mBuffer = buffer;
    }

    memcpy(mBuffer->data() + mBuffer->size(), data, size);
    mBuffer->setRange(0, mBuffer->size() + size);
#ifdef ES_DEBUG
 /* if(mMode == H264)
  {
     fwrite(data,1,size,fp);
     fflush(fp);
  }*/
#endif
    //when the mode is H264 the timestampe must use mTimestamps vendor case
    if(mMode == MPEG2 ||mMode == VC1 ||mMode == PCM_AUDIO ||mMode == HEVC || (mMode == H264 && player_type != 4)
            || mMode == AAC_ADTS)
    {
      if(timeUs < 0)
        {
            timeUs = appendlastTimeus;
        }
        appendlastTimeus = timeUs;
        RangeInfo info;
        info.mLength = size;
        info.mTimestampUs = timeUs;
        mRangeInfos.push_back(info);
   }
   else if (timeUs >= 0)
    {
        if (timeUs >0) {
            appendlastTimeus = timeUs;
        }

        if ((appendlastTimeus >0) && (timeUs ==0)) {
            mTimestamps.push_back(appendlastTimeus);
        } else {
            mTimestamps.push_back(timeUs);
        }
    }

    return OK;

}

MediaBuffer * ElementaryStreamQueue::dequeueAccessUnit() {
#if 0
    if ((mFlags & kFlag_AlignedData) && mMode == H264) {
        if (mRangeInfos.empty()) {
            return NULL;
        }

        RangeInfo info = *mRangeInfos.begin();
        mRangeInfos.erase(mRangeInfos.begin());

        sp<ABuffer> accessUnit = new ABuffer(info.mLength);
        memcpy(accessUnit->data(), mBuffer->data(), info.mLength);
        accessUnit->meta()->setInt64("timeUs", info.mTimestampUs);

        memmove(mBuffer->data(),
                mBuffer->data() + info.mLength,
                mBuffer->size() - info.mLength);

        mBuffer->setRange(0, mBuffer->size() - info.mLength);

        if (mFormat == NULL) {
            mFormat = MakeAVCCodecSpecificData(accessUnit);
        }

        return accessUnit;
    }
#endif
    switch (mMode) {
        case H264:
			if(player_type == 3)
				return dequeueAccessUnitH264_Wireless();
			else
            return dequeueAccessUnitH264();
        case HEVC:
            return dequeueAccessUnitHEVC();
        case MPEG2:

            return dequeueAccessUnitMPEG2();
		case VC1:

			 return dequeueAccessUnitVC1();
        case AC3:

            return dequeueAccessUnitAC3();
        case DTS:

            return dequeueAccessUnitDTS();

		case MP3:

			return dequeueAccessUnitMP3();
        case AAC_LATM:

			return dequeueAccessUnitAAC_LATM();
		case AAC_ADTS:

			return dequeueAccessUnitAAC_ADTS();
        case PCM_AUDIO:
            return dequeueAccessUnitPCMAudio();
        default:
            return NULL;
    }
}


int64_t ElementaryStreamQueue::fetchTimestamp(size_t size) {
    int64_t timeUs = -1;
    bool first = true;

    while (size > 0) {
        if(mRangeInfos.empty()){
            return -1;
        }

        RangeInfo *info = &*mRangeInfos.begin();

        if (first) {
            timeUs = info->mTimestampUs;
            first = false;
        }

        if (info->mLength > size) {
            info->mLength -= size;

            if (first) {
                info->mTimestampUs = -1;
            }

            size = 0;
        } else {
            size -= info->mLength;

            mRangeInfos.erase(mRangeInfos.begin());
            info = NULL;
        }
    }

    if (timeUs == 0ll) {
        ALOGV("Returning 0 timestamp");
    }

    return timeUs;
}

struct NALPosition {
    size_t nalOffset;
    size_t nalSize;
};

MediaBuffer *ElementaryStreamQueue::dequeueAccessUnitH264() {
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    Vector<NALPosition> nals;

    size_t totalSize = 0;

    status_t err;
    const uint8_t *nalStart;
    size_t nalSize;
    bool foundSlice = false;
    while ((err = getNextNALUnit(&data, &size, &nalStart, &nalSize)) == OK) {
        //CHECK_GT(nalSize, 0u);
        if(nalSize <= 0)
        {
            continue;
        }
        unsigned nalType = nalStart[0] & 0x1f;
        if((nalType== 7)&& !spsFlag)
        {
        	if(nalSize + 4 > 1024)
			{
				ALOGD("sps is boo big,may be something wrong");
				continue;
			}
            SpsPpsBuf = (uint8_t*)malloc(1024);
            memcpy(SpsPpsBuf,(uint8_t*)nalStart-4,nalSize+4);
            spsSize = nalSize+4;
            spsFlag = true;
        }
        if(spsFlag &&(nalType==8)&& !ppsFlag)
        {
            if(nalSize + spsSize + 4 + 4 > 1024)
			{
				continue;
			}
             memcpy(SpsPpsBuf+spsSize,(uint8_t*)nalStart-4,nalSize+4);
             spsSize += nalSize+4;
             ppsFlag = true;
        }
        if(spsFlag&&ppsFlag&&(mFormat == NULL))
        {
             sp<ABuffer> accessdata = new ABuffer(spsSize);
             if(SpsPpsBuf != NULL) {
             memcpy(accessdata->data(),SpsPpsBuf,spsSize);
             }else{
                continue;
             }
             mFormat = MakeAVCCodecSpecificData(accessdata);
             if(mFormat != NULL){
                mFormat->setInt32(kKeyisTs, 1);
             }
             if(mFormat != NULL && mSource == NULL)
             {
                mSource = new AnotherPacketSource(mFormat);
             }
             if(mFormat != NULL && mSource != NULL)
             {
                mSource->setFormat(mFormat);
             }
             if(SpsPpsBuf)
             {
                free(SpsPpsBuf);
                SpsPpsBuf = NULL;
                if(mFormat == NULL ){
                    spsSize = 0;
                    spsFlag = false;
                    ppsFlag = false;
                }
             }
        }
        bool flush = false;
        if (nalType == 1 || nalType == 5) {
            if (foundSlice) {
                ABitReader br(nalStart + 1, nalSize);
                unsigned first_mb_in_slice = parseUE(&br);

                if (first_mb_in_slice == 0) {
                    // This slice starts a new frame.

                    flush = true;
                }
            }

            foundSlice = true;
        } else if ((nalType == 9 || nalType == 7) && foundSlice) {
            // Access unit delimiter and SPS will be associated with the
            // next frame.

            flush = true;
        }

        if (flush) {
            // The access unit will contain all nal units up to, but excluding
            // the current one, separated by 0x00 0x00 0x00 0x01 startcodes.

            size_t auSize = 4 * nals.size() + totalSize;
            MediaBuffer * accessUnit = new MediaBuffer(auSize);

#if !LOG_NDEBUG
            AString out;
#endif

            size_t dstOffset = 0;
            for (size_t i = 0; i < nals.size(); ++i) {
                const NALPosition &pos = nals.itemAt(i);

                unsigned nalType = mBuffer->data()[pos.nalOffset] & 0x1f;

#if !LOG_NDEBUG
                char tmp[128];
                sprintf(tmp, "0x%02x", nalType);
                if (i > 0) {
                    out.append(", ");
                }
                out.append(tmp);
#endif
                memcpy(accessUnit->data() + dstOffset, "\x00\x00\x00\x01", 4);

                memcpy(accessUnit->data() + dstOffset + 4,
                       mBuffer->data() + pos.nalOffset,
                       pos.nalSize);

                dstOffset += pos.nalSize + 4;
            }
            size_t nextScan = 0;

            const NALPosition &pos = nals.itemAt(nals.size() - 1);
            nextScan = pos.nalOffset + pos.nalSize;
            //remove the zero at the end of frame add by csy
            {
                int32_t endOffset = accessUnit->range_length();
                uint8_t *data1 = (uint8_t*) accessUnit->data();

                while (endOffset > 1 && data1[endOffset - 1] == 0x00) {
                    --endOffset;
                }
                accessUnit->set_range(0,endOffset);
            }
            memmove(mBuffer->data(),
                    mBuffer->data() + nextScan,
                    mBuffer->size() - nextScan);

            mBuffer->setRange(0, mBuffer->size() - nextScan);
            int64_t timeUs = 0;


			if(player_type == 4/*live_tv*/)
			{

				if(mTimestamps.size() == 0)
				{
					timeUs = lastTimeus;
					ALOGV("no timestampe in quen");
				}
				else
				{
					timeUs = *mTimestamps.begin();
					mTimestamps.erase(mTimestamps.begin());
					lastTimeus	= timeUs;
				}
			}
			else
			{
				timeUs = fetchTimestamp(nextScan);

	            if(timeUs < 0){
	                ALOGE("fetch timeUs fail \n");
	                timeUs = 0;
	            }
			}
            accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
            if (mFormat == NULL) {
#ifdef ES_DEBUG
                fwrite(accessUnit->data(),1,accessUnit->range_length(),fp);
                fflush(fp);
#endif
                sp<ABuffer> accessdata = new ABuffer(accessUnit->range_length());
                memcpy(accessdata->data(),(uint8_t *)accessUnit->data(),accessUnit->range_length());
                mFormat = MakeAVCCodecSpecificData(accessdata);
                 if(mFormat != NULL && mSource == NULL)
                 {
                    mSource = new AnotherPacketSource(mFormat);
                 }

                 if(mFormat != NULL && mSource != NULL)
                 {
                    mSource->setFormat(mFormat);
                 }
                 if(mFormat == NULL){
                    accessUnit->release();
                    return NULL;
                 }
            }
            return accessUnit;
        }

        NALPosition pos;
        pos.nalOffset = nalStart - mBuffer->data();
        pos.nalSize = nalSize;
        nals.push(pos);
        totalSize += nalSize;
    }
    if(err != (status_t)-EAGAIN)
    {
        ALOGV("no nal header in this slice");
        mBuffer->setRange(0,0);
    }
    return NULL;
}
MediaBuffer *ElementaryStreamQueue::dequeueAccessUnitHEVC() {
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    Vector<NALPosition> nals;
    size_t totalSize = 0;
    status_t err;
    const uint8_t *nalStart;
    size_t nalSize;
    bool foundSlice = false;
    int nextScan = 0;
    if(hevcparser_handle != NULL){
       VideoPacket_t packet;
       ParserOut_t   parserpacket;
       uint8_t *outbuff = NULL;
       int len = 0;
       memset(&packet,0,sizeof(packet));
       memset(&parserpacket,0,sizeof(parserpacket));
       packet.data = mBuffer->data();
       packet.size = size;
       do{
           len = HevcParser_api->parser(hevcparser_handle,&packet,&parserpacket);
           if (mFormat == NULL) {
                if(parserpacket.width > 0){
                    sp<MetaData> meta = new MetaData;
                    meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_VIDEO_HEVC);
                   	meta->setInt32(kKeyWidth,parserpacket.width);
            		meta->setInt32(kKeyHeight,parserpacket.height);
                    mFormat = meta;
                    mSource = new AnotherPacketSource(mFormat);
                }
           }
           if(parserpacket.size > 0 && seekFlag){
                if(parserpacket.nFlags != 1){
                   parserpacket.size = 0;
                   fetchTimestamp(parserpacket.size);
                }else{
                    seekFlag = false;
                }
           }
           if(parserpacket.size > 0){
                MediaBuffer * accessUnit = new MediaBuffer(parserpacket.size);
                memcpy(accessUnit->data(),parserpacket.data,parserpacket.size);
                int64_t timeUs = 0;
                timeUs = fetchTimestamp(parserpacket.size);
                accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
                if(mSource != NULL){
                    mSource->queueAccessUnit(accessUnit);
                }else{
                    accessUnit->release();
                    accessUnit = NULL;
                }
           }
           packet.data += len;
           packet.size -= len;
       }while(packet.size);
       mBuffer->setRange(0,0);
       return NULL;
    }else{
        if (mFormat == NULL) {
            ALOGI("creat mFormat");
            sp<MetaData> meta = new MetaData;
            meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_VIDEO_HEVC);
           	meta->setInt32(kKeyWidth,1280);
    		meta->setInt32(kKeyHeight,720);
            mFormat = meta;
            mSource = new AnotherPacketSource(mFormat);
        }
        while ((err = getNextNALUnit(&data, &size, &nalStart, &nalSize)) == OK) {
            CHECK_GT(nalSize, 0u);
            unsigned nalType = (nalStart[0]>> 1) & 0x3f;
            MediaBuffer * accessUnit = new MediaBuffer(nalSize);
            memcpy(accessUnit->data(),nalStart,nalSize);
            int64_t timeUs = 0;
            accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
            mSource->queueAccessUnit(accessUnit);
            nextScan = nalStart - mBuffer->data() + nalSize;
        }
        if(nextScan){
            memmove(mBuffer->data(),
                            mBuffer->data() + nextScan,
                            mBuffer->size() - nextScan);
            mBuffer->setRange(0, mBuffer->size() - nextScan);
        }
    }
    return NULL;
}
#if 0
MediaBuffer *ElementaryStreamQueue::dequeueAccessUnitH264_Wireless() {
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    Vector<NALPosition> nals;

    size_t totalSize = 0;

    status_t err;
    const uint8_t *nalStart;
    size_t nalSize;
    bool foundSlice = false;
	if(mFormat == NULL)
	{
		sp<MetaData> meta = new MetaData;
		meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_VIDEO_AVC);

		mFormat = meta;
		mFormat->setInt32(kKeyWidth,1280);
		mFormat->setInt32(kKeyHeight,720);
		MediaBuffer * accessUnit = new MediaBuffer(0);
		return accessUnit;
	}
    while ((err = getNextNALUnit(&data, &size, &nalStart, &nalSize)) == OK) {
        CHECK_GT(nalSize, 0u);
        unsigned nalType = nalStart[0] & 0x1f;
        if((nalType== 7)&& !spsFlag)
        {
        	if(nalSize + 4 > 1024)
			{
				ALOGD("sps is boo big,may be something wrong");
				continue;
			}
            SpsPpsBuf = (uint8_t*)malloc(1024);
            memcpy(SpsPpsBuf,(uint8_t*)nalStart-4,nalSize+4);
            spsSize = nalSize+4;
            spsFlag = true;
        }
        if(spsFlag &&(nalType==8)&& !ppsFlag)
        {
             memcpy(SpsPpsBuf+spsSize,(uint8_t*)nalStart-4,nalSize+4);
             spsSize += nalSize+4;
             ppsFlag = true;
        }
        if(spsFlag&&ppsFlag&&(mFormat == NULL))
        {
             sp<ABuffer> accessdata = new ABuffer(spsSize);
             memcpy(accessdata->data(),SpsPpsBuf,spsSize);
             mFormat = MakeAVCCodecSpecificData(accessdata);
             if(mFormat != NULL && mSource == NULL)
             {
                mSource = new AnotherPacketSource(mFormat);
             }
             if(mFormat != NULL && mSource != NULL)
             {
                mSource->setFormat(mFormat);
             }
             if(SpsPpsBuf)
             {
                free(SpsPpsBuf);
                SpsPpsBuf = NULL;
             }
        }
        bool flush = false;
        if (nalType == 1 || nalType == 5) {
            if (1){//foundSlice) {
                ABitReader br(nalStart + 1, nalSize);
                unsigned first_mb_in_slice = parseUE(&br);

                if (first_mb_in_slice == 0) {
                    // This slice starts a new frame.

                    flush = true;
                }
            }

            foundSlice = true;
        } else if ((nalType == 9 || nalType == 7) && foundSlice) {
            // Access unit delimiter and SPS will be associated with the
            // next frame.

            flush = true;
        }
	  else if(nalType != 9 && nalType != 7 && nalType != 1 && nalType != 5 && nalType != 8   )
	  {
		ALOGV("naltype %d size %d mTimestamps.size() %d",nalType,nalSize,mTimestamps.size());
		if(nalType == 0xe && mTimestamps.size() > 0)
		{
                mTimestamps.erase(mTimestamps.begin());
		}
	 	continue;
	  }
        NALPosition pos;
        pos.nalOffset = nalStart - mBuffer->data();
        pos.nalSize = nalSize;
        nals.push(pos);
        totalSize += nalSize;

        if (flush) {
            // The access unit will contain all nal units up to, but excluding
            // the current one, separated by 0x00 0x00 0x00 0x01 startcodes.

            size_t auSize = 4 * nals.size() + totalSize;
            MediaBuffer * accessUnit = new MediaBuffer(auSize);

#if !LOG_NDEBUG
            AString out;
#endif

            size_t dstOffset = 0;
            for (size_t i = 0; i < nals.size(); ++i) {
                const NALPosition &pos = nals.itemAt(i);

                unsigned nalType = mBuffer->data()[pos.nalOffset] & 0x1f;

#if !LOG_NDEBUG
                char tmp[128];
                sprintf(tmp, "0x%02x", nalType);
                if (i > 0) {
                    out.append(", ");
                }
                out.append(tmp);
#endif
                memcpy(accessUnit->data() + dstOffset, "\x00\x00\x00\x01", 4);

                memcpy(accessUnit->data() + dstOffset + 4,
                       mBuffer->data() + pos.nalOffset,
                       pos.nalSize);

                dstOffset += pos.nalSize + 4;
            }
            size_t nextScan = 0;

            const NALPosition &pos = nals.itemAt(nals.size() - 1);
            nextScan = pos.nalOffset + pos.nalSize;

            memmove(mBuffer->data(),
                    mBuffer->data() + nextScan,
                    mBuffer->size() - nextScan);

            mBuffer->setRange(0, mBuffer->size() - nextScan);
            int64_t timeUs = 0;
            if(mTimestamps.size() == 0)
            {
                timeUs = lastTimeus;
                ALOGV("no timestampe in quen");
            }
            else
            {
                timeUs = *mTimestamps.begin();
                mTimestamps.erase(mTimestamps.begin());
                lastTimeus  = timeUs;
            }

            accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
		#if 0
            if (mFormat == NULL) {

                sp<ABuffer> accessdata = new ABuffer(accessUnit->range_length());
                memcpy(accessdata->data(),(uint8_t *)accessUnit->data(),accessUnit->range_length());
                mFormat = MakeAVCCodecSpecificData(accessdata);
            }
		#else
		if(mFormat_flag==0)
		{
			sp<ABuffer> accessdata = new ABuffer(accessUnit->range_length());
                	memcpy(accessdata->data(),(uint8_t *)accessUnit->data(),accessUnit->range_length());
			mFormat_flag = MakeAVCCodecSpecificData_Wimo(accessdata,mFormat);
                 if(mFormat != NULL && mSource == NULL)
                 {
                    mSource = new AnotherPacketSource(mFormat);
                 }

                 if(mFormat != NULL && mSource != NULL)
                 {
                    mSource->setFormat(mFormat);
                 }
			ALOGD("dequeueAccess 1280 720");
			//mFormat->setInt32(kKeyWidth,1280);
		//	mFormat->setInt32(kKeyHeight,720);
		}
		#endif
            return accessUnit;
        }

    }
    if(err != (status_t)-EAGAIN)
    {
        ALOGE("no nal header in this slice");
        mBuffer->setRange(0,0);
    }
    return NULL;
}
#else
MediaBuffer *ElementaryStreamQueue::dequeueAccessUnitH264_Wireless()

{
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    Vector<NALPosition> nals;

    size_t totalSize = 0;

    status_t err;
    const uint8_t *nalStart;
    size_t nalSize;
    bool foundSlice = false;

    while ((err = getNextNALUnit(&data, &size, &nalStart, &nalSize)) == OK) {
 //       CHECK_GT(nalSize, 0u);
        if(nalSize <= 0)
        {
            continue;
        }

        unsigned nalType = nalStart[0] & 0x1f;
        if((nalType== 7)&& !spsFlag)
        {
        	if(nalSize + 4 > 1024)
			{
				ALOGD("sps is boo big,may be something wrong");
				continue;
			}
            SpsPpsBuf = (uint8_t*)malloc(1024);
            memcpy(SpsPpsBuf,(uint8_t*)nalStart-4,nalSize+4);
            spsSize = nalSize+4;
            spsFlag = true;
        }
        if(spsFlag &&(nalType==8)&& !ppsFlag)
        {
            if(nalSize + spsSize + 4 + 4 > 1024)
			{
				continue;
			}
             memcpy(SpsPpsBuf+spsSize,(uint8_t*)nalStart-4,nalSize+4);
             spsSize += nalSize+4;
             ppsFlag = true;
        }
        if(spsFlag&&ppsFlag&&(mFormat == NULL))
        {
             sp<ABuffer> accessdata = new ABuffer(spsSize);

             if(SpsPpsBuf != NULL) {
             memcpy(accessdata->data(),SpsPpsBuf,spsSize);
             }else{
                continue;
             }
             mFormat = MakeAVCCodecSpecificData(accessdata);
             if(mFormat != NULL){
                mFormat->setInt32(kKeyisTs, 1);
             }
             if(mFormat != NULL && mSource == NULL)
             {
                mSource = new AnotherPacketSource(mFormat);
             }
             if(mFormat != NULL && mSource != NULL)
             {
                mSource->setFormat(mFormat);
             }
             if(SpsPpsBuf)
             {
                free(SpsPpsBuf);
                SpsPpsBuf = NULL;
                if(mFormat == NULL ){
                    spsSize = 0;
                    spsFlag = false;
                    ppsFlag = false;
                }
             }
        }
        bool flush = false;
        if (nalType == 1 || nalType == 5) {
            if (foundSlice) {
                ABitReader br(nalStart + 1, nalSize);
                unsigned first_mb_in_slice = parseUE(&br);

                if (first_mb_in_slice == 0) {
                    // This slice starts a new frame.

                    flush = true;
                }
            }

            foundSlice = true;
        } else if ((nalType == 9 || nalType == 7||nalType==8) && foundSlice) {
            // Access unit delimiter and SPS will be associated with the
            // next frame.

            flush = true;
        }
	  else if(0)//nalType != 9 && nalType != 7 && nalType != 1 && nalType != 5 && nalType != 8   )
	  {
		ALOGV("naltype %d size %d mTimestamps.size() %d",nalType,nalSize,mTimestamps.size());
		if(nalType == 0xe && mTimestamps.size() > 0)
		{
                mTimestamps.erase(mTimestamps.begin());
		}
	 	continue;
	  }


        if (flush) {
            // The access unit will contain all nal units up to, but excluding
            // the current one, separated by 0x00 0x00 0x00 0x01 startcodes.

            size_t auSize = 4 * nals.size() + totalSize;
            MediaBuffer * accessUnit = new MediaBuffer(auSize);

#if !LOG_NDEBUG
            AString out;
#endif

            size_t dstOffset = 0;
            for (size_t i = 0; i < nals.size(); ++i) {
                const NALPosition &pos = nals.itemAt(i);

                unsigned nalType = mBuffer->data()[pos.nalOffset] & 0x1f;

#if !LOG_NDEBUG
                char tmp[128];
                sprintf(tmp, "0x%02x", nalType);
                if (i > 0) {
                    out.append(", ");
                }
                out.append(tmp);
#endif
                memcpy(accessUnit->data() + dstOffset, "\x00\x00\x00\x01", 4);

                memcpy(accessUnit->data() + dstOffset + 4,
                       mBuffer->data() + pos.nalOffset,
                       pos.nalSize);

                dstOffset += pos.nalSize + 4;
            }
            size_t nextScan = 0;

            const NALPosition &pos = nals.itemAt(nals.size() - 1);
            nextScan = pos.nalOffset + pos.nalSize;

            memmove(mBuffer->data(),
                    mBuffer->data() + nextScan,
                    mBuffer->size() - nextScan);

            mBuffer->setRange(0, mBuffer->size() - nextScan);
            int64_t timeUs = 0;
			timeUs = fetchTimestamp(nextScan);

	        if(timeUs < 0){
	            ALOGE("fetch timeUs fail \n");
	            timeUs = 0;
			}
            accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
            if (mFormat == NULL) {

                sp<ABuffer> accessdata = new ABuffer(accessUnit->range_length());
                memcpy(accessdata->data(),(uint8_t *)accessUnit->data(),accessUnit->range_length());
                mFormat = MakeAVCCodecSpecificData(accessdata);
                 if(mFormat != NULL && mSource == NULL)
                 {
                    mSource = new AnotherPacketSource(mFormat);
                 }

                 if(mFormat != NULL && mSource != NULL)
                 {
                    mSource->setFormat(mFormat);
                 }
                 if(mFormat == NULL){
                    accessUnit->release();
                    return NULL;
                 }
            }

            return accessUnit;
        }
        NALPosition pos;
        pos.nalOffset = nalStart - mBuffer->data();
        pos.nalSize = nalSize;
        nals.push(pos);
        totalSize += nalSize;

    }
    if(err != (status_t)-EAGAIN)
    {
        ALOGE("no nal header in this slice");
        mBuffer->setRange(0,0);
    }
    return NULL;
}



#endif

MediaBuffer * ElementaryStreamQueue::dequeueAccessUnitMPEG2() {
    const uint8_t *data = mBuffer->data() + Nextsize;
    size_t size = mBuffer->size()- Nextsize;
    size_t auSize = 0;
    size_t offset = 0;
    bool skipFlag = false;
    if(seekFlag){
        for (;;) {

            while (offset < size && data[offset] != 0x01) {
                ++offset;
            }
            if (offset == size) {
                mBuffer->setRange(0,0);
                int64_t timeUs = fetchTimestamp(size);
                ALOGV("seek skip current pes ");
                return NULL;
            }
            if(data[offset - 1] == 0x00 && data[offset - 2] == 0x00&&data[offset+1] == 0x00) {
                ALOGV("seek PICTURE_START_CODE \n");
                size_t tempoffset = offset + 2;
                if(mBuffer->size() - tempoffset > 2)
                {
                    ABitReader bits(mBuffer->data() + tempoffset, mBuffer->size() - tempoffset);
                    bits.skipBits(10);
                    int32_t codeType = bits.getBits(3);
                    if(codeType == 1)
                    {
                        offset -= 2;
                        memmove(mBuffer->data(), mBuffer->data() + offset,
                        mBuffer->size() - offset);
                        mBuffer->setRange(0, mBuffer->size() - offset);
                        seekFlag = false;
                        ALOGV("set seek flag ok");
                        int64_t timeUs = fetchTimestamp(offset);
                        return NULL;
                     }

                }

            }
            ++offset;
        }
    }

    for (;;) {
        while (offset < (size -1) && data[offset] != 0x01) {
            ++offset;
        }
        if(offset < 2)
        {
            ++offset;
            continue;
        }
        if (offset == (size - 1)) {
            if(startoffset >= 0)
            {
                if(startoffset > 0)
                {
                    uint8_t *buf = (uint8_t *)malloc(mBuffer->size() - startoffset);
                    if(buf)
                    {
                         memcpy(buf, mBuffer->data() + startoffset,mBuffer->size() - startoffset);
                         memcpy(mBuffer->data(),buf,mBuffer->size() - startoffset);
                         free(buf);
                    }
                    mBuffer->setRange(0, mBuffer->size() - startoffset);
                    Nextsize += offset - startoffset - 2;
                    fetchTimestamp(startoffset);
                    startoffset = 0;
                }
                else
                {
                     Nextsize += offset - 2;
                }
            }
            return NULL;
            // seqHeader without/with extension
        }
        if (data[offset - 1] == 0x00 && data[offset - 2] == 0x00 && data[offset+1] == 0xB3) {
            ALOGV("SEQUENCE_HEADER_CODE found \n");
            if (mFormat == NULL) {
                size_t tempoffset = offset + Nextsize + 2;
                ALOGV("tempoffset = %d \n",tempoffset);
                ABitReader bits(mBuffer->data() + tempoffset, mBuffer->size() - tempoffset);


                sp<MetaData> meta = new MetaData;
                uint32_t width = bits.getBits(12);
                uint32_t height = bits.getBits(12);
                ALOGD("width = %d,height = %d \n",width,height);
                meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_VIDEO_MPEG2);
                meta->setInt32(kKeyWidth, width);
                meta->setInt32(kKeyHeight, height);
                mFormat = meta;
                if (pktStart != 0)
                {
                    skipFlag = true;
                    offset -= 2;
                    break;
                }
            }
            if (pktStart == 0) {
                startoffset = offset - 2;
			}
            else
            {
                offset -= 2;
                break;
            }
        } else if(data[offset] == 0x01 && data[offset - 1] == 0x00 && data[offset - 2] == 0x00&&data[offset+1] == 0x00) {
            ALOGV("PICTURE_START_CODE \n");
            if (pktStart == 0) {
					pktStart = 1;
					if (startoffset == -1) {
						startoffset = offset - 2;
					}
			}
            else
            {
                offset -= 2;
                break;
            }
        }
        ++offset;
    }
    offset += Nextsize;
    auSize = offset - startoffset;
    Nextsize  = 0;
    startoffset = -1;
    pktStart = 0;
    int64_t timeUs = 0;

    if((mFormat == NULL) || skipFlag ||!auSize)
    {
        uint8_t *buf = (uint8_t *)malloc(mBuffer->size() - offset);
        if(buf)
        {
             memcpy(buf, mBuffer->data() + offset,mBuffer->size() - offset);
             memcpy(mBuffer->data(),buf,mBuffer->size() - offset);
             free(buf);
        }
        ALOGV("skip the data before SEQUENCE_HEADER_CODE found \n");
                mBuffer->setRange(0, mBuffer->size() - offset);
        fetchTimestamp(offset);
                // hexdump(csd->data(), csd->size());


                return NULL;
            }
    timeUs = fetchTimestamp(auSize);
    ALOGV("timeUs input = %lld",timeUs);
    uint32_t temptimeUs = (uint32_t)(timeUs/1000);
    MediaBuffer *accessUnit = new MediaBuffer(auSize+sizeof(TsBitsHeader));
    TsBitsHeader *h = (TsBitsHeader *)accessUnit->data();
    h->start_code = XMEDIA_BITSTREAM_START_CODE;
	h->size = auSize;
	h->time.low_part = (uint32_t)temptimeUs;
	h->time.high_part = 0;
	h->type = 0;
	h->pic_num = 0;
	h->reserved[0] = 0;
	h->reserved[1] = 0;
    memcpy(accessUnit->data()+sizeof(TsBitsHeader),(uint8_t*)mBuffer->data(),auSize);
#ifdef ES_DEBUG
#endif
    {
        uint8_t *buf = (uint8_t *)malloc(mBuffer->size() - offset);
        if(buf)
        {
             memcpy(buf, mBuffer->data() + offset,mBuffer->size() - offset);
             memcpy(mBuffer->data(),buf,mBuffer->size() - offset);
             free(buf);
        }
    }
            // Picture start



                mBuffer->setRange(0, mBuffer->size() - offset);



    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);


                // hexdump(accessUnit->data(), accessUnit->size());

                return accessUnit;
            }
MediaBuffer *ElementaryStreamQueue::dequeueAccessUnitAC3() {
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    size_t auSize = size;
    if(!size)
    {
        return NULL;
    }
    if (mFormat == NULL) {
        int32_t kSamplingFreq = 0;
        int32_t channel_configuration = 1;
        uint16_t code = 0xFFFF;
        int TS_AC3SampleRateTable[4] = {48000, 44100, 32000, 0};
        ABitReader bits(mBuffer->data(), mBuffer->size());
        do {
            if(bits.numBitsLeft()){
            code = (code << 8) | bits.getBits(8);
            }else{
                uint32_t code1 = 0xFFFFFFFF;
                ABitReader bits1(mBuffer->data(), mBuffer->size());
                do{
                    if(bits1.numBitsLeft()){
                        code1 = (code1 << 8) | bits1.getBits(8);
                    }
                    else{
                        if(mTimestamps.size()> 2)
                        {
                            int64_t timeUs = *mTimestamps.begin();
                            mTimestamps.erase(mTimestamps.begin());
                        }
                        mBuffer->setRange(0,0);
						mTimestamps.clear();
                        return NULL;
                   }
                }while (code1 != DTSSYNCWORD);
                sp<MetaData> meta = new MetaData;
                meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_AUDIO_DTS);
                meta->setInt32(kKeySampleRate, kSamplingFreq);
                meta->setInt32(kKeyChannelCount, channel_configuration);
                mFormat = meta;
                mSource = new AnotherPacketSource(meta);
                return NULL;
            }
        } while (code != AC3SYNCWORD);
        bits.skipBits(16);
        uint8_t fscod;
        fscod = bits.getBits(8);
        if ((fscod >> 6) == 3) {
            return NULL;
        }
        kSamplingFreq= TS_AC3SampleRateTable[fscod >> 6];
        sp<MetaData> meta = new MetaData;
        meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_AUDIO_AC3);
        meta->setInt32(kKeySampleRate, kSamplingFreq);
        meta->setInt32(kKeyChannelCount, channel_configuration);
        mFormat = meta;
        mSource = new AnotherPacketSource(meta);
    }
    MediaBuffer  *accessUnit = new MediaBuffer(auSize);
    memcpy(accessUnit->data(),(uint8_t *)mBuffer->data(),auSize);
#ifdef ES_DEBUG
#endif
    int64_t timeUs = 0;
    if(mTimestamps.size() > 0)
    {
        timeUs = *mTimestamps.begin();
        lastTimeus = timeUs;
        mTimestamps.erase(mTimestamps.begin());
        }
    else
    {
        timeUs = lastTimeus;
    }
    mBuffer->setRange(0,0);
    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
    mSource->queueAccessUnit(accessUnit);
    return NULL;
}
MediaBuffer * ElementaryStreamQueue::dequeueAccessUnitVC1() {
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    size_t auSize = 0;
    size_t offset = Nextsize;
    bool skipFlag = false;
    uint8_t *tempdata = mBuffer->data();
    if(seekFlag)
    {
        for (;;) {
             while (offset < size && data[offset] != 0x01) {
                ++offset;
            }
            if (offset == size) {
                mBuffer->setRange(0,0);
                int64_t timeUs = fetchTimestamp(size);
                ALOGV("seek skip current pes ");
                return NULL;
            }
            if(data[offset - 1] == 0x00 && data[offset - 2] == 0x00&&data[offset+1] == 0x0D) {
                ALOGV("seek SC_FRAME found \n");
                size_t tempoffset = offset + 2;
                bool IPicTypeFlag = false;
                if(mBuffer->size() - tempoffset > 2)
                {
                    ABitReader bits(mBuffer->data() + tempoffset, mBuffer->size() - tempoffset);
                    int filed_interlace = 0;
                    if(Vc1InterlaceFlag)
                    {
                        if(bits.getBits(1))
                        {
                            filed_interlace = bits.getBits(1);
                        }
                    }
                    if(filed_interlace)
                    {
                        int fieldPicType = bits.getBits(3);
                        if(fieldPicType == 0|| fieldPicType == 1)
                        {
                            IPicTypeFlag = true;
                        }
                    }
                    else
                    {
                        int tmp = bits.getBits(3);
                        if (tmp == 6) {   /* 110b */
                            IPicTypeFlag = true;
                        }
                    }
                    if(IPicTypeFlag)
                    {
                        offset -= 2;
                        memmove(mBuffer->data(), mBuffer->data() + offset,
                        mBuffer->size() - offset);
                        mBuffer->setRange(0, mBuffer->size() - offset);
                        seekFlag = false;
                        int64_t timeUs = fetchTimestamp(offset);
                        return NULL;
                    }
                }
            }
            ++offset;
       }
     }
repe:
    for (;;) {
        while (offset < (size -1) && data[offset] != 0x01) {
            ++offset;
        }
        if(offset < 2)
        {
            ++offset;
            continue;
        }
        if (offset == (size - 1)) {
            if(startoffset >= 0)
            {
                if(startoffset > 0)
                {
                    uint8_t *buf = (uint8_t *)malloc(mBuffer->size() - startoffset);
                    if(buf)
                    {
                         memcpy(buf, mBuffer->data() + startoffset,mBuffer->size() - startoffset);
                         memcpy(mBuffer->data(),buf,mBuffer->size() - startoffset);
                         free(buf);
                    }
                    mBuffer->setRange(0, mBuffer->size() - startoffset);
                    Nextsize += offset - startoffset - 2;
                    startoffset = 0;
                }
                else
                {
                    Nextsize += offset - 2;
                }
            }
            return NULL;
        }
        if (data[offset - 1] == 0x00 && data[offset - 2] == 0x00 && data[offset+1] == 0x0F) {
            ALOGV("SC_SEQ found");
            if (mFormat == NULL) {
                size_t tempoffset = offset + Nextsize + 2;
                ABitReader bits(mBuffer->data() + tempoffset, mBuffer->size() - tempoffset);
                sp<MetaData> meta = new MetaData;
                bits.skipBits(16);
                uint32_t width = bits.getBits(12)*2 + 2;
                uint32_t height = bits.getBits(12)*2 + 2;
                bits.skipBits(1);
                Vc1InterlaceFlag = bits.getBits(1);
                ALOGD("width = %d,height = %d \n",width,height);
                meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_VIDEO_VC1);
                meta->setInt32(kKeyWidth, width);
                meta->setInt32(kKeyHeight, height);
                mFormat = meta;
                mSource = new AnotherPacketSource(meta);
                if (pktStart != 0)
                {
                    skipFlag = true;
                    offset -= 2;
                    break;
                }
            }
            if (pktStart == 0) {
                startoffset = offset - 2;
			}
            else
            {
                offset -= 2;
                break;
            }
        }
        else if(data[offset] == 0x01 && data[offset - 1] == 0x00 && data[offset - 2] == 0x00&&data[offset+1] == 0x0D) {
            ALOGV("SC_FRAME");
            if (pktStart == 0) {
					pktStart = 1;
					if (startoffset == -1) {
						startoffset = offset - 2;
					}
			}
            else
            {
                offset -= 2;
                break;
            }
        }
        ++offset;
    }
    auSize = offset - startoffset;
    Nextsize  = 0;
    startoffset = -1;
    pktStart = 0;
    int64_t timeUs = 0;
    if((mFormat == NULL) || skipFlag ||!auSize)
    {
        uint8_t *buf = (uint8_t *)malloc(mBuffer->size() - offset);
        if(buf)
        {
             memcpy(buf, mBuffer->data() + offset,mBuffer->size() - offset);
             memcpy(mBuffer->data(),buf,mBuffer->size() - offset);
             free(buf);
        }
        ALOGV("skip the data before SEQUENCE_HEADER_CODE found \n");
        mBuffer->setRange(0, mBuffer->size() - offset);
        fetchTimestamp(offset);
        return NULL;
    }
    timeUs = fetchTimestamp(auSize);
    ALOGV("timeUs input = %lld",timeUs);
    MediaBuffer *accessUnit = new MediaBuffer(auSize);
    memcpy(accessUnit->data(),(uint8_t*)tempdata,auSize);
    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
    mSource->queueAccessUnit(accessUnit);
#ifdef ES_DEBUG
  // fwrite(accessUnit->data(),1,auSize,fp1);
 //  fflush(fp1);
#endif
    tempdata += auSize;
    goto repe;
    return NULL;
}

 MediaBuffer * ElementaryStreamQueue::dequeueAccessUnitDTS() {
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    size_t auSize = size;
    if(!size)
    {
        return NULL;
    }
    int32_t kSamplingFreq = 0;
    int32_t channel_configuration = 0;
    if (mFormat == NULL) {
        sp<MetaData> meta = new MetaData;
        meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_AUDIO_DTS);
        meta->setInt32(kKeySampleRate, kSamplingFreq);
        meta->setInt32(kKeyChannelCount, channel_configuration);
        mFormat = meta;
    }
#ifdef ES_DEBUG
#endif
    MediaBuffer * accessUnit = new MediaBuffer(auSize);
    memcpy(accessUnit->data(),(uint8_t *)mBuffer->data(),auSize);
    int64_t timeUs = 0;
    if(mTimestamps.size() > 0)
    {
        timeUs = *mTimestamps.begin();
        lastTimeus = timeUs;
        mTimestamps.erase(mTimestamps.begin());
    }
    else
    {
        timeUs = lastTimeus;
    }
    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
    mBuffer->setRange(0,0);
    return accessUnit;
}

inline void mpg_swap16(uint16_t *pcode) {*pcode = (*pcode << 8) | (*pcode >> 8);}
void alternateCode(uint16_t *buf, int bufSize)
{
    int i;
    for (i = 0; i < bufSize / 2; i++) {
        mpg_swap16(&buf[i]);
    }
}
static const uint8_t bits_per_samples[4] = { 0, 16, 20, 24 };
static const uint32_t channel_layouts[16] = {
  0, AV_CH_LAYOUT_MONO, 0, AV_CH_LAYOUT_STEREO, AV_CH_LAYOUT_SURROUND,
  AV_CH_LAYOUT_2_1, AV_CH_LAYOUT_4POINT0, AV_CH_LAYOUT_2_2, AV_CH_LAYOUT_5POINT0,
  AV_CH_LAYOUT_5POINT1, AV_CH_LAYOUT_7POINT0, AV_CH_LAYOUT_7POINT1, 0, 0, 0, 0
};
static const uint8_t channels[16] = {
  0, 1, 0, 2, 3, 3, 4, 4, 5, 6, 7, 8, 0, 0, 0, 0
};

#if 0
MediaBuffer * ElementaryStreamQueue::dequeueAccessUnitPCM() {
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    size_t auSize = size;
    if(!size)
    {
        return NULL;
    }
    int32_t kSamplingFreq = 0;
    int32_t channel_configuration = 0;
    int32_t bits_per_sample = 0;
    if (mFormat == NULL) {
        uint8_t* header = (uint8_t *)mBuffer->data();
        uint8_t channel_layout = header[2] >> 4;
        int32_t sample_rate = 0;
        ALOGV("pcm_bluray_parse_header: header = %02x%02x%02x%02x\n",
                header[0], header[1], header[2], header[3]);
         int32_t bits_per_coded_sample = bits_per_samples[header[3] >> 6];
         if (!(bits_per_coded_sample == 16 || bits_per_coded_sample == 24)) {
            ALOGE("unsupported sample depth (%d)\n", bits_per_coded_sample);
            return NULL;
         }
        switch (header[2] & 0x0f) {
            case 1:
                sample_rate = 48000;
                break;
            case 4:
                sample_rate = 96000;
                break;
            case 5:
                sample_rate = 192000;
                break;
            default:
            sample_rate = 0;
            ALOGE("unsupported sample rate (%d)\n",
                   header[2] & 0x0f);
            return NULL;
        }
        int32_t channelcount = channels[channel_layout];
        if (!channelcount) {
            ALOGE("unsupported channel configuration (%d)\n",channelcount);
            return NULL;
        }
        int32_t bit_rate = channelcount * sample_rate *bits_per_coded_sample;
        sp<MetaData> meta = new MetaData;
        meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_AUDIO_WAV);
        meta->setInt32(kKeySampleRate, sample_rate);
        meta->setInt32(kKeyBitRate, bits_per_coded_sample);
        meta->setInt32(kKeyChannelCount, channelcount);
        mFormat = meta;
    }
#ifdef ES_DEBUG
#endif
    MediaBuffer * accessUnit = new MediaBuffer(auSize);
    alternateCode((uint16_t *)mBuffer->data(),auSize);
    memcpy(accessUnit->data(),(uint8_t *)mBuffer->data(),auSize);
    int64_t timeUs = 0;
    if(mTimestamps.size() > 0)
    {
        timeUs = *mTimestamps.begin();
        lastTimeus = timeUs;
        mTimestamps.erase(mTimestamps.begin());
    }
    else
    {
        timeUs = lastTimeus;
    }
    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
    mBuffer->setRange(0,0);
    return accessUnit;
}
#endif

MediaBuffer *ElementaryStreamQueue::dequeueAccessUnitPCMAudio() {
#if 1
	int loop_time = 0;
    unsigned numAUs;
	unsigned quantization_word_length ;
    unsigned audio_sampling_frequency ;
    unsigned num_channels ;
	do
	{
		if (mBuffer->size() < loop_time * 4 + 4 ) {

			if(loop_time > 0)
			{
				memmove(
		            mBuffer->data(),
		            mBuffer->data() + loop_time * 4,
		            mBuffer->size() - loop_time * 4);
		    	mBuffer->setRange(0, mBuffer->size() - loop_time * 4);

			}
        	return NULL;
    	}
		ABitReader bits(mBuffer->data() + loop_time * 4, 4);
	        char pcm_sign = bits.getBits(8);
	    numAUs = bits.getBits(8);
	    bits.skipBits(8);
	    quantization_word_length = bits.getBits(2);
	    audio_sampling_frequency = bits.getBits(3);
	    num_channels = bits.getBits(3);

		static const size_t kFramesPerAU = 80;
		size_t frameSize = 2 /* numChannels */ * sizeof(int16_t);
		#if 0
		CHECK_EQ(pcm_sign, 0xa0);


		CHECK_EQ(audio_sampling_frequency, 2);	// 48kHz
		CHECK_EQ(num_channels, 1u);  // stereo!
		#endif
		if(pcm_sign!=0xa0 || audio_sampling_frequency ==3 || num_channels != 1)
		{

		   loop_time++;
                   ALOGE("malformed PCM Audio AU, loop_time = %d", loop_time);
		   continue;
		}
		else
		{
			if(loop_time!=0)
			{
				memmove(
		            mBuffer->data(),
		            mBuffer->data() + loop_time * 4,
		            mBuffer->size() - loop_time * 4);
	    		mBuffer->setRange(0, mBuffer->size() - loop_time * 4);
			}
			break;
		}
	}while(1);

    if (mFormat == NULL) {
        mFormat = new MetaData;
        mFormat->setCString(kKeyMIMEType, MEDIA_MIMETYPE_AUDIO_RAW);
        mFormat->setInt32(kKeyChannelCount, 2);
		if(audio_sampling_frequency == 2)
        mFormat->setInt32(kKeySampleRate, 48000);//8000);
        else
			mFormat->setInt32(kKeySampleRate, 44100);//8000);
    }
    static const size_t kFramesPerAU = 80;
    size_t frameSize = 2 /* numChannels */ * sizeof(int16_t);

    size_t payloadSize = numAUs * frameSize * kFramesPerAU;

    if (mBuffer->size() < 4 + payloadSize) {
        return NULL;
    }

    MediaBuffer * accessUnit = new MediaBuffer(payloadSize);
    memcpy(accessUnit->data(), mBuffer->data() + 4, payloadSize);

    int64_t timeUs = fetchTimestamp(payloadSize + 4);
    CHECK_GE(timeUs, 0ll);
    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);

    int16_t *ptr = (int16_t *)accessUnit->data();
    for (size_t i = 0; i < payloadSize / sizeof(int16_t); ++i) {
        ptr[i] = ntohs(ptr[i]);
    }

    memmove(
            mBuffer->data(),
            mBuffer->data() + 4 + payloadSize,
            mBuffer->size() - 4 - payloadSize);

    mBuffer->setRange(0, mBuffer->size() - 4 - payloadSize);

    return accessUnit;

#else
    if (mBuffer->size() < 4) {
        return NULL;
    }

    ABitReader bits(mBuffer->data(), 4);
    CHECK_EQ(bits.getBits(8), 0xa0);
    unsigned numAUs = bits.getBits(8);
    bits.skipBits(8);
    unsigned quantization_word_length = bits.getBits(2);
    unsigned audio_sampling_frequency = bits.getBits(3);
    unsigned num_channels = bits.getBits(3);

    CHECK_EQ(audio_sampling_frequency, 2);  // 48kHz
    CHECK_EQ(num_channels, 1u);  // stereo!
    if (mFormat == NULL) {
        mFormat = new MetaData;
        mFormat->setCString(kKeyMIMEType, MEDIA_MIMETYPE_AUDIO_RAW);
        mFormat->setInt32(kKeyChannelCount, 2);
        mFormat->setInt32(kKeySampleRate, 48000);
    }

    static const size_t kFramesPerAU = 80;
    size_t frameSize = 2 /* numChannels */ * sizeof(int16_t);

    size_t payloadSize = numAUs * frameSize * kFramesPerAU;

    if (mBuffer->size() < 4 + payloadSize) {
        return NULL;
    }

    MediaBuffer * accessUnit = new MediaBuffer(payloadSize);
    memcpy(accessUnit->data(), mBuffer->data() + 4, payloadSize);

    int64_t timeUs = fetchTimestamp(payloadSize + 4);
    CHECK_GE(timeUs, 0ll);
    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);

    int16_t *ptr = (int16_t *)accessUnit->data();
    for (size_t i = 0; i < payloadSize / sizeof(int16_t); ++i) {
        ptr[i] = ntohs(ptr[i]);
    }

    memmove(
            mBuffer->data(),
            mBuffer->data() + 4 + payloadSize,
            mBuffer->size() - 4 - payloadSize);

    mBuffer->setRange(0, mBuffer->size() - 4 - payloadSize);

    return accessUnit;
#endif
}
MediaBuffer * ElementaryStreamQueue::dequeueAccessUnitMP3() {
    const uint8_t *data = mBuffer->data();
    size_t size = mBuffer->size();
    size_t auSize = size;
	size_t offset = 0;
    if(size < 2048)
    {

        if(!size)
            {
            return NULL;
        }
        if(mTimestamps.size()> 1)
        {
            int64_t timeUs = *mTimestamps.begin();
            mTimestamps.erase(mTimestamps.begin());
            lastTimeus = timeUs;
        }
        return NULL;
                }
    if (mFormat == NULL) {
        int32_t kSamplingFreq = 0;
        int32_t channel_configuration = 1;
        uint16_t code = 0;
        uint8_t samplerate;
        int version;
        int layer;
        int error_protection;
        int bitsrate;
        int chnmode;
        while (offset + 8 <= mBuffer->size()) {
	        ABitReader bits(mBuffer->data() + offset, mBuffer->size() - offset);
	        code = 0;
	        do {
	            code = (code << 8) | bits.getBits(8);
				offset+=1;
            ALOGV("code11 = 0x%x \n",code);
        } while ((code&0xFFE0) != MPASYNCWORD);
        ALOGV("code = 0x%x \n",code);
        version = (code >> 3) & 0x03;
        layer = (code >> 1) & 0x03;
        error_protection = code & 1;
        bitsrate = bits.getBits(4);
        samplerate = bits.getBits(2);
        bits.skipBits(2); // padding bit, private bit
        chnmode = bits.getBits(2);
        bits.skipBits(6); // mode extension 2 bits, copyright bit, original bit, emphasis 2 bits.
			offset+=2;
	        switch(chnmode){
        	case 0:			//stereo, just set to 2
        		channel_configuration =2;
    		break;

        	case 1:			//joint stereo, just set to 2
        		channel_configuration =2;
        		break;

        	case 2:
        		channel_configuration =2;
        		break;

        	case 3:
        		channel_configuration = 1;
        		break;

    	    default:
        		channel_configuration = 2;
    		break;
        }
        kSamplingFreq = TS_MPASampleRateTable[version][samplerate];
        ALOGV("version::%d,layer::%d samplerate:%d",version,layer,samplerate);
        ALOGV("channel_configuration %d,kSamplingFreq %d",channel_configuration,kSamplingFreq);
			if(kSamplingFreq == 0)
			{
				continue;
			}
			else
			{
        sp<MetaData> meta = new MetaData;
        meta->setCString(kKeyMIMEType, MEDIA_MIMETYPE_AUDIO_MPEG);
        meta->setInt32(kKeySampleRate, kSamplingFreq);
        meta->setInt32(kKeyChannelCount, channel_configuration);
        mFormat = meta;
		mFormat_flag = 1;
				break;
            }
	       }
          }
		  if(mFormat==NULL)
   		  {
   		  	memmove(mBuffer->data(), mBuffer->data() + offset,mBuffer->size() - offset);
    		mBuffer->setRange(0, mBuffer->size() - offset);
			return NULL;
   		  }
#ifdef ES_DEBUG
#endif
	MediaBuffer *accessUnit;
	if(mFormat_flag == false)
	{
	    accessUnit = new MediaBuffer(mBuffer->size() - offset + 4);
	    memcpy(accessUnit->data(),mBuffer->data() + offset - 4,mBuffer->size() - offset + 4);
	}
	else
	{
		accessUnit = new MediaBuffer(mBuffer->size());
	    memcpy(accessUnit->data(),mBuffer->data() ,mBuffer->size() );
	}
    int64_t timeUs = 0;
    if(mTimestamps.size() > 0)
            {
        timeUs = *mTimestamps.begin();
        lastTimeus = timeUs;
        mTimestamps.erase(mTimestamps.begin());
            }
    else
            {
        timeUs = lastTimeus;
    }
    mBuffer->setRange(0,0);
    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
    return accessUnit;
            }

#if 0
MediaBuffer *ElementaryStreamQueue::dequeueAccessUnitAAC_ADTS() {
     Vector<size_t> frameOffsets;
     Vector<size_t> frameSizes;
     size_t auSize = 0;
     if(!mBuffer->size())
     {
        return NULL;
     }
    size_t offset = 0;
    uint8_t *data = mBuffer->data();
    bool hasframe = false;
    while (offset + 7 <= mBuffer->size()) {
        ABitReader bits(mBuffer->data() + offset, mBuffer->size() - offset);
 repet:
        while((bits.showBits(16)&0xfff6) != 0xfff0u)
        {
            bits.skipBits(8);  // ID, layer
            offset++;
            if(bits.numBitsLeft() < 56)
            {
                if(frameOffsets.size())
                {
                    hasframe = true;
            break;
        }
                else
                {
        mBuffer->setRange(0,0);
        return NULL;
    }
				 if((offset + 7) > mBuffer->size())
				 {
                     mBuffer->setRange(0,0);
                     return NULL;
				 }
            }
         // adts_fixed_header
        }
        if(hasframe)
        {
            break;
         }
        bits.skipBits(15);  // ID, layer
         bool protection_absent = bits.getBits(1) != 0;

         if (mFormat == NULL) {
             unsigned profile = bits.getBits(2);
            if(profile == 3)
            {
                bits.skipBits(14);  // original_copy, home
                offset += 4;
                goto repet;
            }
             unsigned sampling_freq_index = bits.getBits(4);
             bits.getBits(1);  // private_bit
             unsigned channel_configuration = bits.getBits(3);
            if(channel_configuration == 0)
            {
                bits.skipBits(6);  // original_copy, home
                offset += 4;
                goto repet;
            }
             bits.skipBits(2);  // original_copy, home

             mFormat = MakeAACCodecSpecificData(
                     profile, sampling_freq_index, channel_configuration);
            if(mFormat != NULL)
            {
				if(player_type == 4/*live_tv*/)
					mFormat->setInt32(kKeyIsLATM, true);

                mSource = new AnotherPacketSource(mFormat);
            }
         } else {
             // profile_ObjectType, sampling_frequency_index, private_bits,
             // channel_configuration, original_copy, home
             bits.skipBits(12);
         }

         // adts_variable_header

         // copyright_identification_bit, copyright_identification_start
         bits.skipBits(2);

         unsigned aac_frame_length = bits.getBits(13);

         bits.skipBits(11);  // adts_buffer_fullness

         unsigned number_of_raw_data_blocks_in_frame = bits.getBits(2);

         if (number_of_raw_data_blocks_in_frame != 0) {
             // To be implemented.
		ALOGD("number_of_raw_data_blocks_in_frame % d != 0 error offset %d +7 <= mBuffer->size() %d",
			number_of_raw_data_blocks_in_frame,offset, mBuffer->size());
		offset+=4;
		continue;
             TRESPASS();
         }

         if (offset + aac_frame_length > mBuffer->size()) {
             break;
         }

         size_t headerSize = protection_absent ? 7 : 9;

         frameOffsets.push(offset + headerSize);
         frameSizes.push(aac_frame_length - headerSize);
         auSize += aac_frame_length - headerSize;

         offset += aac_frame_length;
		 if(aac_frame_length <= 0)
		 {
		     mBuffer->setRange(0,0);
             return NULL;
		 }
     }

     if (offset == 0) {
         return NULL;
     }

    MediaBuffer *accessUnit = new MediaBuffer(auSize);
    size_t dstOffset = 0;
    for (size_t i = 0; i < frameOffsets.size(); ++i) {
        memcpy(accessUnit->data() + dstOffset,
               mBuffer->data() + frameOffsets.itemAt(i),
               frameSizes.itemAt(i));

        dstOffset += frameSizes.itemAt(i);
    }

    memmove(mBuffer->data(), mBuffer->data() + offset,
            mBuffer->size() - offset);
    if(hasframe)
        mBuffer->setRange(0,0);
    else
    mBuffer->setRange(0, mBuffer->size() - offset);

    int64_t timeUs = 0;
    if(mTimestamps.size() > 0)
    {
        timeUs = *mTimestamps.begin();
        lastTimeus = timeUs;
    mTimestamps.erase(mTimestamps.begin());
		if(hasframe){
            mTimestamps.clear();
        }
    }
    else
    {
        timeUs = lastTimeus;
    }


        accessUnit->meta_data()->setInt64(kKeyTime, timeUs);
     return accessUnit;
 }
#else
MediaBuffer *ElementaryStreamQueue::dequeueAccessUnitAAC_ADTS() {
    if (mBuffer->size() == 0) {
        return NULL;
    }

    CHECK(!mRangeInfos.empty());

    const RangeInfo &info = *mRangeInfos.begin();
    if (mBuffer->size() < info.mLength) {
        return NULL;
    }

    CHECK_GE(info.mTimestampUs, 0ll);

    // The idea here is consume all AAC frames starting at offsets before
    // info.mLength so we can assign a meaningful timestamp without
    // having to interpolate.
    // The final AAC frame may well extend into the next RangeInfo but
    // that's ok.
    // TODO: the logic commented above is skipped because codec cannot take
    // arbitrary sized input buffers;
    size_t offset = 0;
    while (offset < info.mLength) {
        if (offset + 7 > mBuffer->size()) {
            return NULL;
        }

        ABitReader bits(mBuffer->data() + offset, mBuffer->size() - offset);

        // adts_fixed_header

        CHECK_EQ(bits.getBits(12), 0xfffu);
        bits.skipBits(3);  // ID, layer
        bool protection_absent = bits.getBits(1) != 0;

        if (mFormat == NULL) {
            unsigned profile = bits.getBits(2);
            CHECK_NE(profile, 3u);
            unsigned sampling_freq_index = bits.getBits(4);
            bits.getBits(1);  // private_bit
            unsigned channel_configuration = bits.getBits(3);
            CHECK_NE(channel_configuration, 0u);
            bits.skipBits(2);  // original_copy, home

            mFormat = MakeAACCodecSpecificData(
                    profile, sampling_freq_index, channel_configuration);

            mFormat->setInt32(kKeyIsADTS, true);

            int32_t sampleRate;
            int32_t numChannels;
            CHECK(mFormat->findInt32(kKeySampleRate, &sampleRate));
            CHECK(mFormat->findInt32(kKeyChannelCount, &numChannels));

            ALOGI("found AAC codec config (%d Hz, %d channels)",
                 sampleRate, numChannels);
        } else {
            // profile_ObjectType, sampling_frequency_index, private_bits,
            // channel_configuration, original_copy, home
            bits.skipBits(12);
        }

        // adts_variable_header

        // copyright_identification_bit, copyright_identification_start
        bits.skipBits(2);

        unsigned aac_frame_length = bits.getBits(13);

        bits.skipBits(11);  // adts_buffer_fullness

        unsigned number_of_raw_data_blocks_in_frame = bits.getBits(2);

        if (number_of_raw_data_blocks_in_frame != 0) {
            // To be implemented.
            TRESPASS();
        }

        if (offset + aac_frame_length > mBuffer->size()) {
            return NULL;
        }

        size_t headerSize = protection_absent ? 7 : 9;

        offset += aac_frame_length;
        // TODO: move back to concatenation when codec can support arbitrary input buffers.
        // For now only queue a single buffer
        break;
    }

    int64_t timeUs = fetchTimestampAAC(offset);

    MediaBuffer *accessUnit = new MediaBuffer(offset);
    memcpy(accessUnit->data(), mBuffer->data(), offset);

    memmove(mBuffer->data(), mBuffer->data() + offset,
            mBuffer->size() - offset);
    mBuffer->setRange(0, mBuffer->size() - offset);

    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);

    return accessUnit;
}

#endif

//TODO: avoid interpolating timestamps once codec supports arbitrary sized input buffers
int64_t ElementaryStreamQueue::fetchTimestampAAC(size_t size) {
     int64_t timeUs = -1;
     bool first = true;

     size_t samplesize = size;
     while (size > 0) {
         CHECK(!mRangeInfos.empty());

         RangeInfo *info = &*mRangeInfos.begin();

         if (first) {
             timeUs = info->mTimestampUs;
             first = false;
         }

         if (info->mLength > size) {
             int32_t sampleRate;
             CHECK(mFormat->findInt32(kKeySampleRate, &sampleRate));
             info->mLength -= size;
             size_t numSamples = 1024 * size / samplesize;
             info->mTimestampUs += numSamples * 1000000ll / sampleRate;
             size = 0;
         } else {
             size -= info->mLength;

             mRangeInfos.erase(mRangeInfos.begin());
             info = NULL;
         }

     }

     if (timeUs == 0ll) {
         ALOGV("Returning 0 timestamp");
     }

     return timeUs;
 }


 /************************************************************

 *the follow code is ported by charles chen at jun 28th 2013 for parse the chan_config = 0
 *there are still have some problem ,must fixed it

 *************************************************************/
#define FF_ARRAY_ELEMS(a) (sizeof(a) / sizeof((a)[0]))
#define FFMIN(a, b) ((a) > (b) ? (b) : (a))
#define MKBETAG(a,b,c,d) ((d) | ((c) << 8) | ((b) << 16) | ((unsigned)(a) << 24))

  typedef enum LatmAacErrorCode{
	 STREAM_ERROR = -1,
	 STREAM_OK	  =  0,
	 STREAM_NOT_ENOUGH =1,
 }LATMERROR;


 const int avpriv_mpeg4audio_sample_rates[16] = {
	 96000, 88200, 64000, 48000, 44100, 32000,
	 24000, 22050, 16000, 12000, 11025, 8000, 7350
 };

 const uint8_t ff_mpeg4audio_channels[8] = {
	 0, 1, 2, 3, 4, 5, 6, 8
 };

 // the follow defined for the pce parse
 enum RawDataBlockType {
	 TYPE_SCE,
	 TYPE_CPE,
	 TYPE_CCE,
	 TYPE_LFE,
	 TYPE_DSE,
	 TYPE_PCE,
	 TYPE_FIL,
	 TYPE_END,
 };

 enum ChannelPosition {
	 AAC_CHANNEL_OFF   = 0,
	 AAC_CHANNEL_FRONT = 1,
	 AAC_CHANNEL_SIDE  = 2,
	 AAC_CHANNEL_BACK  = 3,
	 AAC_CHANNEL_LFE   = 4,
	 AAC_CHANNEL_CC    = 5,
 };

 static const int8_t tags_per_config[16] = { 0, 1, 1, 2, 3, 3, 4, 5, 0, 0, 0, 0, 0, 0, 0, 0 };

 static const uint8_t aac_channel_layout_map[7][5][3] = {
	 { { TYPE_SCE, 0, AAC_CHANNEL_FRONT }, },
	 { { TYPE_CPE, 0, AAC_CHANNEL_FRONT }, },
	 { { TYPE_SCE, 0, AAC_CHANNEL_FRONT }, { TYPE_CPE, 0, AAC_CHANNEL_FRONT }, },
	 { { TYPE_SCE, 0, AAC_CHANNEL_FRONT }, { TYPE_CPE, 0, AAC_CHANNEL_FRONT }, { TYPE_SCE, 1, AAC_CHANNEL_BACK }, },
	 { { TYPE_SCE, 0, AAC_CHANNEL_FRONT }, { TYPE_CPE, 0, AAC_CHANNEL_FRONT }, { TYPE_CPE, 1, AAC_CHANNEL_BACK }, },
	 { { TYPE_SCE, 0, AAC_CHANNEL_FRONT }, { TYPE_CPE, 0, AAC_CHANNEL_FRONT }, { TYPE_CPE, 1, AAC_CHANNEL_BACK }, { TYPE_LFE, 0, AAC_CHANNEL_LFE  }, },
	 { { TYPE_SCE, 0, AAC_CHANNEL_FRONT }, { TYPE_CPE, 0, AAC_CHANNEL_FRONT }, { TYPE_CPE, 1, AAC_CHANNEL_FRONT }, { TYPE_CPE, 2, AAC_CHANNEL_BACK }, { TYPE_LFE, 0, AAC_CHANNEL_LFE  }, },
 };


 /**
  * Parse MPEG-4 audio configuration for ALS object type.
  * @param[in] gb		bit reader context
  * @param[in] c		MPEG4AudioConfig structure to fill
  * @return on success 0 is returned, otherwise a value < 0
  */
 static int parse_config_ALS(bitstream_t *gb, MPEG4AudioConfig *c)
 {
	 if (get_bits_left(gb) < 112)
		 return -1;

	 if (read_bits(gb, 32) != MKBETAG('A','L','S','\0'))
		 return -1;

	 // override AudioSpecificConfig channel configuration and sample rate
	 // which are buggy in old ALS conformance files
	 c->sample_rate = read_bits(gb, 32);

	 // skip number of samples
	 skip_bits(gb, 32);

	 // read number of channels
	 c->chan_config = 0;
	 c->channels	= read_bits(gb, 16) + 1;

	 return 0;
 }



 static inline int get_object_type(bitstream_t *gb)
 {
	 int object_type = read_bits(gb, 5);
	 if (object_type == AOT_ESCAPE)
		 object_type = 32 + read_bits(gb, 6);
	 return object_type;
 }

 static inline int get_sample_rate(bitstream_t *gb, int *index)
 {
	 *index = read_bits(gb, 4);
	 return *index == 0x0f ? read_bits(gb, 24) :
		 avpriv_mpeg4audio_sample_rates[*index];
 }



 int avpriv_mpeg4audio_get_config(MPEG4AudioConfig *c, const uint8_t *buf,
								  int bit_size, int sync_extension)
 {
	 bitstream_t gb;
	 int specific_config_bitindex;

	 if(bit_size<=0)
		 return -1;

	 init_bits(&gb, (uint8_t*)buf, bit_size);
	 c->object_type = get_object_type(&gb);
	 c->sample_rate = get_sample_rate(&gb, &c->sampling_index);
	 c->chan_config = read_bits(&gb, 4);
	  if (c->chan_config < FF_ARRAY_ELEMS(ff_mpeg4audio_channels))
		 c->channels = ff_mpeg4audio_channels[c->chan_config];
	 c->sbr = -1;
	 c->ps	= -1;
	 ALOGV("object_type %d	sample_rate %d ch %d\n",  c->object_type,c->sample_rate, c->channels);

	 if (c->object_type == AOT_SBR || (c->object_type == AOT_PS &&
		 // check for W6132 Annex YYYY draft MP3onMP4
		 !(show_bits(&gb, 3) & 0x03 && !(show_bits(&gb, 9) & 0x3F)))) {
		 if (c->object_type == AOT_PS)
			 c->ps = 1;
		 c->ext_object_type = AOT_SBR;
		 c->sbr = 1;
		 c->ext_sample_rate = get_sample_rate(&gb, &c->ext_sampling_index);
		 c->object_type = get_object_type(&gb);
		 if (c->object_type == AOT_ER_BSAC)
			 c->ext_chan_config = read_bits(&gb, 4);
		// ALOGI("ext object_type %d ext sample_rate %d ext ch %d\n",  c->ext_object_type,c->ext_sample_rate, c->ext_chan_config);
	 } else {
		 c->ext_object_type = AOT_NULL;
		 c->ext_sample_rate = 0;
	 }
	 specific_config_bitindex = get_bits_count(&gb);

	 if (c->object_type == AOT_ALS) {
		 skip_bits(&gb, 5);
		 if (show_bits(&gb, 24) != MKBETAG('\0','A','L','S'))
			 skip_bits(&gb, 24);

		 specific_config_bitindex = get_bits_count(&gb);

		 if (parse_config_ALS(&gb, c))
			 return -1;
	 }

	 if (c->ext_object_type != AOT_SBR && sync_extension) {
		 while (get_bits_left(&gb) > 15) {
			 if (show_bits(&gb, 11) == 0x2b7) { // sync extension
				 read_bits(&gb, 11);
				 c->ext_object_type = get_object_type(&gb);
				 if (c->ext_object_type == AOT_SBR && (c->sbr = read_bits1(&gb)) == 1) {
					 c->ext_sample_rate = get_sample_rate(&gb, &c->ext_sampling_index);
					 if (c->ext_sample_rate == c->sample_rate)
						 c->sbr = -1;
				 }
				 if (get_bits_left(&gb) > 11 && read_bits(&gb, 11) == 0x548)
					 c->ps = read_bits1(&gb);
				 break;
			 } else
				 read_bits1(&gb); // skip 1 bit
		 }
	 }

	 //PS requires SBR
	 if (!c->sbr)
		 c->ps = 0;
	 //Limit implicit PS to the HE-AACv2 Profile
	 if ((c->ps == -1 && c->object_type != AOT_AAC_LC) || c->channels & ~0x01)
		 c->ps = 0;

	 return specific_config_bitindex;
 }


 static uint8_t latm_get_value(bitstream_t *bs)
 {
	 int length = read_bits(bs, 2);
	 return read_bits(bs, (length+1) * 8);
 }


 /**
  * Set up channel positions based on a default channel configuration
  * as specified in table 1.17.
  *
  * @return  Returns error status. 0 - OK, !0 - error
  */
 static int set_default_channel_config(  uint8_t (*layout_map)[3],
											   int *tags,
											   int channel_config)
 {
	 if (channel_config < 1 || channel_config > 7) {
		 ALOGE( "invalid default channel configuration (%d)\n",
				channel_config);
		 return -1;
	 }
	 *tags = tags_per_config[channel_config];
	 memcpy(layout_map, aac_channel_layout_map[channel_config-1], *tags * sizeof(*layout_map));
	 return 0;
 }


 /**
  * Decode an array of 4 bit element IDs, optionally interleaved with a stereo/mono switching bit.
  *
  * @param type speaker type/position for these channels
  */
 static void decode_channel_map(uint8_t layout_map[][3],
								enum ChannelPosition type,
								bitstream_t*gb, int n)
 {
	 while (n--) {
		 enum RawDataBlockType syn_ele;
		 switch (type) {
		 case AAC_CHANNEL_FRONT:
		 case AAC_CHANNEL_BACK:
		 case AAC_CHANNEL_SIDE:
			 syn_ele = (enum RawDataBlockType)read_bits1(gb);
			 break;
		 case AAC_CHANNEL_CC:
			 skip_bits(gb,1);
			 syn_ele = TYPE_CCE;
			 break;
		 case AAC_CHANNEL_LFE:
			 syn_ele = TYPE_LFE;
			 break;
		 default:
			 ;
			// av_assert0(0);
		 }
		 layout_map[0][0] = syn_ele;
		 layout_map[0][1] = read_bits(gb, 4);
		 layout_map[0][2] = type;
		 layout_map++;
	 }
 }

 /**
  * Decode program configuration element; reference: table 4.2.
  *
  * @return  Returns error status. 0 - OK, !0 - error
  */
 static int decode_pce(MPEG4AudioConfig *m4ac,
					   uint8_t (*layout_map)[3],
					   bitstream_t*gb)
 {
	 int num_front, num_side, num_back, num_lfe, num_assoc_data, num_cc, sampling_index;
	 int comment_len;
	 int tags;

	 skip_bits(gb, 2);	// object_type

	 sampling_index = read_bits(gb, 4);
	 if (m4ac->sampling_index != sampling_index)
		 ALOGE("Sample rate index in program config element does not match the sample rate index configured by the container.\n");

	 num_front		 = read_bits(gb, 4);
	 num_side		 = read_bits(gb, 4);
	 num_back		 = read_bits(gb, 4);
	 num_lfe		 = read_bits(gb, 2);
	 num_assoc_data  = read_bits(gb, 3);
	 num_cc 		 = read_bits(gb, 4);

	 if (read_bits1(gb))
		 skip_bits(gb, 4); // mono_mixdown_tag
	 if (read_bits1(gb))
		 skip_bits(gb, 4); // stereo_mixdown_tag

	 if (read_bits1(gb))
		 skip_bits(gb, 3); // mixdown_coeff_index and pseudo_surround

	 if (get_bits_left(gb) < 4 * (num_front + num_side + num_back + num_lfe + num_assoc_data + num_cc)) {
		 ALOGE("decode_pce: overread_err" );
		 return -1;
	 }
	 decode_channel_map(layout_map		 , AAC_CHANNEL_FRONT, gb, num_front);
	 tags = num_front;
	 decode_channel_map(layout_map + tags, AAC_CHANNEL_SIDE,  gb, num_side);
	 tags += num_side;
	 decode_channel_map(layout_map + tags, AAC_CHANNEL_BACK,  gb, num_back);
	 tags += num_back;
	 decode_channel_map(layout_map + tags, AAC_CHANNEL_LFE,   gb, num_lfe);
	 tags += num_lfe;

	 skip_bits(gb, 4 * num_assoc_data);

	 decode_channel_map(layout_map + tags, AAC_CHANNEL_CC,	  gb, num_cc);
	 tags += num_cc;

	 align_get_bits(gb);

	 /* comment field, first byte is length */
	 comment_len = read_bits(gb, 8) * 8;
	 if (get_bits_left(gb) < comment_len) {
		 ALOGE("decode_pce: overread_err2");
		 return -1;
	 }
	 skip_bits(gb, comment_len);
	 return tags;
 }

 static int count_channels(uint8_t (*layout)[3], int tags)
 {
	 int i, sum = 0;
	 for (i = 0; i < tags; i++) {
		 int syn_ele = layout[i][0];
		 int pos	 = layout[i][2];
		 sum += (1 + (syn_ele == TYPE_CPE)) *
				(pos != AAC_CHANNEL_OFF && pos != AAC_CHANNEL_CC);
	 }
	 return sum;
 }

 /**
  * Decode GA "General Audio" specific configuration; reference: table 4.1.
  *
  * @param	 gb 		pointer to stream  not be null
  * @param	 m4ac	  pointer to MPEG4AudioConfig ,using for record the audio config
  *
  * @return  Returns error status. 0 - OK, !0 - error
  */
 static int decode_ga_specific_config(bitstream_t *gb,
									  MPEG4AudioConfig *m4ac,
									  int channel_config)
 {
	 int extension_flag, ret;
	 uint8_t layout_map[/*MAX_ELEM_ID*/16*4][3];
	 int tags = 0;

	 if (read_bits1(gb)) { // frameLengthFlag
		ALOGE("960/120 MDCT window");
		 return -1;
	 }

	 if (read_bits1(gb))	   // dependsOnCoreCoder
		 skip_bits(gb, 14);   // coreCoderDelay

	 extension_flag = read_bits1(gb);

	 if (m4ac->object_type == AOT_AAC_SCALABLE ||
		 m4ac->object_type == AOT_ER_AAC_SCALABLE)
		 skip_bits(gb, 3);	   // layerNr


	 if (channel_config == 0) {
		 skip_bits(gb, 4);	// element_instance_tag
		 tags = decode_pce(m4ac, layout_map, gb);
		 if (tags < 0)
			 return tags;
	 } else {
		 if ((ret = set_default_channel_config(layout_map, &tags, channel_config)))
			 return ret;
	 }

	 if (count_channels(layout_map, tags) > 1) {
		 m4ac->ps = 0;
	 } else if (m4ac->sbr == 1 && m4ac->ps == -1)
		 m4ac->ps = 1;


	 if (extension_flag) {
		 switch (m4ac->object_type) {
		 case AOT_ER_BSAC:
			 skip_bits(gb, 5);	  // numOfSubFrame
			 skip_bits(gb, 11);   // layer_length
			 break;
		 case AOT_ER_AAC_LC:
		 case AOT_ER_AAC_LTP:
		 case AOT_ER_AAC_SCALABLE:
		 case AOT_ER_AAC_LD:
			 skip_bits(gb, 3);	/* aacSectionDataResilienceFlag
									 * aacScalefactorDataResilienceFlag
									 * aacSpectralDataResilienceFlag
									 */
			 break;
		 }
		skip_bits(gb, 1);	 // extensionFlag3 (TBD in version 3)
	 }
	 return 0;
 }



 /**
  * Decode audio specific configuration; reference: table 1.13.
  *
  * @param	 m4ac		 pointer to MPEG4AudioConfig, used for parsing
  * @param	 data		 pointer to buffer holding an audio specific config
  * @param	 bit_size	 size of audio specific config or data in bits
  * @param	 sync_extension look for an appended sync extension
  *
  * @return  Returns error status or number of consumed bits. <0 - error
  */
 static int decode_audio_specific_config(MPEG4AudioConfig *m4ac,
										 const uint8_t *data, int bit_size,
										 int sync_extension)
 {
	 bitstream_t gb;
	 int i;

	 ALOGV("audio specific config size %d\n", bit_size >> 3);
	 for (i = 0; i < bit_size >> 3; i++)
		  ALOGV("%02x ", data[i]);


	 init_bits(&gb, (uint8_t*)data, bit_size);

	 if ((i = avpriv_mpeg4audio_get_config(m4ac, data, bit_size, sync_extension)) < 0)
		 return -1;
	 if (m4ac->sampling_index > 12) {
		 ALOGE("invalid sampling rate index %d\n", m4ac->sampling_index);
		 return -1;
	 }

	 skip_bits(&gb, i);

	 switch (m4ac->object_type) {
	 case AOT_AAC_MAIN:
	 case AOT_AAC_LC:
	 case AOT_AAC_LTP:
		 if (decode_ga_specific_config(&gb, m4ac, m4ac->chan_config))
			 return -1;
		 break;
	 default:
		 ALOGV( "Audio object type %s%d is not supported.\n", m4ac->sbr == 1? "SBR+" : "", m4ac->object_type);
		 return -1;
	 }

	 ALOGV("AOT %d chan config %d sampling index %d (%d) SBR %d PS %d\n",
			 m4ac->object_type, m4ac->chan_config, m4ac->sampling_index,
			 m4ac->sample_rate, m4ac->sbr, m4ac->ps);

	 return get_bits_count(&gb);
 }


 static int readAudioSpecificConfig(LatmAacExtConfig * latmAacExtConfig, bitstream_t *b ,int asclen)
 {
	 bitstream_t o;
	 int ret = 0;
	 int sync_extension = 0;
	 int specific_config_bitindex;

	 int bits_consumed, esize;
	 MPEG4AudioConfig *m4ac = &(latmAacExtConfig->aacConfig);

	 struct AACStruct *decoder = &(latmAacExtConfig->aacStruct);

	 int config_start_bit  = get_bits_count(b);


	 if (asclen) {
		 sync_extension = 1;
		 asclen 		= FFMIN(asclen, get_bits_left(b));
	 } else
		 asclen 		= get_bits_left(b);

	 if (config_start_bit % 8) {

		 return -1;
	 }

	 if(asclen <= 0)
		 return -1;
	 ALOGV("--->config_start_bit = %d sync_extension =%d",config_start_bit,sync_extension);


	  bits_consumed = decode_audio_specific_config(m4ac,
										  b->data + (config_start_bit / 8),
										  asclen, sync_extension);

	 if (bits_consumed < 0)
		 return -1;


	 skip_bits(b, bits_consumed);
	 ALOGV("consumed %d esize %d\n",bits_consumed,esize);
	 return bits_consumed;
 }

 /**
  * syntax of StreamMuxConfig()
  * the agreement:
  * 1.refer to the agreement of "syntax of AudioMuxElement" and "syntax of PayloadLengthInfo"
  * 2.the fields taraBufferFullness and latmBufferFullness shall be set to their largest
  * respective value, indicating that buffer fullness measures are not used in DVB context;
  * jan@2010-07-09
  * fixed by charles chen @2013-06-28
  */

 static int readStreamMuxConfig(LatmAacExtConfig * latmAacExtConfig, bitstream_t *b)
 {
	 int allStreamSameTimeFraming, numSubFrames, numPrograms, numLayer;

	 AACStruct * parser = &(latmAacExtConfig->aacStruct);

	 int audio_mux_version = read_bits(b, 1);
	 parser->audio_mux_version_A = 0;
	 if (audio_mux_version /*== 1*/) {				  // audioMuxVersion
		 parser->audio_mux_version_A = read_bits(b, 1);
	 }

	 if (parser->audio_mux_version_A == 0) {

		 int frame_length_type;

		 if (audio_mux_version	/*== 1*/) {
			 // taraFullness
			 latm_get_value(b);
		 }
		 allStreamSameTimeFraming = read_bits(b, 1);		 // allStreamSameTimeFraming = 1
		 numSubFrames = read_bits(b, 6);					// numSubFrames = 0
		 numPrograms = read_bits(b, 4); 				   // numPrograms = 0

		 // for each program (which there is only on in DVB)
		 numLayer = read_bits(b, 3);					// numLayer = 0

		 // code protect, to assume this code is only for dvb/isdb
		 if(numPrograms > 0 || numLayer > 0){
			 ALOGE("this stream is inllegal for ts stream tv, allStreamSameTimeFraming %d, numSubFrames %d, numPrograms %d, numLayer %d\n",
				 allStreamSameTimeFraming, numSubFrames, numPrograms, numLayer);
			 return -1;
		 }

		 // for each layer (which there is only on in DVB)
		 int ret = 0;
		 if (audio_mux_version == 0) {

			 if((ret = readAudioSpecificConfig(latmAacExtConfig, b,0))< 0)
				 return ret;

		 } else {

			 int ascLen = latm_get_value(b);
			 ret = readAudioSpecificConfig(latmAacExtConfig, b,ascLen);
			 if(ret < 0)
				 return ret;
			 ascLen -= ret;

			 // skip left over bits
			 while (ascLen > 16) {
				 skip_bits(b, 16);
				 ascLen -= 16;
			 }
			 skip_bits(b, ascLen);
		 }

		 // these are not needed... perhaps

		 frame_length_type = read_bits(b, 3);
		 //  ALOGI("frame_length_type %d",frame_length_type);
		 parser->frameLengthType = frame_length_type;

		 if (frame_length_type == 0) {

			 parser->frameLength = 0; // faad(maybe useful)
			 read_bits(b, 8);

		 } else if (frame_length_type == 1) {
			 //read_bits(b, 9);
			 parser->frameLength = read_bits(b, 9); // faad(maybe useful)

			 //the follow code not at ffmpeg ,so hide now
			 /* if(parser->frameLength > 0){ // faad(maybe useful)

				 parser->frameLength = (parser->frameLength + 20) * 8; // faad(maybe useful)

			  }*/
			 //3 frameLength = read_bits(b, 9);
		 } else if (frame_length_type == 3 || frame_length_type == 4 || frame_length_type == 5) {

			 // celp_table_index
			 read_bits(b, 6);
			 //3 CELPframeLengthTableIndex = read_bits(b, 6);

		 } else if (frame_length_type == 6 || frame_length_type == 7) {
			 // hvxc_table_index
			 read_bits(b, 1);
			 //3 HVXCframeLengthTableIndex = read_bits(b, 1);
		 }

		 // other data
		 if (read_bits(b, 1)) {
			 // other data present
			 if (audio_mux_version ) {
				 // other_data_bits
				 latm_get_value(b);
			 } else {
				 int esc, tmp;
				 // other data bits
				 int64_t other_data_bits = 0;
				 do {
					 esc = read_bits(b, 1);
					 tmp = read_bits(b, 8);
					 other_data_bits = other_data_bits << 8 | tmp;
				 } while (esc);
			 }
		 }

		 // CRC if necessary
		 if (read_bits(b, 1)) {
			 // config_crc
			 read_bits(b, 8);
		 }
	 } else {
		 // TBD
	 }
	 return 0;
 }

 /**
  * syntax of PlayloadLengthInfo()
  * the agreement:
  * 1.allStreamsSameTimeFraming shall be "1", as all payloads belong to the same access unit;
  * 2.numProgram shall be "0", as there is only one audio program per LATM multiplex;
  * 3.numLayer shall be "0", as no scalable profile is used; When MPEG Surround is used this
  * indicates that a single layer is present consisting of MPEG-4 AAC, MPEG-4 HE AAC or
  * MPEG-4 HE AAC v2 with embedded MPEG Surround data;
  * refer to chapator "6.4.1 LATM/LOAS formatting" of documation "ts_101154v010901p.pdf"
  * jan@2010-07-09
  */
 static int readPayloadLengthInfo(struct AACStruct *parser, bitstream_t *b)
 {
	 if (parser->frameLengthType == 0) {
		 uint8_t tmp;
		 int muxSlotLengthBytes = 0;
		 do {
			 tmp = read_bits(b, 8);
			 muxSlotLengthBytes += tmp;
		 } while (tmp == 255);
		 return muxSlotLengthBytes;
	 }else if(parser->frameLengthType == 1){ // faad(maybe useful)
	  return parser->frameLength;
	 } else {
		 if (parser->frameLengthType == 3 ||
				 parser->frameLengthType == 5 ||
				 parser->frameLengthType == 7) {
			 read_bits(b, 2);
			 //3 MuxSlotLengthCoded = read_bits(b, 2);
		 }
		 return 0;
	 }
 }

 /**
  * syntax of AudioMuxElement()
  * the agreement:
  * 1.audioMuxVersion shall be "0";
  * 2.numSubFrames shall be "0", as there is only one PayloadMux() (access unit) per LATM
  * AudioMuxElement();
  * refer to chapator "6.4.1 LATM/LOAS formatting" of documation "ts_101154v010901p.pdf"
  * jan@2010-07-09
  * fixed by charles chen @2013-06-28
  */

 static LATMERROR readAudioMuxElement(LatmAacExtConfig * latmAacExtConfig,bitstream_t *b)
 {
	 uint8_t use_same_mux = read_bits(b, 1);
	 uint8_t* pIndata = NULL;
	 uint8_t* payload = latmAacExtConfig->pOutputBuffer;
	 int32_t* payloadsize = &(latmAacExtConfig->outputBufferLength);
	 int ret  = 0;
	 struct AACStruct *parser = &(latmAacExtConfig->aacStruct);

	 if (b && b->data && (b->len >7)) {
		 pIndata = b->data;
	 }

	 if (!use_same_mux) {
		 if((ret = readStreamMuxConfig(latmAacExtConfig, b))<0)
			 return STREAM_ERROR;
	 }


	 if (parser->audio_mux_version_A == 0) {
		 int j;
		 int muxSlotLengthBytes = readPayloadLengthInfo(parser, b);

		 // ALOGI("latmaac: muxSlotLengthBytes %d *payloadsize %d", muxSlotLengthBytes,*payloadsize);
		 muxSlotLengthBytes = min(muxSlotLengthBytes, *payloadsize);

		 /**
		  * syntax of PayloadMux()
		  * the agreement: refer to the agreement of "syntax of PayloadLengthInfo"
		  * jan@2010-07-09
		  */
		 for (j=0; j<muxSlotLengthBytes; j++) {
			 *payload++ = read_bits(b, 8);
		 }
		 *payloadsize = muxSlotLengthBytes;
	 }
	 return STREAM_OK;
 }

 static LATMERROR readAudioSyncStream(LatmAacExtConfig * latmAacExtConfig)
 {
	 int muxlength;
	 bitstream_t b;

	 init_bits(&b, latmAacExtConfig->pInputBuffer, latmAacExtConfig->inputBufferCurrentLength* 8);

	 if (read_bits(&b, 11) != SYNC_LATM) return STREAM_ERROR;	 // not LATM

	 muxlength = read_bits(&b, 13);
	 ALOGV("latmaac : muxlength %d", muxlength);
	 if (muxlength + 3 > latmAacExtConfig->inputBufferCurrentLength){
		 ALOGE("latmaac: not a compelete latm packet muxlength %d vs size %d", muxlength, latmAacExtConfig->inputBufferCurrentLength);
		 return STREAM_NOT_ENOUGH;			// not enough data, the parser should have sorted this
	 }
	 return readAudioMuxElement(latmAacExtConfig, &b);
 }

 static LATMERROR latm_decode_frame(LatmAacExtConfig * latmAacExtConfig)
 {
	 bitstream_t		 b, out;
	 latmAacExtConfig->outputBufferLength = MAX_SIZE;

	 memset(&latmAacExtConfig->aacStruct, 0, sizeof(AACStruct));


	 ALOGV("latmaac: 1. size %d", latmAacExtConfig->inputBufferCurrentLength);
	 LATMERROR ret = readAudioSyncStream(latmAacExtConfig);
	 if (ret != STREAM_OK) {
		 ALOGD("latmaac: readAudioSyncStream happen something error code = %d",ret);
		 return ret;
	 }

	 /*parse decoder.extra*/
	 //we don't need the extra data
	 if(latmAacExtConfig->outputBufferLength <= 0){
		  ALOGD("latmaac: no raw data");
		 return STREAM_ERROR;
	 }
	 return STREAM_OK;
 }

 MediaBuffer * ElementaryStreamQueue::dequeueAccessUnitAAC_LATM() {
    Vector<size_t> frameOffsets;
    Vector<size_t> frameSizes;
    size_t auSize = 0;
    size_t offset = 0,muxlen = 0;
    uint8_t *data = mBuffer->data();
    uint32_t len = mBuffer->size();
    uint32_t skipFlag = false;

    if(!len)
	{
		return NULL;
	}

    MediaBuffer *tmpbuf = new MediaBuffer(mBuffer->size()+1024);

    if(tmpbuf == NULL)
    {
       return NULL;
    }

    uint8_t *outdata = (uint8_t *)tmpbuf->data();

	//in this loop we just find a latm frame then will return
    while (len >= LATM_HEADER_LEN) {

        if(data[offset] == 0x56 && (data[offset + 1] & 0xe0) == 0xe0)
        {
            muxlen = (data[offset + 1] & 0x1f) << 8 | data[offset + 2];

            if(len < muxlen + LATM_HEADER_LEN)
            {

				if(tmpbuf)
				{
					tmpbuf->release();
					tmpbuf = NULL;
				}
				return NULL;
            }
            int32_t bufsize = 0;

			//before latm decode ,we must
			mLatmAacExtConfig->pInputBuffer = data + offset;
			mLatmAacExtConfig->inputBufferCurrentLength = muxlen + LATM_HEADER_LEN;
			mLatmAacExtConfig->pOutputBuffer = outdata + tmpbuf->range_offset();
			mLatmAacExtConfig->outputBufferLength = 0;
			LatmAacErrorCode ret = latm_decode_frame(mLatmAacExtConfig);

			//ALOGI(" this %p latmaac: muxlen %d len %d offset %d ", this,muxlen,len,offset);
            if(ret != STREAM_OK) //muxlen is the length of latm without "0x56,0xXX,0xXX"
            {
				if(tmpbuf)
				{
					tmpbuf->release();
					tmpbuf = NULL;
				}
				//when the error is stream error ,just drop stream data



				if(ret != STREAM_NOT_ENOUGH)
				{

					// ALOGI(" this %p latmaac: muxlen %d len %d offset %d left %d", this,muxlen,len,offset,len-muxlen -offset);

					if(len > muxlen + offset && mFormat != NULL)
					{

#if LATM_AAC_DEBUG
							if(fp == NULL)
							{
								fp = fopen("/data/ts.aac","wb");
							}

							if(fp)
								fwrite(mBuffer->data(),1,muxlen +offset,fp);
#endif


							memmove(mBuffer->data(), mBuffer->data() + offset+muxlen, mBuffer->size() - offset-muxlen);
							mBuffer->setRange(0,len-muxlen -offset);


					}

					else
					{

#if LATM_AAC_DEBUG
							if(fp == NULL)
							{
								fp = fopen("/data/ts.aac","wb");
							}

							if(fp)
								fwrite(mBuffer->data(),1,len,fp);
#endif
					    mTimestamps.clear();
							mBuffer->setRange(0,0);

					}

				}


				return NULL;
				/*mLatmAacExtConfig->outputBufferLength = muxlen -3;

				memcpy(mLatmAacExtConfig->pOutputBuffer,mLatmAacExtConfig->pInputBuffer + 6,mLatmAacExtConfig->outputBufferLength);
				*/
			}


			if(mLatmAacExtConfig->outputBufferLength > 2048|| mLatmAacExtConfig->outputBufferLength  <= 0)
			{
				ALOGI(" this %p latmaac: bufsize %d ret %d inputlen %d muxlen %d",this, mLatmAacExtConfig->outputBufferLength,ret,len,muxlen);
				break;
			}


           /* ABitReader bits(outdata + tmpbuf->range_offset(), bufsize);
            CHECK_EQ(bits.getBits(12), 0xfffu);
            bits.skipBits(3);  // ID, layer
            bool protection_absent = bits.getBits(1) != 0;*/

            if (mFormat == NULL) {
				/*unsigned profile = bits.getBits(2);
                CHECK_NE(profile, 3u);
                unsigned sampling_freq_index = bits.getBits(4);
                bits.getBits(1);  // private_bit
                unsigned channel_configuration = bits.getBits(3);*/
                MPEG4AudioConfig aacConfig = mLatmAacExtConfig->aacConfig;
                if(aacConfig.object_type == -1 ||  aacConfig.sampling_index == -1||aacConfig.chan_config == -1)
                {
					ALOGI("--->remove this package data ");
					if(mLatmAacExtConfig->outputBufferLength <= len && mLatmAacExtConfig->outputBufferLength >0)
					{
						memmove(mBuffer->data(), mBuffer->data() + mLatmAacExtConfig->outputBufferLength,mBuffer->size() - mLatmAacExtConfig->outputBufferLength);
    					mBuffer->setRange(0, mBuffer->size() - mLatmAacExtConfig->outputBufferLength);
					}
					else if(mLatmAacExtConfig->outputBufferLength <= 0)
					{
						if(len > muxlen + offset )
						{
							    memmove(mBuffer->data(), mBuffer->data() + offset+muxlen, mBuffer->size() - offset-muxlen);
								mBuffer->setRange(0,len-muxlen -offset);
						}
						else{
					        mTimestamps.clear();
						    mBuffer->setRange(0, 0);
						}
					}

				   	if(tmpbuf)
					{
						tmpbuf->release();
						tmpbuf = NULL;
					}

					return NULL;
                }



				 // bits.skipBits(2);  // original_copy, home
                unsigned profile = 0x0003 & aacConfig.object_type;
			   	unsigned samplingFrequencyIndex = aacConfig.sampling_index;
				//if the channelconfig is 0 then we set the default value is 1 for meta first;in the case of 0, the channel configuration is sent via an inband PCE)
				unsigned channelConfig = aacConfig.chan_config?aacConfig.chan_config:2;
				ALOGI("-->profle %d sr %d ch %d",profile,samplingFrequencyIndex,channelConfig);

                mFormat = MakeAACCodecSpecificData(1,samplingFrequencyIndex, channelConfig);
			    mFormat->setInt32(kKeyIsLATM, true);
            }/* else {
                bits.skipBits(12);
            }

            bits.skipBits(2);
            unsigned aac_frame_length = bits.getBits(13);
            bits.skipBits(11);  // adts_buffer_fullness
            unsigned number_of_raw_data_blocks_in_frame = bits.getBits(2);

            if (number_of_raw_data_blocks_in_frame != 0) {
                TRESPASS();
        	}

            size_t headerSize = protection_absent ? 7 : 9;
			*/
            frameOffsets.push(tmpbuf->range_offset());
            frameSizes.push(mLatmAacExtConfig->outputBufferLength);
			//ALOGI("-->this is %p call the headerSize %d",this,headerSize);


            auSize += mLatmAacExtConfig->outputBufferLength;
            tmpbuf->set_range(0,tmpbuf->range_offset() + mLatmAacExtConfig->outputBufferLength);
            offset += (muxlen + LATM_HEADER_LEN);
            len -= (muxlen + LATM_HEADER_LEN);
            break;
        }
        else
        {
             offset++;
             len--;
        }
    }

    if (offset == 0) {
    	if(tmpbuf)
		{
			tmpbuf->release();
			tmpbuf = NULL;
		}
#if LATM_AAC_DEBUG
	if(fp == NULL)
	{
		fp = fopen("/data/ts.aac","wb");
	}

	if(fp)
		fwrite(mBuffer->data(),1,len,fp);
#endif
		mBuffer->setRange(0, 0);
		mTimestamps.clear();
		return NULL;
    }
    MediaBuffer * accessUnit = new MediaBuffer(auSize);
    size_t dstOffset = 0;
    for (size_t i = 0; i < frameOffsets.size(); ++i) {
        memcpy(accessUnit->data() + dstOffset,
               tmpbuf->data() + frameOffsets.itemAt(i),
               frameSizes.itemAt(i));
        dstOffset += frameSizes.itemAt(i);
    }

#if LATM_AAC_DEBUG
	if(fp == NULL)
	{
		fp = fopen("/data/ts.aac","wb");
	}

	if(fp)
		fwrite(mBuffer->data(),1,offset,fp);
#endif
    memmove(mBuffer->data(), mBuffer->data() + offset,
            mBuffer->size() - offset);

    mBuffer->setRange(0, mBuffer->size() - offset);

    tmpbuf->release();
    tmpbuf = NULL;
    int64_t timeUs = 0;
    if(mTimestamps.size() > 0)
    {
        timeUs = *mTimestamps.begin();
        lastTimeus = timeUs;
        mTimestamps.erase(mTimestamps.begin());
    }
    else
    {
        timeUs = lastTimeus;
    }
    accessUnit->meta_data()->setInt64(kKeyTime, timeUs);

    return accessUnit;

}
ElementaryStreamQueue:: ~ElementaryStreamQueue()
{
    if(SpsPpsBuf)
    {
        free(SpsPpsBuf);
        SpsPpsBuf = NULL;
    }
    mTimestamps.clear();
    mRangeInfos.clear();
	if(mLatmAacExtConfig)
	{
		delete mLatmAacExtConfig;
		mLatmAacExtConfig = NULL;
	}
    if(HevcParser_api != NULL){
        HevcParser_api->close(hevcparser_handle);
        free(HevcParser_api);
        HevcParser_api = NULL;
        hevcparser_handle = NULL;
    }
}
}  // namespace android
