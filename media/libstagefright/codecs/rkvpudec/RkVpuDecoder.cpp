/*
 * Copyright (C) 2009 The Android Open Source Project
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
#define LOG_TAG "RkVpuDecoder"
#include <utils/Log.h>

#include "RkVpuDecoder.h"
#include <OMX_Component.h>
#include "ESDS.h"

#include <media/stagefright/MediaBufferGroup.h>
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/MediaDefs.h>
#include <media/stagefright/MediaErrors.h>
#include <media/stagefright/MetaData.h>
#include <media/stagefright/Utils.h>
#include <media/stagefright/foundation/hexdump.h>
#include "vpu_global.h"
#include <cutils/properties.h>

#define MAX_STREAM_LENGHT 1024*500
#define RKVPU_ALIGN(value, x)   ((value + (x-1)) & (~(x-1)))

namespace android {

struct CodeMap {
   OMX_RK_VIDEO_CODINGTYPE codec_id;
   const char *mime;
};

static const CodeMap kCodeMap[] = {
   { OMX_RK_VIDEO_CodingMPEG2, MEDIA_MIMETYPE_VIDEO_MPEG2},
   { OMX_RK_VIDEO_CodingH263,  MEDIA_MIMETYPE_VIDEO_H263},
   { OMX_RK_VIDEO_CodingMPEG4, MEDIA_MIMETYPE_VIDEO_MPEG4},
   { OMX_RK_VIDEO_CodingVC1,   MEDIA_MIMETYPE_VIDEO_VC1 },
   { OMX_RK_VIDEO_CodingWMV,   MEDIA_MIMETYPE_VIDEO_WMV3},
   { OMX_RK_VIDEO_CodingAVC,   MEDIA_MIMETYPE_VIDEO_AVC},
   { OMX_RK_VIDEO_CodingMJPEG, MEDIA_MIMETYPE_VIDEO_MJPEG},
   { OMX_RK_VIDEO_CodingFLV1,  MEDIA_MIMETYPE_VIDEO_FLV},
   { OMX_RK_VIDEO_CodingVP8,   MEDIA_MIMETYPE_VIDEO_VP8},
   { OMX_RK_VIDEO_CodingHEVC,   MEDIA_MIMETYPE_VIDEO_HEVC},
};

typedef enum
{
    MPEG4_MODE = 0,
    HEVC_MODE = 1,
    AVC_MODE = 2,
    VC1_MODE = 3,
    UNKNOWN_MODE,
} RKDecodingMode;


RkVpuDecoder::RkVpuDecoder(const sp<MediaSource> &source)
     :mSource(source),
      mStarted(false),
      mInputBuffer(NULL),
      mNumFramesOutput(0),
      mPendingSeekTimeUs(-1),
      mPendingSeekMode(MediaSource::ReadOptions::SEEK_CLOSEST_SYNC),
      mTargetTimeUs(-1),
      mWidth(0),
      mHeight(0),
      mVpuCtx(NULL),
      mExtraData(NULL),
      mPool(NULL),
      mExtraDataSize(0),
      mUseDtsTimeFlag(0),
      _success(true){

    ALOGV("new RkVpuDecoder in");
    mCodecId = OMX_RK_VIDEO_CodingUnused;

    getwhFlg = 0;
    memset(&mRkDecPrivate, 0, sizeof(RkDecPrivate_t));

    mFormat = new MetaData;
    uint32_t deInt = 0;
    mFormat->setInt32(kKeyDeIntrelace, deInt);
    mFormat->setCString(kKeyMIMEType, MEDIA_MIMETYPE_VIDEO_RAW);
    if(mSource != NULL){
        CHECK(mSource->getFormat()->findInt32(kKeyWidth, &mWidth));
        CHECK(mSource->getFormat()->findInt32(kKeyHeight, &mHeight));
        const char *mime;
        mSource->getFormat()->findCString(kKeyMIMEType, &mime);
        int32_t kNumMapEntries = sizeof(kCodeMap) / sizeof(kCodeMap[0]);
        for(int i = 0; i < kNumMapEntries; i++){
            if(!strcmp(kCodeMap[i].mime, mime)){
                mCodecId = kCodeMap[i].codec_id;
                ALOGD("mCodecId = %d video code mine %s",mCodecId,mime);
                break;
            }
        }
        mFormat->setInt32(kKeyWidth, mWidth);
        mFormat->setInt32(kKeyHeight, mHeight);
        if(mCodecId == OMX_RK_VIDEO_CodingUnused){
            _success = false;
        }
    }else{

        ALOGI("Wimo -------------------- ");
        ALOGI("mWidth = 0x%x,mHeight = 0x%x\n",mWidth,mHeight);
    }
    int ret = vpu_open_context(&mVpuCtx);
    if (ret || (mVpuCtx ==NULL)) {
        ALOGE("vpu_open_context fail");
        _success = false;
    }

	mFormat->setInt32(kKeyColorFormat, OMX_COLOR_FormatYUV420SemiPlanar);
    mFormat->setCString(kKeyDecoderComponent, "FLVDecoder");

    int64_t durationUs;
	if(mSource!=NULL)
	{
	    if (mSource->getFormat()->findInt64(kKeyDuration, &durationUs)) {
	        mFormat->setInt64(kKeyDuration, durationUs);
	    }
	}
	ALOGV("new RkVpuDecoder out");
}

RkVpuDecoder::~RkVpuDecoder() {
    if (mStarted) {
        stop();
    }

    vpu_close_context(&mVpuCtx);

    if(mExtraData)
	{
		free(mExtraData);
		mExtraData = NULL;
	}
    if(mPool != NULL){
        release_vpu_memory_pool_allocator(mPool);
        mPool = NULL;
	}
}

int RkVpuDecoder::getExtraData(int code_mode){
    uint32_t type;
    const void *data = NULL;
    size_t size = 0;
    sp<MetaData> meta = mSource->getFormat();
    switch(code_mode){
        case MPEG4_MODE:
        {
            if (meta->findData(kKeyESDS, &type, &data, &size)) {
                ESDS esds((const uint8_t *)data, size);

                const void *codec_specific_data;
                size_t codec_specific_data_size;
                if (esds.InitCheck() !=OK) {
                    codec_specific_data = data;
                    codec_specific_data_size = size;
                } else {
	                esds.getCodecSpecificInfo(
	                &codec_specific_data, &codec_specific_data_size);
                }
                mExtraData = (uint8_t*)malloc(codec_specific_data_size);
                if(!mExtraData){
                    ALOGE("mExtraData malloc fail");
                }
                mExtraDataSize= codec_specific_data_size;
                memcpy(mExtraData,codec_specific_data,mExtraDataSize);
            }
            break;
        }
        case HEVC_MODE:
        {
            if ((meta->findData(kKeyHVCC, &type, &data, &size)) && size) {
                mExtraData = (uint8_t*)malloc(size);
                mExtraDataSize = 0;
                static const uint8_t kNALStartCode[4] =
                { 0x00, 0x00, 0x00, 0x01 };

                if(!mExtraData){
                    ALOGE("mExtraData malloc fail");
                }
                const uint8_t *ptr = (const uint8_t *)data;
                // verify minimum size and configurationVersion == 1.
                if (size < 7) {
                    return ERROR_MALFORMED;
                }

                ptr += 22;
                size -= 22;

                size_t numofArrays = (char)ptr[0];
                ptr += 1;
                size -= 1;
                size_t j = 0, i = 0;
                for (i = 0; i < numofArrays; i++) {
                    ptr += 1;
                    size -= 1;
                    // Num of nals
                    size_t numofNals = U16_AT(ptr);
                    ptr += 2;
                    size -= 2;

                    for (j = 0;j < numofNals;j++) {
                        if (size < 2) {
                            return ERROR_MALFORMED;
                        }
                        size_t length = U16_AT(ptr);
                        ptr += 2;
                        size -= 2;
                        if (size < length) {
                            return ERROR_MALFORMED;
                        }

                        memcpy(mExtraData+mExtraDataSize,kNALStartCode, 4);
                        mExtraDataSize += 4;
                        memcpy(mExtraData+mExtraDataSize,ptr,length);
                        ptr += length;
                        size -= length;
                        mExtraDataSize += length;
                    }
                }
            }
            break;
        }
        case AVC_MODE:
        {
            if(meta->findData(kKeyAVCC, &type, &data, &size)&& size){
                mExtraData = (uint8_t*)malloc(size);
                if(!mExtraData){
                    ALOGE("mExtraData malloc fail");
                }
                mExtraDataSize= size;
                memcpy(mExtraData,data,mExtraDataSize);
            }
            break;
        }
        case VC1_MODE:
        {
            if (meta->findData(kKeyVC1, &type, &data, &size))
            {
                mExtraData = (uint8_t*)malloc(size);
                if(!mExtraData){
                    ALOGE("mExtraData malloc fail");
                }
                mExtraDataSize= size;
                memcpy(mExtraData,data,mExtraDataSize);
            }
            break;
        }
        default:
            break;
    }
    return OK;
}
status_t RkVpuDecoder::keyDataProcess(){
    const void *data = NULL;
    uint32_t type =0;
    size_t size = 0;
    int32_t vc1extraDataSize = 0;
    sp<MetaData> meta = mSource->getFormat();
    meta->findInt32(kKeyThumbnailDec,&mVpuCtx->no_thread);
    switch(mCodecId){
        case OMX_RK_VIDEO_CodingHEVC:
        {
            mFormat->setInt32(kKeyRkHevc, 1);
            getExtraData(HEVC_MODE);
            break;
        }

        case OMX_RK_VIDEO_CodingMPEG4:
        {
            mVpuCtx->enableparsing = 1;
            getExtraData(MPEG4_MODE);
            break;
        }
        case OMX_RK_VIDEO_CodingAVC:
        {
            mSource->getFormat()->findInt32(kKeyisTs, &mVpuCtx->extra_cfg.tsformat);
            mSource->getFormat()->findInt32(kKeyAvcSendDts, &mUseDtsTimeFlag);
            getExtraData(AVC_MODE);
            break;
        }
        case OMX_RK_VIDEO_CodingVC1:
        case OMX_RK_VIDEO_CodingWMV:
        {
			mVpuCtx->enableparsing = 1;
            if (!mVpuCtx->enableparsing) {
                getExtraData(VC1_MODE);
                meta->findInt32(kKeyVC1ExtraSize, &vc1extraDataSize);
                mVpuCtx->extra_cfg.vc1extra_size = vc1extraDataSize;
            } else {
                if (meta->findData(kKeyExtraData, &type, &data, &size)) {
                    mExtraData = (uint8_t*)malloc(size);
                    if(!mExtraData){
                        ALOGE("mExtraData malloc fail");
                    }
                    mExtraDataSize= size;
                    memcpy(mExtraData, data, mExtraDataSize);
                }
            }
            break;
        }
        case OMX_RK_VIDEO_CodingFLV1:
        case OMX_RK_VIDEO_CodingH263:
        case OMX_RK_VIDEO_CodingVP8:
        {
            mVpuCtx->enableparsing = 1;
        }

        default:
            break;

    }
    return OK;
}

status_t RkVpuDecoder::start(MetaData *) {

    CHECK(!mStarted);

	ALOGI("RkVpuDecoder::start in");
    if(!_success)
		return UNKNOWN_ERROR;

    mVpuCtx->enableparsing = 0;
    if(keyDataProcess()){
        ALOGE("RkVpuDecoder::start fail for keyDataProcess");
        return UNKNOWN_ERROR;
    }
    mVpuCtx->width = mWidth;
    mVpuCtx->height = mHeight;
    mVpuCtx->videoCoding = mCodecId;
    mVpuCtx->codecType = CODEC_DECODER;
    if(mVpuCtx->init(mVpuCtx, mExtraData, mExtraDataSize)){
        ALOGE("RkVpuDecoder::start fail for mVpuCtx->init");
        return !OK;
    }
    if((mWidth != 0) && (mHeight != 0) && !mVpuCtx->no_thread){
        int32_t align_w= mWidth;
        if(mCodecId == OMX_RK_VIDEO_CodingHEVC){
            align_w = ((mWidth+255)&(~255))| 256;
        }
        if(0 != create_vpu_memory_pool_allocator(&mPool,8,(align_w*mHeight*2))){
            return false;
        }
        if(mVpuCtx->control(mVpuCtx,VPU_API_SET_VPUMEM_CONTEXT,(void*)mPool) < 0){
            if(mPool != NULL){
                release_vpu_memory_pool_allocator(mPool);
                mPool = NULL;
            }
        }
    }
    mSource->start();
    mPendingSeekTimeUs = -1;
    mPendingSeekMode = ReadOptions::SEEK_CLOSEST_SYNC;
    mTargetTimeUs = -1;
    mStarted = true;
	ALOGV("RkVpuDecoder::start out ");

    return OK;
}


status_t RkVpuDecoder::stop() {
    CHECK(mStarted);

    if (mInputBuffer) {
        mInputBuffer->release();
        mInputBuffer = NULL;
    }

    mSource->stop();

    mStarted = false;

    return OK;
}

sp<MetaData> RkVpuDecoder::getFormat() {
    return mFormat;
}

int32_t RkVpuDecoder::checkVideoInfoChange(void* aFrame)
{
    if (aFrame ==NULL) {
        return 0;
    }
    if (OMX_RK_VIDEO_CodingHEVC == mCodecId) {
        return 0;
    }

    VPU_FRAME* frame = (VPU_FRAME*)aFrame;
    RkDecPrivate_t* pRkPrivat = &mRkDecPrivate;
    int32_t change = 0;

    int32_t w_old = RKVPU_ALIGN(mWidth, 16);
    int32_t h_old = RKVPU_ALIGN(mHeight, 16);
    int32_t w_new = RKVPU_ALIGN(frame->DisplayWidth, 16);
    int32_t h_new = RKVPU_ALIGN(frame->DisplayHeight, 16);

    if (!(pRkPrivat->flags & MBAFF_MODE_INFO_CHANGE) &&
            ((w_old != w_new) || (h_old != h_new))) {
        mFormat->setInt32(kKeyWidth, frame->DisplayWidth);
        mFormat->setInt32(kKeyHeight, frame->DisplayHeight);
        mWidth = frame->DisplayWidth;
        mHeight = frame->DisplayHeight;
        change =1;
        goto CHECK_INFO_OUT;
    }

    if (!(pRkPrivat->flags & MBAFF_MODE_INFO_CHANGE) &&
            (frame->FrameHeight >0) &&
            (frame->FrameHeight !=h_new)) {
		mFormat->setInt32(kKeyIsMbaff, 1);
        mFormat->setInt32(kKeyHeight, frame->FrameHeight);
        mHeight = frame->FrameHeight;
        pRkPrivat->flags |=MBAFF_MODE_INFO_CHANGE;
        change =1;
        goto CHECK_INFO_OUT;
    }

CHECK_INFO_OUT:
    if (change) {
        ALOGI("video size change, from (%d x %d) to (%d x %d)",
                w_old, h_old, mWidth, mHeight);
    }
    return change;
}

status_t RkVpuDecoder::read(
        MediaBuffer **out, const ReadOptions *options) {
    *out = NULL;

    if (mVpuCtx == NULL) {
        ALOGE("invalid vpu context");
        return !OK;
    }

    RkDecPrivate_t* pRkPrivat = &mRkDecPrivate;
    int64_t seekTimeUs;
    ReadOptions::SeekMode mode;
    if (options && options->getSeekTo(&seekTimeUs, &mode)) {
        if (!(pRkPrivat->flags & FIRST_FRAME)) {
            /*
             ** some rock-chip codecs such as rv, need to read sequence
             ** stream info before decode stream. so we need to save this
             ** seek request, and do seek after decoded at least one frame.
            */
            pRkPrivat->seek_req.mode = mode;
            pRkPrivat->seek_req.seekTimeUs = seekTimeUs;
            pRkPrivat->flags |=HAVE_SEEK_REQUEST;
        } else {
            ALOGV("seek requested to %lld us (%.2f secs)", seekTimeUs, seekTimeUs / 1E6);

            CHECK(seekTimeUs >= 0);
            mPendingSeekTimeUs = seekTimeUs;
            mPendingSeekMode = mode;

            if (mInputBuffer) {
                mInputBuffer->release();
                mInputBuffer = NULL;
            }
        }
    }

    /*
     ** check whether we have seek request. response seek after
     ** we have decoded at least one frame.
    */
    if ((pRkPrivat->flags & HAVE_SEEK_REQUEST) &&
            (pRkPrivat->flags & FIRST_FRAME)) {
        seekTimeUs = pRkPrivat->seek_req.seekTimeUs;
        mode = pRkPrivat->seek_req.mode;

        CHECK(seekTimeUs >= 0);
        mPendingSeekTimeUs = seekTimeUs;
        mPendingSeekMode = mode;

        if (mInputBuffer) {
            mInputBuffer->release();
            mInputBuffer = NULL;
        }

        /* clear seek request*/
        pRkPrivat->flags &=(~HAVE_SEEK_REQUEST);
    }

    if (mInputBuffer == NULL) {
        ALOGV("fetching new input buffer.");
        bool seeking = false;

        for (;;) {
            if (mPendingSeekTimeUs >= 0) {
                ALOGV("reading data from timestamp %lld (%.2f secs)",
                     mPendingSeekTimeUs, mPendingSeekTimeUs / 1E6);
            }

            ReadOptions seekOptions;
            if (mPendingSeekTimeUs >= 0) {
                seeking = true;
                seekOptions.setSeekTo(mPendingSeekTimeUs, mPendingSeekMode);
                mPendingSeekTimeUs = -1;
            }
            status_t err = OK;
            if (seekOptions.getSeekTo(&seekTimeUs, &mode)) {
                err = mSource->read(&mInputBuffer, &seekOptions);
            } else {
                err = mSource->read(&mInputBuffer, NULL);
            }
            seekOptions.clearSeekTo();

            if (err != OK) {
                *out = NULL;
                return (*out == NULL)  ? err : (status_t)OK;
            }

            if (mInputBuffer->range_length() > 0) {
                break;
            }

            mInputBuffer->release();
            mInputBuffer = NULL;
        }

        if (seeking) {
            int64_t targetTimeUs;
            if (mInputBuffer->meta_data()->findInt64(kKeyTargetTime, &targetTimeUs)
                    && targetTimeUs >= 0) {
                mTargetTimeUs = targetTimeUs;
            } else {
                mTargetTimeUs = -1;
            }
            int seekstatus = 0;
            if (mInputBuffer->meta_data()->findInt32(kKeySeekFail, &seekstatus) && seekstatus)
            {
                ALOGV("no process");
            } else{
                mVpuCtx->flush(mVpuCtx);
                /*fix when sps pps has change in mp4 steam seek directly will cause decoder error. modify by csy 2014.8.4*/
                if(mExtraDataSize != 0 && mCodecId == OMX_RK_VIDEO_CodingAVC){
                    VideoPacket_t pkt;
                    memset(&pkt, 0, sizeof(VideoPacket_t));
                    pkt.nFlags = OMX_BUFFERFLAG_EXTRADATA;
                    pkt.data = mExtraData;
                    pkt.size = mExtraDataSize;
                    mVpuCtx->decode_sendstream(mVpuCtx, &pkt);
                }
            }
        }
    }
    VideoPacket_t pkt;
    DecoderOut_t  picture;
    memset(&pkt, 0, sizeof(VideoPacket_t));
    memset(&picture, 0, sizeof(DecoderOut_t));
    int64_t inputTime = 0LL;

    pkt.data = (uint8_t*)mInputBuffer->data() + mInputBuffer->range_offset();
    pkt.size = mInputBuffer->range_length();

    mInputBuffer->meta_data()->findInt64(kKeyTime, &inputTime);
    if(mUseDtsTimeFlag){
        pkt.pts = VPU_API_NOPTS_VALUE;
        pkt.dts = inputTime;
    }else{
        pkt.pts = pkt.dts = inputTime;
    }

    int ret = 0;
    MediaBuffer *aOutBuf = new MediaBuffer(sizeof(VPU_FRAME));
    picture.data = (uint8_t *)aOutBuf->data();
    picture.size = 0;

    /*
     ** if we have open codec thread in vpu_api, then we can
     ** try to get output picture every time, this will make
     ** decoder thread more effective.
    */
    if (mVpuCtx->no_thread) {
        ret = mVpuCtx->decode(mVpuCtx,&pkt,&picture);
        if(ret != 0){
            if (mInputBuffer) {
                mInputBuffer->release();
                mInputBuffer = NULL;
            }
            if(picture.size){
                aOutBuf->releaseframe();//INFO Changed ,the output must be NULL;
            }else{
                aOutBuf->release();
            }
            return !OK;
        }
    } else {
        ret = mVpuCtx->decode_sendstream(mVpuCtx, &pkt);
        if(ret != 0){
            if (mInputBuffer !=NULL) {
                mInputBuffer->release();
                mInputBuffer = NULL;
            }
            return !OK;
        }

        ret = mVpuCtx->decode_getframe(mVpuCtx, &picture);
        if (ret !=0) {
            if (mInputBuffer !=NULL) {
                mInputBuffer->release();
                mInputBuffer = NULL;
            }

            if(picture.size){
                aOutBuf->releaseframe();
            }else{
                aOutBuf->release();
            }
            return !OK;
        }
    }

    if(picture.size){
        if (!(pRkPrivat->flags & FIRST_FRAME)) {
            pRkPrivat->flags |=FIRST_FRAME;

            /*
             ** drop video out frame if we have seek request.
            */
            if (pRkPrivat->flags & HAVE_SEEK_REQUEST) {
                aOutBuf->releaseframe();
                mNumFramesOutput++;
                *out = new MediaBuffer(0);
                (*out)->set_range(0, 0);
                return OK;
            }
        }

        VPU_FRAME *frame = (VPU_FRAME *)picture.data;
        if (checkVideoInfoChange(frame)) {
            aOutBuf->releaseframe();
            if(mInputBuffer && (!pkt.size)) {
                mInputBuffer->release();
                mInputBuffer = NULL;
            }
            return INFO_FORMAT_CHANGED;
        }

        mNumFramesOutput++;
    }

    if(mInputBuffer && (!pkt.size))
    {
        mInputBuffer->release();
        mInputBuffer = NULL;
    }

    aOutBuf->meta_data()->setInt64(kKeyTime, picture.timeUs);

    if (picture.size  <= 0) {
        aOutBuf->set_range(0, 0);
    }
    *out = aOutBuf;
    return OK;
}
void RkVpuDecoder::SetParameterForWimo(const sp<MediaSource> &source)
{
    ALOGI("Into AVCDecoder::SetParameterForWimo \n");
    int32_t width, height;
    mSource = source ;
    const char *mime;
    CHECK(mSource->getFormat()->findInt32(kKeyWidth, &width));
    CHECK(mSource->getFormat()->findInt32(kKeyHeight, &height));
    CHECK(mSource->getFormat()->findCString(kKeyMIMEType, &mime));
    mFormat->setInt32(kKeyWidth, width);
    mFormat->setInt32(kKeyHeight, height);

    int32_t kNumMapEntries = sizeof(kCodeMap) / sizeof(kCodeMap[0]);
    for(int i = 0; i < kNumMapEntries; i++){
        if(!strcmp(kCodeMap[i].mime, mime)){
            mCodecId = kCodeMap[i].codec_id;
            ALOGD("mCodecId = %d video code mine %s",mCodecId,mime);
            break;
        }
    }
    ALOGI("width = %d ,height = %d \n",width,height);
    ALOGI("Endof AVCDecoder::SetParameterForWimo \n");
    return ;
}

void RkVpuDecoder::signalBufferReturned(MediaBuffer *buffer) {

}

}  // namespace android
