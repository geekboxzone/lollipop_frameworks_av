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
#define LOG_TAG "StreamingSource"
#include <utils/Log.h>

#include "StreamingSource.h"

#include "ATSParser.h"
#include "AnotherPacketSource.h"
#include "NuPlayerStreamListener.h"

#include <media/stagefright/foundation/ABuffer.h>
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/MediaSource.h>
#include <media/stagefright/MetaData.h>
#include "../config.h"
#include <utils/CallStack.h>
namespace android {

NuPlayer::StreamingSource::StreamingSource(
        const sp<AMessage> &notify,
        const sp<IStreamSource> &source)
    : Source(notify),
      mSource(source),
      mWFDFlag(false),
      mWFDStartSysTimeUs(0),
      mWFDStartMediaTimeUs(0), 
      mFinalResult(OK),
      mBuffering(false) {
      sys_time_base = streaming_audio_start_timeUs = streaming_sys_start_timeUs = StreamingSource_Sign = 0;
}

NuPlayer::StreamingSource::~StreamingSource() {
    if (mLooper != NULL) {
        mLooper->unregisterHandler(id());
        mLooper->stop();
    }
}

void NuPlayer::StreamingSource::prepareAsync() {
    if (mLooper == NULL) {
        mLooper = new ALooper;
        mLooper->setName("streaming");
        mLooper->start();

        mLooper->registerHandler(this);
    }

    notifyVideoSizeChanged();
    notifyFlagsChanged(0);
    notifyPrepared();
}

void NuPlayer::StreamingSource::start() {
    mStreamListener = new NuPlayerStreamListener(mSource, 0);

    uint32_t sourceFlags = mSource->flags();
    if ((sourceFlags >> 16 & 0xFFFF) == 0x1234) {
    	mWFDFlag = true;
	StreamingSource_Sign = 1;
	ALOGD("NuPlayer::StreamingSource::start sourceFlags %x",sourceFlags);
    }

    uint32_t parserFlags = ATSParser::TS_TIMESTAMPS_ARE_ABSOLUTE;
    if (sourceFlags & IStreamSource::kFlagAlignedVideoData) {
        parserFlags |= ATSParser::ALIGNED_VIDEO_DATA;
    }

    mTSParser = new ATSParser(parserFlags);

    mStreamListener->start();

    postReadBuffer();
}

status_t NuPlayer::StreamingSource::feedMoreTSData() {
    return postReadBuffer();
}

void NuPlayer::StreamingSource::onReadBuffer() {
    //for (int32_t i = 0; i < 100; ++i) {
    ssize_t n;
    do {
        char buffer[188];
        sp<AMessage> extra;
		// 188 bytes or -11
        n = mStreamListener->read(buffer, sizeof(buffer), &extra);

        if (n == 0) {
            ALOGI("input data EOS reached.");
            mTSParser->signalEOS(ERROR_END_OF_STREAM);
            setError(ERROR_END_OF_STREAM);
            break;
        } else if (n == INFO_DISCONTINUITY) {
            int32_t type = ATSParser::DISCONTINUITY_TIME;

		int64_t sys_timeUs;
		int64_t mediaTimeUs;

		int64_t sys_time = 0ll;
		int64_t timeUs = 0ll;
		int	temp;

        int32_t mask;
        if (extra != NULL
                && extra->findInt32(
                    IStreamListener::kKeyDiscontinuityMask, &mask)) {
            if (mask == 0) {
                ALOGE("Client specified an illegal discontinuity type.");
                setError(ERROR_UNSUPPORTED);
                break;
            }

            type = mask;
        }

		extra->findInt64("timeUs", &timeUs);
		if(!extra->findInt64("wifidisplay_sys_timeUs", &sys_time)  )
		{
			mTSParser->signalDiscontinuity((ATSParser::DiscontinuityType)type, extra);
		}

		if(extra->findInt32("first_packet", &temp))
		{
			mTSParser->set_player_type(3);// jmj for wfd
			ALOGD("first_packet set ATsparser type 3");
		}
		if(StreamingSource_Sign == 1)
		{
			streaming_sys_start_timeUs 		= sys_time;
			streaming_audio_start_timeUs	= 	timeUs ;
		}
		
			/*if (mWFDFlag) { 
				if (extra->findInt64("wifidisplay_sys_timeUs", &sys_timeUs) && extra->findInt64("timeUs", &mediaTimeUs)) {
					mWFDStartSysTimeUs = sys_timeUs; 
					 mWFDStartMediaTimeUs = mediaTimeUs;
				}
			} else {
            			mTSParser->signalDiscontinuity((ATSParser::DiscontinuityType)type, extra);
			}*/
           // ----------------------
        } else if (n < 0) {
            break;
        } else {
            if (buffer[0] == 0x00) {
                // XXX legacy

                if (extra == NULL) {
                    extra = new AMessage;
                }

                uint8_t type = buffer[1];

                if (type & 2) {
                    int64_t mediaTimeUs;
                    memcpy(&mediaTimeUs, &buffer[2], sizeof(mediaTimeUs));

                    extra->setInt64(IStreamListener::kKeyMediaTimeUs, mediaTimeUs);
                }

                mTSParser->signalDiscontinuity(
                        ((type & 1) == 0)
                            ? ATSParser::DISCONTINUITY_TIME
                            : ATSParser::DISCONTINUITY_FORMATCHANGE,
                        extra);
            } else {
            	//this way 188 bytes
                status_t err = mTSParser->feedTSPacket(buffer, sizeof(buffer));
				int64_t systime = systemTime(SYSTEM_TIME_MONOTONIC) / 1000ll; 
				ALOGV("	mTSParser->feedTSPacket	systime = %lld. ", systime);

                if (err != OK) {
                    ALOGE("TS Parser returned error %d", err);

                    mTSParser->signalEOS(err);
                    setError(err);
                    break;
                }
            }
        }
    }while(n > 0);
}

status_t NuPlayer::StreamingSource::postReadBuffer() {
    {
        Mutex::Autolock _l(mBufferingLock);
        if (mFinalResult != OK) {
            return mFinalResult;
        }
        if (mBuffering) {
            return OK;
        }
        mBuffering = true;
    }

    (new AMessage(kWhatReadBuffer, id()))->post();
    return OK;
}

bool NuPlayer::StreamingSource::haveSufficientDataOnAllTracks() {
    // We're going to buffer at least 2 secs worth data on all tracks before
    // starting playback (both at startup and after a seek).

    static const int64_t kMinDurationUs = 5000ll;

    sp<AnotherPacketSource> audioTrack = getSource(true /*audio*/);
    sp<AnotherPacketSource> videoTrack = getSource(false /*audio*/);

    status_t err;
    int64_t durationUs;
    if (audioTrack != NULL
            && (durationUs = audioTrack->getBufferedDurationUs(&err))
                    < kMinDurationUs
            && err == OK) {
        ALOGV("audio track doesn't have enough data yet. (%.2f secs buffered)",
              durationUs / 1E6);
        return false;
    }

    if (videoTrack != NULL
            && (durationUs = videoTrack->getBufferedDurationUs(&err))
                    < kMinDurationUs
            && err == OK) {
        ALOGE("video track doesn't have enough data yet. (%.2f secs buffered)",
              durationUs / 1E6);
        return false;
    }

    return true;
}

void NuPlayer::StreamingSource::setError(status_t err) {
    Mutex::Autolock _l(mBufferingLock);
    mFinalResult = err;
}

sp<AnotherPacketSource> NuPlayer::StreamingSource::getSource(bool audio) {
    if (mTSParser == NULL) {
        return NULL;
    }

    sp<MediaSource> source = mTSParser->getSource(
            audio ? ATSParser::AUDIO : ATSParser::VIDEO);

    return static_cast<AnotherPacketSource *>(source.get());
}

sp<MetaData> NuPlayer::StreamingSource::getFormatMeta(bool audio) {
    sp<AnotherPacketSource> source = getSource(audio);

    if (source == NULL) {
        return NULL;
    }

    return source->getFormat();
}

status_t NuPlayer::StreamingSource::dequeueAccessUnit(
        bool audio, sp<ABuffer> *accessUnit) {
    sp<AnotherPacketSource> source = getSource(audio);

    if (source == NULL) {
        return -EWOULDBLOCK;
    }

    if (!haveSufficientDataOnAllTracks()) {
        postReadBuffer();
    }

    status_t finalResult;
    if (!source->hasBufferAvailable(&finalResult)) {
        return finalResult == OK ? -EWOULDBLOCK : finalResult;
    }
	int64_t  systime = systemTime(SYSTEM_TIME_MONOTONIC) / 1000ll; 

    status_t err = source->dequeueAccessUnit(accessUnit);

#if !defined(LOG_NDEBUG) || LOG_NDEBUG == 0
    if (err == OK) {
        int64_t timeUs;
        CHECK((*accessUnit)->meta()->findInt64("timeUs", &timeUs));
        ALOGD_IF(DEBUG_DELAY_TIME,"dequeueAccessUnit timeUs=%lld us is video %d ", timeUs, !audio);
    }
#endif

    return err;
}

bool NuPlayer::StreamingSource::isRealTime() const {
    return mSource->flags() & IStreamSource::kFlagIsRealTimeData;
}

void NuPlayer::StreamingSource::onMessageReceived(
        const sp<AMessage> &msg) {
    switch (msg->what()) {
        case kWhatReadBuffer:
        {
            onReadBuffer();

            {
                Mutex::Autolock _l(mBufferingLock);
                mBuffering = false;
            }
            break;
        }
        default:
        {
            TRESPASS();
        }
    }
}

bool NuPlayer::StreamingSource::isWFDStreaming()
{
    return mWFDFlag != 0;
}

int64_t NuPlayer::StreamingSource::getWFDStartSysTimeUs() {
    return  mWFDStartSysTimeUs;
}

int64_t NuPlayer::StreamingSource::getWFDStartMediaTimeUs() {
    return  mWFDStartMediaTimeUs; 
}

uint32_t NuPlayer::StreamingSource::flags() const {
     return 0;
}

int	NuPlayer::StreamingSource::getwifidisplay_info(int *info)
{
	return (StreamingSource_Sign != 0);
};
int	NuPlayer::StreamingSource::Wifidisplay_get_TimeInfo(int64_t *start_time,int64_t *audio_start_time)
{

	if(1 == StreamingSource_Sign)
	{
		if(streaming_sys_start_timeUs !=0 && streaming_audio_start_timeUs !=0 )
		{
			*start_time 		=	streaming_sys_start_timeUs;
			*audio_start_time	=	streaming_audio_start_timeUs;
		}
	}
	ALOGV("StreamingSource_Sign %d start_time %lld %lld",StreamingSource_Sign,*start_time,*audio_start_time );
	return (1 == StreamingSource_Sign) ? 0 : -1;
}



}  // namespace android

