/*
 * Copyright 2012, The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//#define LOG_NDEBUG 0
#define LOG_TAG "WifiDisplaySink"
#include <utils/Log.h>

#include "WifiDisplaySink.h"
#include "ParsedMessage.h"
#include "RTPSink.h"
#include <binder/IServiceManager.h>
#include <media/IHDCP.h>
#include <media/IMediaPlayerService.h>

#include <media/stagefright/foundation/ABuffer.h>
#include <media/stagefright/foundation/ADebug.h>
#include <media/stagefright/foundation/AMessage.h>
#include <media/stagefright/MediaErrors.h>
#include <stdio.h>
   #include <stdlib.h>
   #include <errno.h>
   #include <string.h>
   #include <sys/types.h>
   #include <sys/socket.h>
   #include <sys/un.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <ui/DisplayInfo.h>  // add by lance 2013.07.31
#include <gui/SurfaceComposerClient.h>  // add by lance 2013.07.31
#include <gui/ISurfaceComposer.h>  // add by lance 2013.07.31
#include <netdb.h>
#include <arpa/inet.h>
#include <cutils/properties.h>

#define SOCK_PATH "/data/data/wfd_socket"

namespace android {

WifiDisplaySink::WifiDisplaySink(
        const sp<ANetworkSession> &netSession)
    : mUsingHDCP(false),
      mHDCPInitializationComplete(false),
      mHDCP(NULL),
      mState(UNDEFINED),
      mNetSession(netSession),
      mSessionID(0),
      mNextCSeq(1),
	connectionCount(0) {
      mUrl = NULL;
	 end_flag = 0;
	 pthread_create(&mThread, NULL, rec_data, this);
}

WifiDisplaySink::~WifiDisplaySink() {
	void* retval1;
	end_flag = 1;
	connectionCount = 0;
	ALOGD("~WifiDisplaySink::~~WifiDisplaySink start mSocket_Rec_Client mState %d",mState);
	pthread_join(mThread, &retval1);
	if(mUrl != NULL)
	{
		delete mUrl;
		mUrl = NULL;
	}
	ALOGD("~WifiDisplaySink::~~WifiDisplaySink end");
	if(mRTPSink!=NULL)
	{
		if (mRTPSink->get_mRTCPSessionID() != 0) {
		      ALOGD("RTPSink::~RTPSink mRTCPSessionID %d",mRTPSink->get_mRTCPSessionID());
	             mRTPSink->get_mNetSession()->destroySession(mRTPSink->get_mRTCPSessionID());
	       }
	       if (mRTPSink->get_mRTPSessionID() != 0) {
		      ALOGD("RTPSink::~RTPSink mRTPSessionID %d",mRTPSink->get_mRTPSessionID());
	             mRTPSink->get_mNetSession()->destroySession(mRTPSink->get_mRTPSessionID());
	        }
		mRTPSink->clear_mNetSession();  // for resolve the sp problem modify by lance 2013.06.01
	}
	else
		ALOGD("WifiDisplaySink::~WifiDisplaySink quit unnormally ,maybe something wrong with the connection");
}

void WifiDisplaySink::start(const char *sourceHost, int32_t sourcePort) {
    sp<AMessage> msg = new AMessage(kWhatStart, id());
	ALOGI("start---------(shost,sport)");
    msg->setString("sourceHost", sourceHost);
    msg->setInt32("sourcePort", sourcePort);
    msg->post();
}

void WifiDisplaySink::start(const char *uri) {
    sp<AMessage> msg = new AMessage(kWhatStart, id());
	ALOGI("start---------(uri)");
    msg->setString("setupURI", uri);
    msg->post();
}
void* WifiDisplaySink::rec_data(void* me)
{
	WifiDisplaySink* wifidisplaysink = static_cast<WifiDisplaySink *>(me);
	ALOGD("%x wifidisplaysink 1 %x",me,wifidisplaysink);
	wifidisplaysink->ThreadWrapper(NULL);
	return NULL;
}
void *WifiDisplaySink::ThreadWrapper(void *)
{
	int s, client_sock, len;
	socklen_t t;
	struct sockaddr_un local, remote;
	char str[100];
	struct pollfd fds[2];
	if ((s = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
		ALOGE("open socket error");
		goto wfd_sink_thread_end;
	}
	fds[0].fd= s;
	fds[0].events = POLLIN;
	fds[0].revents = 0;

	fds[1].fd= -1;
	fds[1].events = POLLIN;
	fds[1].revents = 0;


	
	local.sun_family = AF_UNIX;
	strcpy(local.sun_path, SOCK_PATH);
	unlink(local.sun_path);
	len = strlen(local.sun_path) + sizeof(local.sun_family);
	if (bind(s, (struct sockaddr *)&local, len) == -1) {
		ALOGE("bin socket error");
		shutdown(fds[0].fd, SHUT_RDWR);
		close(fds[0].fd);
		fds[0].fd = -1;
		goto wfd_sink_thread_end;
	}
	chmod(SOCK_PATH,0x777);

	ALOGD(" before listen s %d" ,s);
	if (listen(s, 5) == -1) {
		ALOGE("listen socket error");
		shutdown(fds[0].fd, SHUT_RDWR);
		close(fds[0].fd);
		fds[0].fd = -1;
		goto wfd_sink_thread_end;
	}
	while(!end_flag)
	{
		int ret;
		if ((ret = poll(fds, 2, 50)) > 0)//poll(fds, 2, -1) > 0)  -1 ==inftim
		{
		
			if (fds[0].revents & POLLIN)
			{
				if ((client_sock = accept(s, (struct sockaddr *)&remote, &t)) == -1) {
					ALOGE("accept socket error");
					shutdown(fds[0].fd, SHUT_RDWR);
					close(fds[0].fd);
					fds[0].fd = -1;
					goto wfd_sink_thread_end;
				}
                if (client_sock  >= 0) {
                    ALOGE("receiver: accept client %d success\n", client_sock);
		      		fds[1].fd = client_sock ;
                } else {
                    ALOGE("receiver: accept client %d failed\n", client_sock);
					break;
                }				
			}
			if (fds[0].revents & POLLHUP)
			{
				ALOGD("sender shutdown fds[0] %d  fds[1]  %d",fds[0].revents,fds[1].revents);
				shutdown(fds[0].fd, SHUT_RDWR);
				close(fds[0].fd);
				fds[0].fd = -1;
				break;
	                    
			}
			if (fds[1].fd > 0)
			{
				
				if (fds[1].revents & POLLIN)
				{
					long temp;
					int n = recv(fds[1].fd, &temp, 4, 0);
					if(n > 0 && temp == 0x1234)
						sendIDR(mSessionID,mUrl->c_str());
					ALOGD("sendIDR mSessionID %d %s",mSessionID,mUrl->c_str());
				}
				if (fds[1].revents & POLLHUP)
				{
		
					shutdown(fds[1].fd, SHUT_RDWR);
					close(fds[1].fd);
					fds[1].fd = -1;
					break;
				}
				if(fds[1].revents & (POLLERR  | POLLNVAL))
				{
					ALOGD("fds 2 error  %d errno %d",fds[1].revents,errno);
					shutdown(fds[1].fd, SHUT_RDWR);
					close(fds[1].fd);
					fds[1].fd = -1;
					break;
				}	
			}
			
		}
	}
wfd_sink_thread_end:
	ALOGD("end of threadloop end_flag %d errno %d",end_flag,errno);
	if(fds[1].fd!=-1)
	{
		shutdown(fds[1].fd, SHUT_RDWR);
		close(fds[1].fd);
		fds[1].fd = -1;
	}
	if(fds[0].fd!=-1)
	{
		shutdown(fds[0].fd, SHUT_RDWR);
		close(fds[0].fd);
		fds[0].fd = -1;
	}
	return NULL;
}


// static
bool WifiDisplaySink::ParseURL(
        const char *url, AString *host, int32_t *port, AString *path,
        AString *user, AString *pass) {
    host->clear();
    *port = 0;
    path->clear();
    user->clear();
    pass->clear();

    if (strncasecmp("rtsp://", url, 7)) {
        return false;
    }

    const char *slashPos = strchr(&url[7], '/');

    if (slashPos == NULL) {
        host->setTo(&url[7]);
        path->setTo("/");
    } else {
        host->setTo(&url[7], slashPos - &url[7]);
        path->setTo(slashPos);
    }

    ssize_t atPos = host->find("@");

    if (atPos >= 0) {
        // Split of user:pass@ from hostname.

        AString userPass(*host, 0, atPos);
        host->erase(0, atPos + 1);

        ssize_t colonPos = userPass.find(":");

        if (colonPos < 0) {
            *user = userPass;
        } else {
            user->setTo(userPass, 0, colonPos);
            pass->setTo(userPass, colonPos + 1, userPass.size() - colonPos - 1);
        }
    }

    const char *colonPos = strchr(host->c_str(), ':');

    if (colonPos != NULL) {
        char *end;
        unsigned long x = strtoul(colonPos + 1, &end, 10);

        if (end == colonPos + 1 || *end != '\0' || x >= 65536) {
            return false;
        }

        *port = x;

        size_t colonOffset = colonPos - host->c_str();
        size_t trailing = host->size() - colonOffset;
        host->erase(colonOffset, trailing);
    } else {
        *port = 554;
    }

    return true;
}

void WifiDisplaySink::onMessageReceived(const sp<AMessage> &msg) {
    switch (msg->what()) {
        case kWhatStart:
        {
			ALOGI("start   sink");
            if (msg->findString("setupURI", &mSetupURI)) {
                AString path, user, pass;
                CHECK(ParseURL(
                            mSetupURI.c_str(),
                            &mRTSPHost, &mRTSPPort, &path, &user, &pass)
                        && user.empty() && pass.empty());
            } else {
                CHECK(msg->findString("sourceHost", &mRTSPHost));
                CHECK(msg->findInt32("sourcePort", &mRTSPPort));
            }

            sp<AMessage> notify = new AMessage(kWhatRTSPNotify, id());

            status_t err = mNetSession->createRTSPClient(
                    mRTSPHost.c_str(), mRTSPPort, notify, &mSessionID);
            CHECK_EQ(err, (status_t)OK);

            mState = CONNECTING;
            break;
        }

        case kWhatRTSPNotify:
        {
            int32_t reason;
            CHECK(msg->findInt32("reason", &reason));

            switch (reason) {
                case ANetworkSession::kWhatError:
                {
                    int32_t sessionID;
                    CHECK(msg->findInt32("sessionID", &sessionID));

                    int32_t err;
                    CHECK(msg->findInt32("err", &err));

                    AString detail;
                    CHECK(msg->findString("detail", &detail));

                    // ---------------
                    // add by lance 2014.05.20
                    if ((connectionCount < 25) && (mState == CONNECTING) && (err == -111)){
						++connectionCount;
                        ALOGD("RTSP server may not yet run, try reconnect");
                        usleep(200*1000);
                        start(mRTSPHost.c_str(),mRTSPPort);
                        break; 
                    }
                    // ---------------

                    ALOGE("An error occurred in session %d (%d, '%s/%s').",
                          sessionID,
                          err,
                          detail.c_str(),
                          strerror(-err));

                    if (sessionID == mSessionID) {
                        ALOGI("Lost control connection.");

                        // The control connection is dead now.
						ALOGD("WifiDisplaySink::onMessageReceivedSessionID %d",mSessionID);
						mNetSession->destroySession(mSessionID);
						mSessionID = 0;
						looper()->stop();
                        finishStop();  
                    }
                    break;
                }

                case ANetworkSession::kWhatConnected:
                {
                    ALOGI("We're now connected.");
                    mState = CONNECTED;

                    if (!mSetupURI.empty()) {
                        status_t err =
                            sendDescribe(mSessionID, mSetupURI.c_str());

                        CHECK_EQ(err, (status_t)OK);
                    }
                    break;
                }

                case ANetworkSession::kWhatData:
                {
					ALOGD("onReceiveClientData sessionID %d",mSessionID);
                    onReceiveClientData(msg);
                    break;
                }

                case ANetworkSession::kWhatBinaryData:
                {
                    CHECK(sUseTCPInterleaving);

                    int32_t channel;
                    CHECK(msg->findInt32("channel", &channel));

                    sp<ABuffer> data;
                    CHECK(msg->findBuffer("data", &data));

                    mRTPSink->injectPacket(channel == 0 /* isRTP */, data);
                    break;
                }

                default:
                    TRESPASS();
            }
            break;
        }
		 case kWhatHDCPNotify:
		{
			int32_t msgCode, ext1, ext2;
			CHECK(msg->findInt32("msg", &msgCode));
			CHECK(msg->findInt32("ext1", &ext1));
			CHECK(msg->findInt32("ext2", &ext2));

			ALOGI("Saw HDCP notification code %d, ext1 %d, ext2 %d",
					msgCode, ext1, ext2);

			switch (msgCode) {
				case HDCPModule::HDCP_INITIALIZATION_COMPLETE:
				{
					mHDCPInitializationComplete = true;
					break;
				}
				
				case HDCPModule::HDCP_INITIALIZATION_FAILED:
				{
					mHDCPInitializationComplete = false;
					break;
				}

				case HDCPModule::HDCP_SHUTDOWN_COMPLETE:
				case HDCPModule::HDCP_SHUTDOWN_FAILED:
				{
					// Ugly hack to make sure that the call to
					// HDCPObserver::notify is completely handled before
					// we clear the HDCP instance and unload the shared
					// library :(
					//(new AMessage(kWhatStop, id()))->post(300000ll);
					(new AMessage(kWhatStop, id()))->post(300000ll);
					break;
				}

				default:
				{
					ALOGE("HDCP failure, shutting down.");
					break;
				}
			}
			break;
		}

        case kWhatStop:
        {
			ALOGD("kWhatStop");
			mNetSession->destroySession(mSessionID);
			mSessionID = 0;
            looper()->stop();
			finishStop();  
            break;
        }

        default:
            TRESPASS();
    }
}

void WifiDisplaySink::registerResponseHandler(
        int32_t sessionID, int32_t cseq, HandleRTSPResponseFunc func) {
    ResponseID id;
    id.mSessionID = sessionID;
    id.mCSeq = cseq;
    mResponseHandlers.add(id, func);
}

status_t WifiDisplaySink::sendM2(int32_t sessionID) {
    AString request = "OPTIONS * RTSP/1.0\r\n";
    AppendCommonResponse(&request, mNextCSeq);

    request.append(
            "Require: org.wfa.wfd1.0\r\n"
            "\r\n");

	ALOGI("*******************************");
	ALOGI("%s\n",request.c_str());
	ALOGI("*******************************");
    status_t err =
        mNetSession->sendRequest(sessionID, request.c_str(), request.size());

    if (err != OK) {
        return err;
    }

    registerResponseHandler(
            sessionID, mNextCSeq, &WifiDisplaySink::onReceiveM2Response);

    ++mNextCSeq;

    return OK;
}

status_t WifiDisplaySink::onReceiveM2Response(
        int32_t sessionID, const sp<ParsedMessage> &msg) {
    int32_t statusCode;
    if (!msg->getStatusCode(&statusCode)) {
        return ERROR_MALFORMED;
    }

    if (statusCode != 200) {
        return ERROR_UNSUPPORTED;
    }

    return OK;
}

status_t WifiDisplaySink::onReceiveDescribeResponse(
        int32_t sessionID, const sp<ParsedMessage> &msg) {
    int32_t statusCode;
    if (!msg->getStatusCode(&statusCode)) {
        return ERROR_MALFORMED;
    }

    if (statusCode != 200) {
        return ERROR_UNSUPPORTED;
    }

    return sendSetup(sessionID, mSetupURI.c_str());
}

status_t WifiDisplaySink::onReceiveSetupResponse(
        int32_t sessionID, const sp<ParsedMessage> &msg) {
    int32_t statusCode;
	ALOGI("onReceiveSetupResponse********************");
    if (!msg->getStatusCode(&statusCode)) {
		ALOGI("status error******************");
        return ERROR_MALFORMED;
    }

    if (statusCode != 200) {
        return ERROR_UNSUPPORTED;
    }

    if (!msg->findString("session", &mPlaybackSessionID)) {
        return ERROR_MALFORMED;
    }

    if (!ParsedMessage::GetInt32Attribute(
                mPlaybackSessionID.c_str(),
                "timeout",
                &mPlaybackSessionTimeoutSecs)) {
        mPlaybackSessionTimeoutSecs = -1;
    }

    ssize_t colonPos = mPlaybackSessionID.find(";");
    if (colonPos >= 0) {
        // Strip any options from the returned session id.
        mPlaybackSessionID.erase(
                colonPos, mPlaybackSessionID.size() - colonPos);
    }

    status_t err = 0;//configureTransport(msg);

    if (err != OK) {
        return err;
    }

    mState = PAUSED;

    return sendPlay(
            sessionID,
            !mSetupURI.empty()
                ? mSetupURI.c_str() : mUrl->c_str());
}

status_t WifiDisplaySink::configureTransport(const sp<ParsedMessage> &msg) {
    if (sUseTCPInterleaving) {
        return OK;
    }

    AString transport;
    if (!msg->findString("transport", &transport)) {
        ALOGE("Missing 'transport' field in SETUP response.");
        return ERROR_MALFORMED;
    }

    AString sourceHost;
    if (!ParsedMessage::GetAttribute(
                transport.c_str(), "source", &sourceHost)) {
        sourceHost = mRTSPHost;
    }

    AString serverPortStr;
    if (!ParsedMessage::GetAttribute(
                transport.c_str(), "server_port", &serverPortStr)) {
        ALOGE("Missing 'server_port' in Transport field.");
		return OK;
        //return ERROR_MALFORMED;
    } else {

    int rtpPort, rtcpPort;
	ALOGI("serverPortStr = %s\n",serverPortStr.c_str());
	if(serverPortStr.find("-") >= 0)
	{
    if (sscanf(serverPortStr.c_str(), "%d-%d", &rtpPort, &rtcpPort) != 2
            || rtpPort <= 0 || rtpPort > 65535
            || rtcpPort <=0 || rtcpPort > 65535
            /*|| rtcpPort != rtpPort + 1*/) {  
			ALOGE("%d",rtcpPort);
        ALOGE("Invalid server_port description '%s'.",
                serverPortStr.c_str());

        return ERROR_MALFORMED;
    }
	}
	else
	{
		if(sscanf(serverPortStr.c_str(), "%d", &rtpPort) == 1)
		{
			rtcpPort = rtpPort+1;
		}
    }

    if (rtpPort & 1) {
        ALOGW("Server picked an odd numbered RTP port.");
    }

    return mRTPSink->connect(sourceHost.c_str(), rtpPort, rtcpPort);
  }
}

status_t WifiDisplaySink::onReceiveTearDownResponse(
         int32_t sessionID, const sp<ParsedMessage> &msg){
       ALOGI("I have received source teardown response");
       int32_t statusCode;
    if (!msg->getStatusCode(&statusCode)) {
        return ERROR_MALFORMED;
    }

    if (statusCode != 200) {
        return ERROR_UNSUPPORTED;
    }
    
    mState =  TEARDOWN;

	sp<AMessage> msg_stop = new AMessage(kWhatStop, id());
	msg_stop->post();
    return OK; 
 
}
status_t WifiDisplaySink::onReceivePlayResponse(
        int32_t sessionID, const sp<ParsedMessage> &msg) {
    int32_t statusCode;
    if (!msg->getStatusCode(&statusCode)) {
        return ERROR_MALFORMED;
    }

    if (statusCode != 200) {
        return ERROR_UNSUPPORTED;
    }

    mState = PLAYING;

    return OK;
}

void WifiDisplaySink::onReceiveClientData(const sp<AMessage> &msg) {
    int32_t sessionID;
    CHECK(msg->findInt32("sessionID", &sessionID));

    sp<RefBase> obj;
    CHECK(msg->findObject("data", &obj));

    sp<ParsedMessage> data =
        static_cast<ParsedMessage *>(obj.get());

    //ALOGD("session %d received %s",sessionID, data->debugString().c_str());
	ALOGI("****************************************\n");
	ALOGI("%s\n",data->debugString().c_str());
	ALOGI("****************************************\n");

    AString method;
    AString uri;
    data->getRequestField(0, &method);

    int32_t cseq;
    if (!data->findInt32("cseq", &cseq)) {
        sendErrorResponse(sessionID, "400 Bad Request", -1 /* cseq */);
        return;
    }

    if (method.startsWith("RTSP/")) {
        // This is a response.

        ResponseID id;
        id.mSessionID = sessionID;
        id.mCSeq = cseq;

        ssize_t index = mResponseHandlers.indexOfKey(id);

        if (index < 0) {
            ALOGW("Received unsolicited server response, cseq %d", cseq);
            return;
        }

        HandleRTSPResponseFunc func = mResponseHandlers.valueAt(index);
        mResponseHandlers.removeItemsAt(index);

        status_t err = (this->*func)(sessionID, data);
        CHECK_EQ(err, (status_t)OK);
    } else {
        AString version;
        data->getRequestField(2, &version);
        if (!(version == AString("RTSP/1.0"))) {
            sendErrorResponse(sessionID, "505 RTSP Version not supported", cseq);
            return;
        }

        if (method == "OPTIONS") {
            onOptionsRequest(sessionID, cseq, data);
        } else if (method == "GET_PARAMETER") {
            onGetParameterRequest(sessionID, cseq, data);
        } else if (method == "SET_PARAMETER") {
            onSetParameterRequest(sessionID, cseq, data);
        } else {
            sendErrorResponse(sessionID, "405 Method Not Allowed", cseq);
			ALOGD("onReceiveClientData::----- error 405");
        }
    }
}

void WifiDisplaySink::onOptionsRequest(
        int32_t sessionID,
        int32_t cseq,
        const sp<ParsedMessage> &data) {
    AString response = "RTSP/1.0 200 OK\r\n";
    AppendCommonResponse(&response, cseq);
    response.append("Public: org.wfa.wfd1.0, GET_PARAMETER, SET_PARAMETER\r\n");
    response.append("\r\n");

    status_t err = mNetSession->sendRequest(sessionID, response.c_str());
    CHECK_EQ(err, (status_t)OK);

    err = sendM2(sessionID);
    CHECK_EQ(err, (status_t)OK);
}

uint8_t WifiDisplaySink::vauleParsedByResolution(int32_t displayWidth, int32_t displayHeight, float displayFps)
{
    uint8_t parsedValue = 0x00;
    switch(displayWidth)
    {
       case 640:
	   	if(displayHeight == 480)
	   	{
	   	    parsedValue = 0x00;
	   	}
		else if(displayHeight == 360)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x32;
		    else
			parsedValue = 0x3A;
		}
		break;
	case 720:
		if(displayHeight == 480 && 1) // 1 stand for progressive frame   modify by lance 2013.07.31
                   parsedValue = 0x08;
		else if(displayHeight == 480 && 0)  // 0 stand for interlaced frame  modify by lance 2013.07.31
		    parsedValue =0x10;
		else if(displayHeight == 576 && 1)
		    parsedValue = 0x18;
		else if(displayHeight == 576 && 0)
		    parsedValue = 0x20;
		break;
	case 800:
		if(displayHeight == 600)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))
	               parsedValue = 0x01;
		    else
			parsedValue = 0x09;
		}
		else if(displayHeight == 480)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x02;
		    else
			parsedValue = 0x0A;
		}
		break;
	case 848:
		if(displayHeight == 480)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x52;
		    else
			parsedValue = 0x5A;
		}
		break;
	case 854:
		if(displayHeight == 480)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x12;
		    else
			parsedValue = 0x1A;
		}
		break;
	case 864:
		if(displayHeight == 480)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x22;
		    else
			parsedValue = 0x2A;
		}
		break;
	case 960:
		if(displayHeight == 540)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x42;
		    else
			parsedValue = 0x4A;
		}
		break;
	case 1024:
		if(displayHeight == 768)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))
	               parsedValue = 0x11;
		    else
			parsedValue = 0x19;
		}
		break;
	case 1152:
		if(displayHeight == 864)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))
	               parsedValue = 0x21;
		    else
			parsedValue = 0x29;
		}
		break;
	case 1280:
		if(displayHeight == 720)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x28;
		    else
			parsedValue = 0x30;
		}
		else if(displayHeight == 768)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x31;
		    else
			parsedValue = 0x39;
		}
		else if(displayHeight == 800)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x41;
		    else
			parsedValue = 0x49;
		}
		else if(displayHeight == 1024)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x71;
		    else
			parsedValue = 0x79;
		}
		break;
	case 1360:
		if(displayHeight == 768)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x51;
		    else
			parsedValue = 0x59;
		}
		break;
	case 1366:
		if(displayHeight == 768)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x61;
		    else
			parsedValue = 0x69;
		}
		break;
	case 1400:
		if(displayHeight == 1050)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x81;
		    else
			parsedValue = 0x89;
		}
		break;
	case 1440:
		if(displayHeight == 900)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0x91;
		    else
			parsedValue = 0x99;
		}
		break;
	case 1600:
		if(displayHeight == 900)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0xA1;
		    else
			parsedValue = 0xA9;
		}
		else if(displayHeight == 1200)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0xB1;
		    else
			parsedValue = 0xB9;
		}
		break;
	case 1680:
		if(displayHeight == 1024)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0xC1;
		    else
			parsedValue = 0xC9;
		}
		else if(displayHeight == 1050)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0xD1;
		    else
			parsedValue = 0xD9;
		}
		break;
	case 1920:
		if(displayHeight == 1080 && 1)  // 1 stand for the progressive frame  modify by lance 2013.07.31  // here should decide the fps 25/50/30 modify by lance 2013.07.31
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))
			 parsedValue = 0x38;
		    else
		        parsedValue = 0x40;
		}
		else if(displayHeight == 1080 && 0)  // 0 stand for the interlaced frame  modify by lance 2013.07.31
		    parsedValue = 0x48;
		else if(displayHeight == 1200)
		{
		    if(abs((int)displayFps - 30) < abs((int)displayFps - 60))  // here should step to decide 24\25\30\50\60 fps  modify by lance 2013.07.31
		        parsedValue = 0xE1;
		    else
			parsedValue = 0xE9;
		}
		break;
	default:
		break;		
    }
    return parsedValue;
}
void WifiDisplaySink::getParameterString(char *parameterByte, uint8_t parsedValue)
{
    sprintf(parameterByte, "%02x", parsedValue);   
}
void WifiDisplaySink::resolutionToParameterByte(char *parameterByte, int32_t displayWidth, int32_t displayHeight, float displayFps)
{
    uint8_t parsedValue = vauleParsedByResolution(displayWidth, displayHeight, displayFps);
    getParameterString(parameterByte, parsedValue);
}
void WifiDisplaySink::onGetParameterRequest(
        int32_t sessionID,
        int32_t cseq,
        const sp<ParsedMessage> &data) {
    AString msession;
	bool find = data->findString("Session",&msession);
    if(!find)
    {
	  DisplayInfo info;
	  sp<IBinder> display(SurfaceComposerClient::getBuiltInDisplay(ISurfaceComposer::eDisplayIdMain)); 
	  SurfaceComposerClient::getDisplayInfo(display, &info);
	  int32_t displayWidth ;
	  int32_t displayHeight ;
	  if(info.w > info.h)
	  {
	      displayWidth = (int32_t)info.w;
	      displayHeight = (int32_t)info.h;
          }
	  else
	  {
	      displayWidth = (int32_t)info.h;
	      displayHeight = (int32_t)info.w;
          }	 
         char parameterByte[3];  // for save the transfer byte modify by lance 2013.07.31
         memset(parameterByte, 0, 3);
	  resolutionToParameterByte(parameterByte, displayWidth, displayHeight, info.fps);
	  AString wfdVideoFormatsString(parameterByte);
	  wfdVideoFormatsString.append(" 00 02 02 0001DEFF 157C7FFF 00000FFF 00 0000 0000 00 none none");
#if 1
      const char *request_param = data->getContent();
             
	    AString body =  StringPrintf(
		"wfd_video_formats: %s\r\n"
               "wfd_audio_codecs: LPCM 00000002 00, AAC 0000000F 00\r\n", wfdVideoFormatsString.c_str());
            /* answers capabilities that the WFD source are only interested in */
            if (strstr(request_param, "wfd_3d_video_formats"))
                body.append("wfd_3d_video_formats: none\r\n");

        if (strstr(request_param, "wfd_uibc_capability"))
        	body.append("wfd_uibc_capability: none\r\n"); 
             
		/* using HDCP only if HDCP2.x Key inside, otherwise not support*/
		char prop_value[PROPERTY_VALUE_MAX];
		if (strstr(request_param, "wfd_content_protection"))
		{
			if (property_get("ro.secure.hdcp2xkey.inside",prop_value,NULL) && !strcmp(prop_value,"true"))
			{
				mUsingHDCP = true;
				body.append(StringPrintf("wfd_content_protection: HDCP2.1 port=%d\r\n",kHDCPDefaultPort));
			}
			else
			{
				body.append(StringPrintf("wfd_content_protection: none\r\n"));
				mUsingHDCP = false;
			}
			ALOGD("key inside ?:%s",prop_value);
		}
		else
		{
			mUsingHDCP = false;
		}

        if (strstr(request_param, "wfd_display_edid"))
        	body.append("wfd_display_edid: none\r\n");

        if (strstr(request_param, "wfd_coupled_sink"))
        	body.append("wfd_coupled_sink: none\r\n");
     	if (strstr(request_param, "wfd_connector_type"))
  		body.append("wfd_connector_type: 05\r\n");  //05:HDMI		
     body.append("wfd_client_rtp_ports: RTP/AVP/UDP;unicast 15550 0 mode=play\r\n");
#else
    AString body =
	       // "wfd_video_formats: xxx\r\n"
	      //  "wfd_audio_codecs: xxx\r\n"
	      //  "wfd_client_rtp_ports: RTP/AVP/UDP;unicast xxx 0 mode=play\r\n";
			 "wfd_video_formats: 00 00 02 02 0001DEFF 157C7FFF 00000FFF 00 0000 0000 00 none none\r\n"
			 "wfd_audio_codecs: LPCM 00000002 00, AAC 0000000F 00\r\n"
			 "wfd_uibc_capability: none\r\n"   //mtk phone request it
			 "wfd_client_rtp_ports: RTP/AVP/UDP;unicast 15550 0 mode=play\r\n";
#endif
	if (mUsingHDCP) {
		status_t errHDCP = makeHDCP();
		if (errHDCP != OK) {
		   ALOGE("Unable to instantiate HDCP component.");
		   mUsingHDCP = false;
		}
		/* important! sleep 100ms to ensure hdcp listen thread is running up */
		usleep(100*1000);
	}

    AString response = "RTSP/1.0 200 OK\r\n";
    AppendCommonResponse(&response, cseq);
    response.append("Content-Type: text/parameters\r\n");
    response.append(StringPrintf("Content-Length: %d\r\n", body.size()));
    response.append("\r\n");
    response.append(body);
		ALOGD("send m3 respose\n");
		ALOGD("%s\n",response.c_str());
	    status_t err = mNetSession->sendRequest(sessionID, response.c_str());
	    CHECK_EQ(err, (status_t)OK);
    }
	else
	{
		static int64_t last_time;
		 int64_t cur_time = systemTime(SYSTEM_TIME_MONOTONIC) / 1000;
		ALOGD("onReceiveClientData:: and send----- m16 time %lld %lld cseq %d",cur_time,
			cur_time- last_time, cseq);
		last_time = systemTime(SYSTEM_TIME_MONOTONIC) / 1000;
		AString response = "RTSP/1.0 200 OK\r\n";
	    AppendCommonResponse(&response, cseq);
		response.append("\r\n");
    status_t err = mNetSession->sendRequest(sessionID, response.c_str());
		ALOGI("%s\n",response.c_str());
    CHECK_EQ(err, (status_t)OK);
	}
}

status_t WifiDisplaySink::sendDescribe(int32_t sessionID, const char *uri) {
    uri = "rtsp://xwgntvx.is.livestream-api.com/livestreamiphone/wgntv";
    uri = "rtsp://v2.cache6.c.youtube.com/video.3gp?cid=e101d4bf280055f9&fmt=18";

    AString request = StringPrintf("DESCRIBE %s RTSP/1.0\r\n", uri);
    AppendCommonResponse(&request, mNextCSeq);

    request.append("Accept: application/sdp\r\n");
    request.append("\r\n");

    status_t err = mNetSession->sendRequest(
            sessionID, request.c_str(), request.size());

    if (err != OK) {
        return err;
    }

    registerResponseHandler(
            sessionID, mNextCSeq, &WifiDisplaySink::onReceiveDescribeResponse);

    ++mNextCSeq;

    return OK;
}

status_t WifiDisplaySink::sendSetup(int32_t sessionID, const char *uri) {
    mRTPSink = new RTPSink(mNetSession);
    looper()->registerHandler(mRTPSink);

    status_t err = mRTPSink->init(sUseTCPInterleaving);

    if (err != OK) {
        looper()->unregisterHandler(mRTPSink->id());
        mRTPSink.clear();
        return err;
    }

    AString request = StringPrintf("SETUP %s RTSP/1.0\r\n", uri);

    AppendCommonResponse(&request, mNextCSeq);

    if (sUseTCPInterleaving) {
        request.append("Transport: RTP/AVP/TCP;interleaved=0-1\r\n");
    } else {
        int32_t rtpPort = mRTPSink->getRTPPort();

        request.append(
                StringPrintf(
                    "Transport: RTP/AVP/UDP;unicast;client_port=%d-%d\r\n",
                    rtpPort, rtpPort + 1));
    }

    request.append("\r\n");

    ALOGV("request = '%s'", request.c_str());

    err = mNetSession->sendRequest(sessionID, request.c_str(), request.size());

    if (err != OK) {
        return err;
    }

    registerResponseHandler(
            sessionID, mNextCSeq, &WifiDisplaySink::onReceiveSetupResponse);

    ++mNextCSeq;

    return OK;
}

status_t WifiDisplaySink::sendPlay(int32_t sessionID, const char *uri) {
    AString request = StringPrintf("PLAY %s RTSP/1.0\r\n", uri);

    AppendCommonResponse(&request, mNextCSeq);

    request.append(StringPrintf("Session: %s\r\n", mPlaybackSessionID.c_str()));
    request.append("\r\n");

    status_t err =
        mNetSession->sendRequest(sessionID, request.c_str(), request.size());

    if (err != OK) {
        return err;
    }

    registerResponseHandler(
            sessionID, mNextCSeq, &WifiDisplaySink::onReceivePlayResponse);

    ++mNextCSeq;
    return OK;
}

status_t WifiDisplaySink::sendTearDown(int32_t sessionID, const char *uri) {
    AString request = StringPrintf("TEARDOWN %s RTSP/1.0\r\n", uri);

    AppendCommonResponse(&request, mNextCSeq);

    request.append(StringPrintf("Session: %s\r\n", mPlaybackSessionID.c_str()));
    request.append("\r\n");

    status_t err =
        mNetSession->sendRequest(sessionID, request.c_str(), request.size());

    if (err != OK) {
        return err;
    }

    registerResponseHandler(
            sessionID, mNextCSeq, &WifiDisplaySink::onReceiveTearDownResponse);

    ++mNextCSeq;

    return OK;
}

void WifiDisplaySink::sendIDR(int32_t sessionID, const char *uri)
{
    AString request = StringPrintf("SET_PARAMETER %s RTSP/1.0\r\n", uri);

    AppendCommonResponse(&request, mNextCSeq);
    request.append("Content-Type: text/parameters\r\n");
    request.append("Content-Length: 17\r\n");
    request.append("\r\n");
    request.append("wfd_idr_request\r\n");
    
    status_t err =
        mNetSession->sendRequest(sessionID, request.c_str(), request.size());

    ALOGI("%s\n",request.c_str());

    registerResponseHandler(
            sessionID, mNextCSeq, &WifiDisplaySink::onReceiveIdrResponse);
    
    if (err != OK) {
        return;
    }
    
    ++mNextCSeq;
    
}


status_t WifiDisplaySink::onReceiveIdrResponse(
        int32_t sessionID, const sp<ParsedMessage> &msg) {
    int32_t statusCode;
	ALOGD("WifiDisplaySink::onReceiveIdrResponse");
    if (!msg->getStatusCode(&statusCode)) {
        return ERROR_MALFORMED;
    }

    if (statusCode != 200) {
        return ERROR_UNSUPPORTED;
    }
    return OK;
}

void WifiDisplaySink::onSetParameterRequest(
        int32_t sessionID,
        int32_t cseq,
        const sp<ParsedMessage> &data) {
	if (mUsingHDCP && !mHDCPInitializationComplete) {
		ALOGI("HDCP initialization uncompletes.");
	}
	const char *content = data->getContent();
	char* position;
    if (strstr(content, "wfd_trigger_method: SETUP\r\n") != NULL) {
        status_t err =
            sendSetup(
                    sessionID,
                    mUrl->c_str());//rtsp://x.x.x.x:x/wfd1.0/streamid=0
					
        CHECK_EQ(err, (status_t)OK);
		
#if 1
		ALOGD("onSetParmeterRequest content %s",content);
	    AString response = "RTSP/1.0 200 OK\r\n";
	    AppendCommonResponse(&response, cseq);
	    response.append("\r\n");

	    err = mNetSession->sendRequest(sessionID, response.c_str());
	    CHECK_EQ(err, (status_t)OK);
#endif
   }
   else   if (strstr(content, "wfd_trigger_method: TEARDOWN\r\n") != NULL) {
            ALOGI("receive source trigger teardown request, do nothing"); 
            AString response = "RTSP/1.0 200 OK\r\n";
            AppendCommonResponse(&response, cseq);
            response.append("\r\n");
             status_t err1 = mNetSession->sendRequest(sessionID, response.c_str());
            CHECK_EQ(err1, (status_t)OK);
         
             status_t err =
            sendTearDown(
                    sessionID,
                    mUrl->c_str());//rtsp://x.x.x.x:x/wfd1.0/streamid=0
					
        CHECK_EQ(err, (status_t)OK);
    }
	else if ((position = strstr(content, "wfd_presentation_URL:"))!= NULL)
	{
		AString temp(position);
		int quo = temp.find("rtsp");
		
		if(quo >= 0)
		{
			int space = temp.find(" ",quo+1);
			if(space >= 0)
				mUrl = new AString(temp,quo,space-quo);
			ALOGD("SET_Parameter: parse url = %s\n",mUrl->c_str());
		}
		char* pos2;
		
		if((pos2 = strstr(content,"AAC")) != NULL){
				//AAC codec
			}else if ((pos2 = strstr(content,"LPCM")) != NULL){
				//LPCM codec
			}else{
				//LPCM
		}
		
		AString response = "RTSP/1.0 200 OK\r\n";
	    AppendCommonResponse(&response, cseq);
	    response.append("\r\n");
		ALOGI("send m4 response\n");
		ALOGI("%s\n",response.c_str());
	    status_t err = mNetSession->sendRequest(sessionID, response.c_str());
	    CHECK_EQ(err, (status_t)OK);
	}
	else
	{
		#if 1
		ALOGD("onSetParmeterRequest content %s",content);
	    AString response = "RTSP/1.0 200 OK\r\n";
	    AppendCommonResponse(&response, cseq);
	    response.append("\r\n");

	    status_t err = mNetSession->sendRequest(sessionID, response.c_str());
	    CHECK_EQ(err, (status_t)OK);
		#endif
	}
}

void WifiDisplaySink::sendErrorResponse(
        int32_t sessionID,
        const char *errorDetail,
        int32_t cseq) {
    AString response;
    response.append("RTSP/1.0 ");
    response.append(errorDetail);
    response.append("\r\n");

    AppendCommonResponse(&response, cseq);

    response.append("\r\n");

    status_t err = mNetSession->sendRequest(sessionID, response.c_str());
    CHECK_EQ(err, (status_t)OK);
}

void WifiDisplaySink::finishStop() 
{
    if (mHDCP != NULL)
	{
		ALOGI("Initiating HDCP shutdown.");
        mHDCP->shutdownAsync();
		mHDCP->setObserver(NULL);
		mHDCPObserver.clear();
		mHDCP.clear();
		return;
    }
}

/*void WifiDisplaySink::finishStop2() {
    if (mHDCP != NULL) {
        ALOGI("finishStop2.");
	mHDCP->setObserver(NULL);
        mHDCPObserver.clear();
        mHDCP.clear();
        mHDCP = NULL;
    }

    (new AMessage(kWhatStop, id()))->post(30000l);
}*/


struct WifiDisplaySink::HDCPObserver : public BnHDCPObserver {
    HDCPObserver(const sp<AMessage> &notify);

    virtual void notify(
            int msg, int ext1, int ext2, const Parcel *obj);

private:
    sp<AMessage> mNotify;

    DISALLOW_EVIL_CONSTRUCTORS(HDCPObserver);
};

WifiDisplaySink::HDCPObserver::HDCPObserver(
        const sp<AMessage> &notify)
    : mNotify(notify) {
}

void WifiDisplaySink::HDCPObserver::notify(
        int msg, int ext1, int ext2, const Parcel *obj) {
    sp<AMessage> notify = mNotify->dup();
    notify->setInt32("msg", msg);
    notify->setInt32("ext1", ext1);
    notify->setInt32("ext2", ext2);
    notify->post();
}

status_t WifiDisplaySink::makeHDCP() {
    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder = sm->getService(String16("media.player"));
    sp<IMediaPlayerService> service = interface_cast<IMediaPlayerService>(binder);
    CHECK(service != NULL);
    mHDCP = service->makeHDCP(false);
    if (mHDCP == NULL) {
        return ERROR_UNSUPPORTED;
    }

    sp<AMessage> notify = new AMessage(kWhatHDCPNotify, id());
    mHDCPObserver = new HDCPObserver(notify);
    status_t err = mHDCP->setObserver(mHDCPObserver);
    if (err != OK) {
        ALOGE("Failed to set HDCP observer.");
        mHDCPObserver.clear();
        mHDCP.clear();
        return err;
    }

	char* localIP = getFirstLocalAddress();
    err = mHDCP->initAsync(localIP, kHDCPDefaultPort);
    if (err != OK) {
        return err;
    }
    return OK;
}

char* WifiDisplaySink::getFirstLocalAddress(){
	char local[255] = {0};
	gethostname(local, sizeof(local));
	hostent* ph = gethostbyname(local);
	if(ph == NULL)
		return NULL;
	char *addrChar = NULL;
	for(int i=0; ph->h_addr_list[i]; ++i){
		in_addr addr;
		memcpy(&addr, ph->h_addr_list[i], sizeof(in_addr));
		addrChar = (inet_ntoa(addr));
		AString addrStr(addrChar);
		if(addrStr=="127.0.0.1"){
			addrChar = NULL;	
			continue;
		}
	}
	if(addrChar==NULL){
		char* anyAddr = (char*)"0.0.0.0";
		return anyAddr;
	}
	return addrChar;
}

// static
void WifiDisplaySink::AppendCommonResponse(AString *response, int32_t cseq) {
    time_t now = time(NULL);
    struct tm *now2 = gmtime(&now);
    char buf[128];
    strftime(buf, sizeof(buf), "%a, %d %b %Y %H:%M:%S %z", now2);

    response->append("Date: ");
    response->append(buf);
    response->append("\r\n");

    response->append("User-Agent: stagefright/1.1 (Linux;Android 4.1):rockchip\r\n");

    if (cseq >= 0) {
        response->append(StringPrintf("CSeq: %d\r\n", cseq));
    }
}

}  // namespace android
