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
#define LOG_TAG "wfd"
#include <utils/Log.h>

#include "sink/WifiDisplaySink.h"
#include "source/WifiDisplaySource.h"

#include <binder/ProcessState.h>
#include <binder/IServiceManager.h>
#include <gui/SurfaceComposerClient.h>
#include <media/AudioSystem.h>
#include <media/IMediaPlayerService.h>
#include <media/IRemoteDisplay.h>
#include <media/IRemoteDisplayClient.h>
#include <media/stagefright/DataSource.h>
#include <media/stagefright/foundation/ADebug.h>
#include <stdio.h>
#include <stdlib.h>
#include<string.h>
#include <cutils/properties.h> // for property_get

#define TRY_ARP_COUNT 36

namespace android {

static void usage(const char *me) {
    fprintf(stderr,
            "usage:\n"
            "           %s -c host[:port]\tconnect to wifi source\n"
            "               -u uri        \tconnect to an rtsp uri\n"
            "               -l ip[:port] \tlisten on the specified port "
            "(create a sink)\n",
            me);
}
#if 0
struct RemoteDisplayClient : public BnRemoteDisplayClient {
    RemoteDisplayClient();

    virtual void onDisplayConnected(
            const sp<ISurfaceTexture> &surfaceTexture,
            uint32_t width,
            uint32_t height,
            uint32_t flags);

    virtual void onDisplayDisconnected();
    virtual void onDisplayError(int32_t error);

    void waitUntilDone();

protected:
    virtual ~RemoteDisplayClient();

private:
    Mutex mLock;
    Condition mCondition;

    bool mDone;

    sp<SurfaceComposerClient> mComposerClient;
    sp<ISurfaceTexture> mSurfaceTexture;
    sp<IBinder> mDisplayBinder;

    DISALLOW_EVIL_CONSTRUCTORS(RemoteDisplayClient);
};

RemoteDisplayClient::RemoteDisplayClient()
    : mDone(false) {
    mComposerClient = new SurfaceComposerClient;
    CHECK_EQ(mComposerClient->initCheck(), (status_t)OK);
}

RemoteDisplayClient::~RemoteDisplayClient() {
}

void RemoteDisplayClient::onDisplayConnected(
        const sp<ISurfaceTexture> &surfaceTexture,
        uint32_t width,
        uint32_t height,
        uint32_t flags) {
    ALOGI("onDisplayConnected width=%u, height=%u, flags = 0x%08x",
          width, height, flags);

    mSurfaceTexture = surfaceTexture;
    mDisplayBinder = mComposerClient->createDisplay(
            String8("foo"), false /* secure */);

    SurfaceComposerClient::openGlobalTransaction();
    mComposerClient->setDisplaySurface(mDisplayBinder, mSurfaceTexture);

    Rect layerStackRect(1280, 720);  // XXX fix this.
    Rect displayRect(1280, 720);

    mComposerClient->setDisplayProjection(
            mDisplayBinder, 0 /* 0 degree rotation */,
            layerStackRect,
            displayRect);

    SurfaceComposerClient::closeGlobalTransaction();
}

void RemoteDisplayClient::onDisplayDisconnected() {
    ALOGD("onDisplayDisconnected");

    Mutex::Autolock autoLock(mLock);
    mDone = true;
    mCondition.broadcast();
}

void RemoteDisplayClient::onDisplayError(int32_t error) {
    ALOGI("onDisplayError error=%d", error);

    Mutex::Autolock autoLock(mLock);
    mDone = true;
    mCondition.broadcast();
}

void RemoteDisplayClient::waitUntilDone() {
    Mutex::Autolock autoLock(mLock);
    while (!mDone) {
        mCondition.wait(mLock);
    }
}
#endif
static status_t enableAudioSubmix(bool enable) {
    status_t err = AudioSystem::setDeviceConnectionState(
            AUDIO_DEVICE_IN_REMOTE_SUBMIX,
            enable
                ? AUDIO_POLICY_DEVICE_STATE_AVAILABLE
                : AUDIO_POLICY_DEVICE_STATE_UNAVAILABLE,
            NULL /* device_address */);

    if (err != OK) {
        return err;
    }

    err = AudioSystem::setDeviceConnectionState(
            AUDIO_DEVICE_OUT_REMOTE_SUBMIX,
            enable
                ? AUDIO_POLICY_DEVICE_STATE_AVAILABLE
                : AUDIO_POLICY_DEVICE_STATE_UNAVAILABLE,
            NULL /* device_address */);

    return err;
}

static void createSource(const AString &addr, int32_t port) {
    sp<IServiceManager> sm = defaultServiceManager();
    sp<IBinder> binder = sm->getService(String16("media.player"));
    sp<IMediaPlayerService> service =
        interface_cast<IMediaPlayerService>(binder);

    CHECK(service.get() != NULL);

    enableAudioSubmix(true /* enable */);

    String8 iface;
    iface.append(addr.c_str());
    iface.append(StringPrintf(":%d", port).c_str());
#if 0
    sp<RemoteDisplayClient> client;// = new RemoteDisplayClient;
    sp<IRemoteDisplay> display = service->listenForRemoteDisplay(client, iface);

    client->waitUntilDone();

    display->dispose();
    display.clear();
#endif
    enableAudioSubmix(false /* enable */);
}

}  // namespace android

static int Search_ipAddr(const char *hwSrcAddr,char *IpAddr)
{
	//FILE *fp = fopen("/data/misc/dhcp/dnsmasq.leases", "rb");
	FILE *fp = fopen("/proc/net/arp", "rb");
	ALOGI("##############");
	int iRet =-1;
	if (fp)
		{
			char *save_ptr;
			char *id, *hwAddr,*ipAddr,*hwType,*Flag,*Mash,*Device;
			char line[255];				
			while(fgets(line, sizeof(line), fp)) {
				const char *delim = " ";
				
				line[strlen(line)-1] = '\0';
				ALOGI("line =%s",line);

				if (!(ipAddr = strtok_r(line, delim, &save_ptr))) {
					ALOGI("Error parsing hwAddr");
				}
				
				if (!(hwType = strtok_r(NULL, delim, &save_ptr))) {
					ALOGI("Error parsing hwType");
				}
				if (!(Flag = strtok_r(NULL, delim, &save_ptr))) {
					ALOGI("Error parsing hwType");
				}				
				if (!(hwAddr = strtok_r(NULL, delim, &save_ptr))) {
					ALOGI("Error parsing hwAddr");
				}
				if (!(Mash = strtok_r(NULL, delim, &save_ptr))) {
					ALOGI("Error parsing Mash");
				}
				if (!(Device = strtok_r(NULL, delim, &save_ptr))) {
					ALOGI("Error parsing Device");
				}				
				ALOGI("Device =%s",Device);
				if(memcmp("p2p-p2p",Device,strlen("p2p"))==0 && memcmp("0x2",Flag,3)==0)
					{
						memcpy(IpAddr,ipAddr,strlen(ipAddr));
						ALOGI("####IpAddr ==%s",IpAddr);
						iRet =1;
						break;						
					}
			}
			fclose(fp);
			return iRet;
			
		}
	return iRet;
}

int main(int argc, char **argv) {
    using namespace android;

    ProcessState::self()->startThreadPool();

    DataSource::RegisterDefaultSniffers();

	int32_t iCount =0;
    AString connectToHost;
    int32_t connectToPort = -1;
    AString uri;

    AString listenOnAddr;
    int32_t listenOnPort = -1;
	AString connectAddr;
	ALOGE("start---------WFD");

    int res;
    while ((res = getopt(argc, argv, "s:hc:l:u:")) >= 0) {
        switch (res) {
            case 'c':
            {
				ALOGE("start---------WFD c");
                const char *colonPos = strrchr(optarg, ':');

                if (colonPos == NULL) {
                    connectToHost = optarg;
                    connectToPort = WifiDisplaySource::kWifiDisplayDefaultPort;
                } else {
                    connectToHost.setTo(optarg, colonPos - optarg);

                    char *end;
                    connectToPort = strtol(colonPos + 1, &end, 10);

                    if (*end != '\0' || end == colonPos + 1
                            || connectToPort < 1 || connectToPort > 65535) {
                        fprintf(stderr, "Illegal port specified.\n");
                        exit(1);
                    }
                }
                break;
            }
			case 's':
			{
				char ipAddr[PROPERTY_VALUE_MAX];
                const char *colonPos = strrchr(optarg, ':');
				ALOGE("start---------WFD s optarg=%s",optarg);
                if (colonPos == NULL) {
                    connectAddr = optarg;
                    connectToPort = WifiDisplaySource::kWifiDisplayDefaultPort;
                } else {
                    connectAddr.setTo(optarg, colonPos - optarg);

                    char *end;
                    connectToPort = strtol(colonPos + 1, &end, 10);

                    if (*end != '\0' || end == colonPos + 1
                            || connectToPort < 1 || connectToPort > 65535) {
                        fprintf(stderr, "Illegal port specified.\n");
                        exit(1);
                    }
					ALOGE("start---------WFD s connectAddr=%s",connectAddr.c_str());
					memset(ipAddr,0,sizeof(ipAddr));
					char buf[PROPERTY_VALUE_MAX];
                    int try_times = 20;
                    bool dhcp_result = false;
                    while (try_times--){
                    	property_get("dhcp.p2p.result", buf, NULL);
                        if (!strcmp(buf, "ok")){
                        	ALOGD("dhcp client udpate status %s", buf);
                            dhcp_result = true;
                            break;
                        }
                            usleep(1000*100); //sleep 100ms at one time 
                    }

                    if (dhcp_result){
                    	property_get("dhcp.p2p.server", ipAddr, NULL);
                        ALOGD("Client Role: connect to Host %s:%d", ipAddr,connectToPort);
                        connectToHost = ipAddr;
                    }else{
						for(iCount =0;iCount <TRY_ARP_COUNT;iCount++)
						{
							if(Search_ipAddr(connectAddr.c_str(),ipAddr) >0)
								break;
							usleep(300*1000);
						}
						if(iCount >=TRY_ARP_COUNT)
						{
							ALOGE("####iCount =%d > (TRY_ARP_COUNT=%d)",iCount,TRY_ARP_COUNT);
							exit(1);
						}
						
						connectToHost =(AString)ipAddr;
						ALOGE("start---------WFD s connectToHost =%s,connectToPort=%d",connectToHost.c_str(),connectToPort);
					}		
				}		
				break;
			}
            case 'u':
            {
                uri = optarg;
                break;
            }

            case 'l':
            {
                const char *colonPos = strrchr(optarg, ':');

                if (colonPos == NULL) {
                    listenOnAddr = optarg;
                    listenOnPort = WifiDisplaySource::kWifiDisplayDefaultPort;
                } else {
                    listenOnAddr.setTo(optarg, colonPos - optarg);

                    char *end;
                    listenOnPort = strtol(colonPos + 1, &end, 10);

                    if (*end != '\0' || end == colonPos + 1
                            || listenOnPort < 1 || listenOnPort > 65535) {
                        fprintf(stderr, "Illegal port specified.\n");
                        exit(1);
                    }
                }
                break;
            }

            case '?':
            case 'h':
            default:
                usage(argv[0]);
                exit(1);
        }
    }

    if (connectToPort >= 0 && listenOnPort >= 0) {
        fprintf(stderr,
                "You can connect to a source or create one, "
                "but not both at the same time.\n");
        exit(1);
    }

    if (listenOnPort >= 0) {
		ALOGD("listenOnPort listenOnPort %d",listenOnPort);
        createSource(listenOnAddr, listenOnPort);
        exit(0);
    }

    if (connectToPort < 0 && uri.empty()) {
        fprintf(stderr,
                "You need to select either source host or uri.\n");

        exit(1);
    }

    if (connectToPort >= 0 && !uri.empty()) {
        fprintf(stderr,
                "You need to either connect to a wfd host or an rtsp url, "
                "not both.\n");
        exit(1);
    }
	usleep(800*1000);
    sp<ANetworkSession> session = new ANetworkSession;
    session->start();

    sp<ALooper> looper = new ALooper;
	looper->setName("wfd");
	{
    sp<WifiDisplaySink> sink = new WifiDisplaySink(session);
    looper->registerHandler(sink);
    if (connectToPort >= 0) {
		ALOGE("start---------WFD s connectToPort=%d connectToHost.c_str() %s",connectToPort,connectToHost.c_str() );
        sink->start(connectToHost.c_str(), connectToPort);
    } else {
		ALOGE("start---------WFD s connectToPort=%d uri.c_str() %s",connectToPort,uri.c_str() );
        sink->start(uri.c_str());
    }

    looper->start(true /* runOnCallingThread */);
	}

    return 0;
}
