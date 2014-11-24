LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=                 \
        AnotherPacketSource.cpp   \
        ATSParser.cpp             \
        ESQueue.cpp               \
        MPEG2TSExtractor.cpp      \
			bitstream.cpp

LOCAL_C_INCLUDES:= \
		$(JNI_H_INCLUDE) \
		$(TOP)/frameworks/native/include/media/openmax \
	$(TOP)/frameworks/av/media/libstagefright \
	$(TOP)/frameworks/native/include/media/openmax \
	$(TOP)/hardware/rk29/librkvpu


LOCAL_MODULE:= libstagefright_mpeg2ts

ifeq ($(TARGET_ARCH),arm)
    LOCAL_CFLAGS += -Wno-psabi
endif

include $(BUILD_STATIC_LIBRARY)
