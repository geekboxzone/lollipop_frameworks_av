#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cutils/log.h>
#define LOG_TAG "vpu_mem_observer"

#define SUN_LEN(ptr) ((size_t) (((struct sockaddr_un *) 0)->sun_path) \
                      + strlen ((ptr)->sun_path))

int main(int argc, char *argv[])
{
    int sock;
    fd_set testfds; 
    char buf[200] = "\0";
    int i;

    if ((sock = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        ALOGE("create socket error\n");
	return 0;
    }

    struct sockaddr_un uaddr;
    uaddr.sun_family = AF_UNIX;
    strcpy(uaddr.sun_path, "/data/vpumem_observer");

    if (connect(sock, (struct sockaddr*)&uaddr, SUN_LEN(&uaddr)) < 0) {
        ALOGE("connect error");
        return 0;
    }
    
    strcat(buf, "mem");

    for (i=1; i<argc; i++) {
        strcat(buf, argv[i]);
        strcat(buf, " ");
    }

    send(sock, buf, strlen(buf), 0);

    FD_ZERO(&testfds);
    FD_SET(sock, &testfds);

    select(sock+1, (fd_set *)NULL, &testfds, (fd_set *)NULL, NULL);

    close(sock);

    return 0;
}

