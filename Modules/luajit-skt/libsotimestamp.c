#include <sys/socket.h>
#include <string.h>
#include <sys/time.h>

void set_msg_controllen(struct msghdr *msgh) {
	if (msgh == NULL) {
		msgh->msg_controllen = 0;
	} else {
		msgh->msg_controllen = CMSG_SPACE(sizeof(struct timeval));
	}
}

int get_msg_timestamp(struct timeval* tv_msg, struct msghdr *msgh) {
	if (tv_msg == NULL) {
		return -1;
	}
	if (msgh == NULL) {
		return -1;
	}
#ifdef SO_TIMESTAMP
    for (struct cmsghdr *cmsg = CMSG_FIRSTHDR(msgh); cmsg != NULL; cmsg = CMSG_NXTHDR(msgh, cmsg)){
        if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMP) {
            memcpy(tv_msg, CMSG_DATA(cmsg), sizeof(struct timeval));
            return 0;
        }
    }
#endif
    return -1;
}
