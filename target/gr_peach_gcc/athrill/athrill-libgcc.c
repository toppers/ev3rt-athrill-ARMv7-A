#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <stdlib.h>
#include "athrill_syscall.h"
#include "kernel.h"

#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"

#include "ev3api.h"

unsigned int athrill_device_func_call __attribute__ ((section(".athrill_device_section")));

#define OS_DLY_TSK(arg)	dly_tsk(arg)
//#define OS_DLY_TSK(arg)

void lwip_socket_init(void)
{
	//nothing to do
    return;
}

void lwip_init(void)
{
	lwip_socket_init();
}


int lwip_accept(int s, struct sockaddr *addr, socklen_t *addrlen)
{
	sys_int32 client_sockfd;
	struct sys_sockaddr_in client_sockaddr;

	do {
		client_sockfd = athrill_posix_accept(s, &client_sockaddr, (sys_uint32 *)addrlen);
		if (client_sockfd != SYS_API_ERR_AGAIN) {
			break;
		}
		else if (client_sockfd == SYS_API_ERR_AGAIN) {
			//sleep 1msec
			OS_DLY_TSK(1);
			continue;
		}
	} while (client_sockfd < 0);

	if (client_sockfd < 0) {
		return -1;
	}
	return client_sockfd;
}

int lwip_bind(int s, const struct sockaddr *name, socklen_t namelen)
{
	sys_int32 err;
	if (name->sa_family == PF_INET) {
		struct sys_sockaddr_in sockaddr;
		struct sockaddr_in *addr_in = (struct sockaddr_in*)name;

		sockaddr.sin_family = ATHRILL_SYSCALL_SOCKADDR_FAMILIY_PF_INET;
		sockaddr.sin_addr = addr_in->sin_addr.s_addr;
		sockaddr.sin_port = addr_in->sin_port;

		err = athrill_posix_bind(s, &sockaddr, sizeof(sockaddr));
		if (err != 0) {
			return -1;
		}
		return 0;
	}
	else {
		return -1;
	}
}
int lwip_shutdown(int s, int how)
{
	//nothing to do
    return 0;
}
int lwip_getpeername (int s, struct sockaddr *name, socklen_t *namelen)
{
    //not supported
    return -1;
}
int lwip_getsockname (int s, struct sockaddr *name, socklen_t *namelen)
{
    //not supported
    return -1;
}
int lwip_getsockopt (int s, int level, int optname, void *optval, socklen_t *optlen)
{
    //not supported
    return -1;
}
int lwip_setsockopt (int s, int level, int optname, const void *optval, socklen_t optlen)
{
    //not supported
    return -1;
}
int lwip_close(int s)
{
	(void)athrill_posix_shutdown(s, ATHRILL_POSIX_SHUT_RDWR);
    return 0;
}

int lwip_connect(int s, const struct sockaddr *name, socklen_t namelen)
{
	sys_int32 err;

	if (name->sa_family == PF_INET) {
		struct sys_sockaddr_in sockaddr;
		struct sockaddr_in *addr_in = (struct sockaddr_in*)name;

		sockaddr.sin_family = ATHRILL_SYSCALL_SOCKADDR_FAMILIY_PF_INET;
		sockaddr.sin_addr = addr_in->sin_addr.s_addr;
		sockaddr.sin_port = addr_in->sin_port;
		bool_t is_connected = 0;

	    syslog(LOG_NOTICE, "lwip_connect connect ip=%x port=%x", addr_in->sin_addr.s_addr, addr_in->sin_port);

		err = athrill_posix_connect(s, &sockaddr, sizeof(sockaddr));
		if (err != SYS_API_ERR_INPROGRESS) {
			return err;
		}
		while (is_connected == 0) {
			do {
				err = athrill_posix_sense(s, SYS_API_ID_CONNECT);
				if (err == SYS_API_ERR_OK) {
					is_connected = 1;
				}
				else if (err == SYS_API_ERR_AGAIN) {
					//sleep 1msec
					OS_DLY_TSK(1);
					continue;
				}
				else if (err == SYS_API_ERR_CONNREFUSED) {
					return err;
				}
			} while (err < 0);
		}
		return 0;
	}
	else {
		return -1;
	}
}

int lwip_listen(int s, int backlog)
{
	sys_int32 err = athrill_posix_listen(s, backlog);
	if (err != 0) {
		return -1;
	}
    return 0;
}

int lwip_recv(int s, void *mem, size_t len, int flags)
{
	sys_int32 err;

    do {
        err = athrill_posix_recv(s, (sys_addr)mem, len, ATHRILL_POSIX_MSG_DONTWAIT);
		if (err == SYS_API_ERR_AGAIN) {
			//sleep 1msec
			OS_DLY_TSK(1);
            continue;
		}
    } while (err < 0);

    return err;
}

int lwip_read(int s, void *mem, size_t len)
{
    //not supported
    return -1;
}
int lwip_recvfrom(int s, void *mem, size_t len, int flags,
      struct sockaddr *from, socklen_t *fromlen)
{
    //not supported
    return -1;
}
int lwip_send(int s, const void *dataptr, size_t size, int flags)
{
	sys_int32 err;

    do {
        err = athrill_posix_send(s, (sys_addr)dataptr, size, ATHRILL_POSIX_MSG_DONTWAIT);
		if (err == SYS_API_ERR_AGAIN) {
			//sleep 1msec
			OS_DLY_TSK(1);
            continue;
		}
		break;
    } while (1);

    return err;
}
int lwip_sendto(int s, const void *dataptr, size_t size, int flags,
    const struct sockaddr *to, socklen_t tolen)
{
    //not supported
    return -1;
}
int lwip_socket(int domain, int type, int protocol)
{
	int sockfd = athrill_posix_socket(ATHRILL_SYSCALL_SOCKET_DOMAIN_AF_INET, ATHRILL_SYSCALL_SOCKET_TYPE_STREAM, ATHRILL_SYSCALL_SOCKET_PROTOCOL_ZERO);
    return sockfd;
}
int lwip_write(int s, const void *dataptr, size_t size)
{
    //not supported
    return -1;
}

static void fd_set_copy(unsigned char *dst, unsigned char *src)
{
    sys_int32 i;
    sys_int32 fd_size = sizeof(fd_set);
    sys_int32 num = (fd_size < ATHRILL_FD_SETSIZE) ? fd_size: ATHRILL_FD_SETSIZE;

    for (i = 0; i < num; i++) {
        dst[i] = src[i];
    }
    return;
}
static int athrill_lwip_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset)
{
	sys_int32 ret;
	sys_fd_set sys_readfdset;
	sys_fd_set sys_writefdset;
	sys_fd_set sys_exceptfdset;

	memset((void*)&sys_readfdset, 0, sizeof(sys_fd_set));
	memset((void*)&sys_writefdset, 0, sizeof(sys_fd_set));
	memset((void*)&sys_exceptfdset, 0, sizeof(sys_fd_set));
	fd_set_copy((unsigned char*)&sys_readfdset, (unsigned char*)readset);
	fd_set_copy((unsigned char*)&sys_writefdset, (unsigned char*)writeset);
	fd_set_copy((unsigned char*)&sys_exceptfdset, (unsigned char*)exceptset);


	ret = athrill_posix_select(maxfdp1, &sys_readfdset, &sys_writefdset, &sys_exceptfdset);
	if (ret < 0) {
		return ret;
	}
	FD_ZERO(readset);
	FD_ZERO(writeset);
	FD_ZERO(exceptset);
	fd_set_copy((unsigned char*)readset, (unsigned char*)&sys_readfdset);
	fd_set_copy((unsigned char*)writeset, (unsigned char*)&sys_writefdset);
	fd_set_copy((unsigned char*)exceptset, (unsigned char*)&sys_exceptfdset);

    return ret;
}
int lwip_select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset,
                struct timeval *timeout)
{
	sys_int32 count = (timeout->tv_sec * 1000) + (timeout->tv_usec / 1000);
	sys_int32 ret;
	sys_int32 i;
	fd_set org_readset;
	fd_set org_writeset;
	fd_set org_exceptset;
	FD_ZERO(&org_readset);
	FD_ZERO(&org_writeset);
	FD_ZERO(&org_exceptset);

	if (count == 0) {
		count = 1;
	}

	for (i = 0; i < count; i++) {
		if (readset != NULL) {
			org_readset = *readset;
		}
		if (writeset != NULL) {
			org_writeset = *writeset;
		}
		if (exceptset != NULL) {
			org_exceptset = *exceptset;
		}
		ret = athrill_lwip_select(maxfdp1, &org_readset, &org_writeset, &org_exceptset);
		if (ret == 0) {
			//sleep 1msec
			OS_DLY_TSK(1);
		}
		else {
			return ret;
		}
	}
	return ret;
}
int lwip_ioctl(int s, long cmd, void *argp)
{
    //not supported
    return -1;
}
int lwip_fcntl(int s, int cmd, int val)
{
    //not supported
    return -1;
}
struct hostent *lwip_gethostbyname(const char *name)
{
	//TODO
	static struct hostent hent;
	static char *haddr_list[1];
	static unsigned int ipaddr[1];

	ipaddr[0] = 0;

	haddr_list[0] = (char*)&ipaddr;
	hent.h_addr_list = haddr_list;

	return &hent;
}
char *ipaddr_ntoa_r(const ip_addr_t *addr, char *buf, int buflen)
{
    //not supported
    return NULL;
}


// need to delete following files from lib.a
// lib_a-exit.o lib_a-__atexit.o lib_a-__call_atexit.o lib_a-fflush.o lib_a-findfp.o lib_a-refill.o
void exit(int status)
{
}

void __register_exitproc(void)
{

}
void __sync_synchronize(void)
{

}

// Special FD Handling
// (to convert one fd to in or out for bluetooth )
// index is fd for out. if the value is 0, it means the fd is normal file



int _close_r _PARAMS ((struct _reent *unused, int fd))
{
	return athrill_newlib_close_r(fd);
}

_off_t _lseek_r _PARAMS ((struct _reent *unused, int fd, _off_t offset, int whence))
{
	// TODO: how to treat when pipe
	return (_off_t) athrill_newlib_lseek_r(fd,offset,whence);
}

int _open_r _PARAMS ((struct _reent *unused, char *file_name, int flags, int mode))
{
	return athrill_newlib_open_r(file_name, flags, mode);
}

_ssize_t _read_r _PARAMS ((struct _reent *unused, int fd, void *buf, size_t size))
{
	// if read returns EAGAIN, repeat again after 10msec
	_ssize_t ret;	
	while ( (ret = athrill_newlib_read_r(fd, buf, size)) == -1 ) {
		if ( errno != SYS_API_ERR_AGAIN && errno != 0 ) break;
		tslp_tsk(100*1000); // 100msec(T.B.D.)
	}
	
	return ret;	
}

_ssize_t _write_r _PARAMS ((struct _reent *unused, int fd, const void *buf, size_t size))
{
	return (_ssize_t)athrill_newlib_write_r(fd, buf, size);
}



ER filesys_opendir(const char *path) {
		
	return athrill_ev3_opendir((sys_addr)path);
}

ER filesys_readdir(ID dirid, fatfs_filinfo_t *p_fileinfo) 
{

	return athrill_ev3_readdir(dirid, p_fileinfo);
}

ER filesys_closedir(ID dirid) 
{

	return athrill_ev3_closedir(dirid);

}

ER filesys_serial_open(sys_serial_port_t port)
{
	int fd = 0; // Default is stdout
	sys_int32 sys_port;
	
	if ( port == SYS_EV3_SERIAL_UART ) {
		sys_port = SYS_SERIAL_UART;
	} else if ( port == SYS_EV3_SERIAL_BT ) {
		sys_port = SYS_SERIAL_BT;
	} else {
		return -1;
	}

	fd = athrill_ev3_serial_open(sys_port);

	return fd;
}


#if 0 // For fopen optimization
 FILE * _EXFUN(fopen, (const char *__restrict file_name, const char *__restrict mode))
{
	if ( !is_top_dir_set ) {
		// check and set top_dir(only for first time)
		if ( athrill_set_virtfs_top((sys_addr)ev3rtfs_top_dir) == -1 ) {
			return 0;
		}
		is_top_dir_set = 1;
	}
	return (FILE *)athrill_posix_fopen((sys_addr)file_name,(sys_addr)mode);
}
int fclose(FILE *fp)
{
	return athrill_posix_fclose((sys_addr)fp);
}

size_t _EXFUN(fread, (_PTR __restrict buf, size_t size, size_t n, FILE *__restrict fp))
{
	return athrill_posix_fread((sys_addr)buf, size, n, (sys_addr)fp);
}

size_t _EXFUN(fwrite, (const _PTR __restrict buf, size_t size, size_t n, FILE * fp))
{
	return athrill_posix_fwrite((sys_addr)buf, size, n, (sys_addr)fp);
}


int _EXFUN(fflush, (FILE *fp))
{
	return athrill_posix_fflush((sys_addr)fp);
}


FILE * _EXFUN(fdopen, (int a, const char *b))
{
	//TODO syscall
	return NULL;
}
#endif
