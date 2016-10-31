/*!
  \file
   Linux的串口配置

  $Id: urg_serial_linux.c,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/

#include "urg_ring_buffer.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>


enum {
    INVALID_FD = -1,
};

/*
 * 串口初始化，需要初始化一个ringbuffer来存储接收到的数据
 */
static void serial_initialize(urg_serial_t *serial)
{
    serial->fd = INVALID_FD;
    serial->has_last_ch = False;

    ring_initialize(&serial->ring, serial->buffer, RING_BUFFER_SIZE_SHIFT);
}

/*
 * 清除串口接收到的数据
 */
static void serial_clear(urg_serial_t* serial)
{
    tcdrain(serial->fd);
    tcflush(serial->fd, TCIOFLUSH);
    ring_clear(&serial->ring);
    serial->has_last_ch = False;
}

/*
 打开串口
 */
int serial_open(urg_serial_t *serial, const char *device, long baudrate)
{
    int flags = 0;
    int ret = 0;

    serial_initialize(serial);

#ifndef URG_MAC_OS
    enum { O_EXLOCK = 0x0 }; /*  Not used in Linux, used as dummy */
#endif
	/* Linux系统打开一个设备，详细的可以看 UNIX高级环境编程 */
    serial->fd = open(device, O_RDWR | O_EXLOCK | O_NONBLOCK | O_NOCTTY);
    if (serial->fd < 0) {
        /*  设备打开失败 */
        //strerror_r(errno, serial->error_string, ERROR_MESSAGE_SIZE);
        return -1;
    }

    flags = fcntl(serial->fd, F_GETFL, 0);
    fcntl(serial->fd, F_SETFL, flags & ~O_NONBLOCK);

    /*  串口初始化部分  */
    tcgetattr(serial->fd, &serial->sio);
    serial->sio.c_iflag = 0;
    serial->sio.c_oflag = 0;
	/* 下面的大写字母表示的都是Linux系统的宏定义，没有必要全部搞懂，看一下了解一下 */
    serial->sio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
    serial->sio.c_cflag |= CS8 | CREAD | CLOCAL;
    serial->sio.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);

    serial->sio.c_cc[VMIN] = 0;
    serial->sio.c_cc[VTIME] = 0;
	/* 串口设置波特率 */
    ret = serial_set_baudrate(serial, baudrate);
    if (ret < 0) {
        return ret;
    }

    /*  Initializes serial control structures */
    serial->has_last_ch = False;

    return 0;
}

/*
 * 关闭已经打开的串口
 */
void serial_close(urg_serial_t *serial)
{
    if (serial->fd >= 0) {
        close(serial->fd);
        serial->fd = INVALID_FD;
    }
}

/*
 * 串口波特率设置
 */
int serial_set_baudrate(urg_serial_t *serial, long baudrate)
{
    long baudrate_value = -1;

    switch (baudrate) {
    case 4800:
        baudrate_value = B4800;
        break;

    case 9600:
        baudrate_value = B9600;
        break;

    case 19200:
        baudrate_value = B19200;
        break;

    case 38400:
        baudrate_value = B38400;
        break;

    case 57600:
        baudrate_value = B57600;
        break;

    case 115200:
        baudrate_value = B115200;
        break;

    default:
        return -1;
    }

    /*  设置输入输出波特率(Linux系统调用) */
    cfsetospeed(&serial->sio, baudrate_value);
    cfsetispeed(&serial->sio, baudrate_value);
	/* 设置终端属性 */
    tcsetattr(serial->fd, TCSADRAIN, &serial->sio);
    serial_clear(serial);

    return 0;
}

/*
 * 向串口写入数据
 */
int serial_write(urg_serial_t *serial, const char *data, int size)
{
    if (serial->fd == INVALID_FD) {
        return -1;
    }
    return write(serial->fd, data, size);
}

/*
 * 串口等待接收数据
 */
static int wait_receive(urg_serial_t* serial, int timeout)
{
    fd_set rfds;
    struct timeval tv;

    // Configures the timeout
    FD_ZERO(&rfds);
    FD_SET(serial->fd, &rfds);

    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;
	/* 这里的这个select函数是Linux系统调用，这个好好看一下到底怎么用的，一般需要配合上面的FD_XXX宏来使用 */
    if (select(serial->fd + 1, &rfds, NULL, NULL,
               (timeout < 0) ? NULL : &tv) <= 0) {
        /*  Timeout occurred */
        return 0;
    }
    return 1;
}


static int internal_receive(char data[], int data_size_max,
                            urg_serial_t* serial, int timeout)
{
    int filled = 0;

    if (data_size_max <= 0) {
        return 0;
    }

    while (filled < data_size_max) {
        int require_n;
        int read_n;

        if (! wait_receive(serial, timeout)) {
            break;
        }

        require_n = data_size_max - filled;
        read_n = read(serial->fd, &data[filled], require_n);
        if (read_n <= 0) {
            /*  Read error, returns all the data up to now */
            break;
        }
        filled += read_n;
    }
    return filled;
}

/*
 * 从串口读取数据
 */
int serial_read(urg_serial_t *serial, char *data, int max_size, int timeout)
{
    int buffer_size;
    int read_n;
    int filled = 0;

    if (max_size <= 0) {
        return 0;
    }

    /*  If there is a single character return it */
    if (serial->has_last_ch != False) {
        data[0] = serial->last_ch;
        serial->has_last_ch = False;
        ++filled;
    }
    if (serial->fd == INVALID_FD) {
        if (filled > 0) {
            return filled;
        } else {
            return -1;
        }
    }

    buffer_size = ring_size(&serial->ring);
    read_n = max_size - filled;
    if (buffer_size < read_n) {
        // Reads data if there is space in the ring buffer
        char buffer[RING_BUFFER_SIZE];
        int n = internal_receive(buffer,
                                 ring_capacity(&serial->ring) - buffer_size,
                                 serial, 0);
        if (n > 0) {
            ring_write(&serial->ring, buffer, n);
            buffer_size += n;
        }
    }

    // Returns the data stored in the ring buffer
    if (read_n > buffer_size) {
        read_n = buffer_size;
    }
    if (read_n > 0) {
        ring_read(&serial->ring, &data[filled], read_n);
        filled += read_n;
    }

    // Reads data within the given timeout
    filled += internal_receive(&data[filled], max_size - filled,
                               serial, timeout);
    return filled;
}
