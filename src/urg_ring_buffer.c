/*!
  \file
  \brief Implements a ring buffer
  \author Satofumi KAMIMURA

  $Id: urg_ring_buffer.c,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/

#include "urg_ring_buffer.h"

/*
 * 初始化一个环形buffer
 */
void ring_initialize(ring_buffer_t *ring, char *buffer, const int shift_length)
{
    ring->buffer = buffer;
    ring->buffer_size = 1 << shift_length;
    ring_clear(ring);
}

/*
 * 清空一个环形buffer
 */
void ring_clear(ring_buffer_t *ring)
{
    ring->first = 0;
    ring->last = 0;
}

/*
 * 返回一个环形buffer当前占用空间的大小
 */
int ring_size(const ring_buffer_t *ring)
{
    int first = ring->first;
    int last = ring->last;

    return (last >= first) ? last - first : ring->buffer_size - (first - last);
}

/*
 * 返回环形buffer的容量
 */
int ring_capacity(const ring_buffer_t *ring)
{
    return ring->buffer_size - 1;
}

/*
 * 复制一个环形buffer
 */
static void byte_move(char *dest, const char *src, int n)
{
    const char *last_p = dest + n;
    while (dest < last_p) {
        *dest++ = *src++;
    }
}

/*
 * 向环形buffer写入数据
 */
int ring_write(ring_buffer_t *ring, const char *data, int size)
{
	/* 获取buffer的剩余大小 */
    int free_size = ring_capacity(ring) - ring_size(ring);
    int push_size = (size > free_size) ? free_size : size;

    /* 存储数据 */
    if (ring->first <= ring->last) {
        // Stores data at the last element of the buffer and before buffer_size
        int left_size = 0;
        int to_end = ring->buffer_size - ring->last;
        int move_size = (to_end > push_size) ? push_size : to_end;

        byte_move(&ring->buffer[ring->last], data, move_size);
        ring->last += move_size;
        ring->last &= (ring->buffer_size -1);

        left_size = push_size - move_size;
        if (left_size > 0) {
	    // Stores data before the first element
            byte_move(ring->buffer, &data[move_size], left_size);
            ring->last = left_size;
        }
    } else {
        // Stores data from last towards first
        byte_move(&ring->buffer[ring->last], data, size);
        ring->last += push_size;
    }
    return push_size;
}

/*
 * 从环形buffer读取数据
 */
int ring_read(ring_buffer_t *ring, char *buffer, int size)
{
    // Reads data
    int now_size = ring_size(ring);
    int pop_size = (size > now_size) ? now_size : size;

    if (ring->first <= ring->last) {
        byte_move(buffer, &ring->buffer[ring->first], pop_size);
        ring->first += pop_size;

    } else {
        // Gets data from first element of the buffer and before buffer_size
        int left_size = 0;
        int to_end = ring->buffer_size - ring->first;
        int move_size = (to_end > pop_size) ? pop_size : to_end;
        byte_move(buffer, &ring->buffer[ring->first], move_size);

        ring->first += move_size;
        ring->first &= (ring->buffer_size -1);

        left_size = pop_size - move_size;
        if (left_size > 0) {
	    // Gets data before the last element
            byte_move(&buffer[move_size], ring->buffer, left_size);

            ring->first = left_size;
        }
    }
    return pop_size;
}
