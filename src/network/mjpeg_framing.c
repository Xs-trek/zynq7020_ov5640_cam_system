/* mjpeg_framing.c - see mjpeg_framing.h */
#include "network/mjpeg_framing.h"

#include <stdio.h>

int mjpeg_format_part_tail(char *buf, size_t size, unsigned int frame_seq,
                           unsigned int dropped_frames)
{
    int n = snprintf(buf, size,
                     "\r\n--" MJPEG_BOUNDARY "\r\n"
                     MJPEG_PART_CONTENT_TYPE
                     "X-Frame-Seq: %u\r\n"
                     "X-Dropped-Frames: %u\r\n\r\n",
                     frame_seq, dropped_frames);

    return (n < 0 || (size_t)n >= size) ? -1 : n;
}
