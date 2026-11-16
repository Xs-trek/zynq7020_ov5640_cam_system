#ifndef CAM_SYSTEM_NETWORK_MJPEG_FRAMING_H
#define CAM_SYSTEM_NETWORK_MJPEG_FRAMING_H

#include "cam_system/config.h"

#include <stddef.h>

/* Multipart framing of GET /stream.
 *
 * Chromium, and so the phone app's WebView, shows a part of a multipart image only when the next
 * part's headers arrive.  The delimiter and those headers therefore follow each JPEG at once:
 * sent with the next frame, they would delay every frame by one frame period.  They go out before
 * the next frame exists, so they carry no Content-Length, and X-Frame-Seq and X-Dropped-Frames
 * describe the JPEG that has just ended.
 *
 * Never send an empty part: it crashes the Chromium renderer. */
#define MJPEG_PART_CONTENT_TYPE "Content-Type: image/jpeg\r\n"

/* Opens the first part; sent once, straight after the HTTP response headers. */
#define MJPEG_FIRST_PART_HEAD "--" MJPEG_BOUNDARY "\r\n" MJPEG_PART_CONTENT_TYPE "\r\n"

#define MJPEG_PART_TAIL_MAX 128

/* Formats what follows a JPEG: the delimiter and the next part's headers.
 * Returns the length, or -1 if it does not fit in size. */
int mjpeg_format_part_tail(char *buf, size_t size, unsigned int frame_seq,
                           unsigned int dropped_frames);

#endif /* CAM_SYSTEM_NETWORK_MJPEG_FRAMING_H */
