#ifndef JPEG_WORKER_H
#define JPEG_WORKER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int jpeg_worker_init(void);
/* Timing log period in encoded frames (0 = off).  Call after jpeg_worker_init. */
void jpeg_worker_set_log_every(int frames);
/* stream.max_fps: encode at most this many frames per second (0 = every frame; media/stream_rate.h).
 * Call after jpeg_worker_init, before the thread starts. */
void jpeg_worker_set_max_fps(int max_fps);
/* Number of connected stream clients.  With none, the worker sleeps: it takes no frames and
 * encodes nothing. */
void jpeg_worker_set_demand(int clients);
int jpeg_worker_has_demand(void);
void jpeg_worker_request_stop(void);
void *jpeg_worker_thread(void *arg);
void jpeg_worker_close(void);

#ifdef __cplusplus
}
#endif

#endif
