#ifndef JPEG_WORKER_H
#define JPEG_WORKER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int jpeg_worker_init(void);
void jpeg_worker_request_stop(void);
void *jpeg_worker_thread(void *arg);
void jpeg_worker_close(void);

#ifdef __cplusplus
}
#endif

#endif
