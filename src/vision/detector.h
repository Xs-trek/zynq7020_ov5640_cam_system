#ifndef CAM_SYSTEM_DETECTOR_H
#define CAM_SYSTEM_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

void *detector_thread(void *arg);
int   detector_init(void);
void  detector_close(void);

#ifdef __cplusplus
}
#endif

#endif
