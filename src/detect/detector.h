/* detector.h - detection thread: takes the latest frame from the mailbox, runs the configured
 * backend and publishes detections into g_state for the control and network threads. */
#ifndef CAM_DETECTOR_H
#define CAM_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DETECTOR_HEALTH_RUNNING = 0,
    DETECTOR_HEALTH_RECOVERING,
    DETECTOR_HEALTH_FAILED,
    DETECTOR_HEALTH_UNAVAILABLE   /* no backend (model backend failed to load) */
} detector_health_t;

typedef struct {
    unsigned int driver_uapi;     /* 0 for a backend without a driver */
    unsigned int hw_revision;     /* 0 on R9 */
    char weight_sha256[65];       /* verified by the driver; "" when unknown */
} detector_identity_t;

typedef struct {
    detector_health_t health;
    unsigned long failures;       /* failed jobs, resets and reopens since start */
    unsigned long recoveries;
    int last_error;               /* negative errno, 0 if none */
    float fps;                    /* published detections per second */
    int direct_input;             /* jobs read capture buffers directly, without a copy */
    detector_identity_t identity;
} detector_status_t;

/* Creates and opens the configured backend.  A hardware backend that cannot be opened yet is
 * kept and retried by the thread (health "recovering"); returns -1 only when no backend can run. */
int detector_init(void);
void *detector_thread(void *arg);
void detector_close(void);

/* Thread-safe snapshot for the status interface. */
void detector_get_status(detector_status_t *status);
const char *detector_health_name(detector_health_t health);

#ifdef __cplusplus
}
#endif

#endif /* CAM_DETECTOR_H */
