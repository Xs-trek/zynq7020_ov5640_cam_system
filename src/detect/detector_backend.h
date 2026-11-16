/* detector_backend.h - interface between the detector thread and a detection backend.
 *
 * The product backend is the PL face accelerator (backend_face_accel.cpp).  The CPU model
 * candidates (backend_cpu.cpp) are built only with CAM_SYSTEM_ENABLE_CPU_DETECTORS.
 */
#ifndef CAM_DETECTOR_BACKEND_H
#define CAM_DETECTOR_BACKEND_H

#include <errno.h>
#include <stdint.h>
#include <vector>

#include "vision/face_temporal_tracker.h"

extern "C" {
#include "cam_system/app_state.h"
#include "common/app_config.h"
#include "detect/detector.h"
}

typedef FaceTemporalBox BBox;

struct DetectorJob {
    const uint8_t *rgb;             /* IMG_W x IMG_H RGB24 */
    int capture_idx;                /* rgb is this capture buffer, or -1 for a cached copy */
    unsigned int frame_seq;
    frame_view_t view;
    int max_det;
    float conf_thresh;
    const int *enabled_classes;
    int enabled_count;
};

struct DetectorJobInfo {
    double pre_ms;
    double infer_ms;
    double post_ms;
    double hw_ms;                   /* accelerator time; 0 for CPU backends */
    int raw_count;                  /* candidates before temporal filtering */
    double raw_max_conf;
};

class DetectorBackend {
public:
    virtual ~DetectorBackend() {}

    virtual const char *name() const = 0;

    /* Hardware backends can fail at run time and are reset or reopened under the policy in
     * detector_recovery.h.  Model backends fail only at open(). */
    virtual bool recoverable() const { return false; }

    /* Opens the device or loads the model, then warms it up.  0 or negative errno. */
    virtual int open() = 0;
    virtual void close() = 0;
    virtual int reset() { return -ENOTSUP; }

    /* True while jobs read capture buffers directly: the detector then leases frames from the
     * mailbox's direct channel. */
    virtual bool reads_capture_buffers() const { return false; }

    /* What the status interface reports about the hardware. */
    virtual void identity(detector_identity_t *id) const { (void)id; }

    /* Runs one job.  boxes receives pixel coordinates with publish flags set.  0 or negative
     * errno; on failure boxes is empty. */
    virtual int run(const DetectorJob &job, std::vector<BBox> &boxes, DetectorJobInfo *info) = 0;

    /* Detection was disabled or the device state was interrupted: forget temporal state so
     * faces are confirmed again before they are published. */
    virtual void clear_temporal_state() {}
};

DetectorBackend *detector_backend_face_accel_create(const app_config_t *cfg);

#ifdef CAM_SYSTEM_ENABLE_CPU_DETECTORS
/* Returns NULL for a name that is not a CPU backend. */
DetectorBackend *detector_backend_cpu_create(const char *name);
#endif

#endif /* CAM_DETECTOR_BACKEND_H */
