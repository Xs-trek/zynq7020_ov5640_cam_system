/* backend_face_accel.cpp - PL face accelerator backend.
 *
 * One job: the accelerator reads the frame (the driver checks the PL and resets it on error),
 * integer decode and NMS (face_accel_client), then the temporal filter that confirms a face over
 * several frames before it is published.  Optional diagnostics: per-frame input/tensor summaries
 * and a one-shot repeatability self-test.
 *
 * Input path: on R10 the accelerator reads capture buffers directly (docs/V020_RELEASE_NOTES.md).
 * Everything else falls back to copying the frame through the driver: an older bitstream or
 * driver, face.zero_copy=0, a buffer that cannot be imported, or a direct path that fails.
 */
#define LOG_TAG "detector"

#include "detect/detector_backend.h"

#include "vision/face_accel_client.h"

extern "C" {
#include "cam_system/config.h"
#include "cam_system/frame_mailbox.h"
#include "common/log.h"
#include "face_accel_regs.h"
#include "platform/hal_frmbuf.h"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <time.h>
#include <unistd.h>

namespace {

struct FaceInputSummary {
    uint64_t sample_hash;
    float luma_mean;
    float luma_stddev;
    float luma_delta_mean;
    int luma_delta_max;
};

struct FaceRepeatSummary {
    int requested;
    int completed;
    int errors;
    int mismatches;
    int prior_changes;
    uint64_t first_tensor_hash;
    size_t unique_tensors;
    float top_min;
    float top_max;
    double elapsed_sum_ms;
    double elapsed_min_ms;
    double elapsed_max_ms;
};

double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

uint64_t fnv1a64(const uint8_t *data, size_t size)
{
    uint64_t hash = UINT64_C(14695981039346656037);
    const uint64_t prime = UINT64_C(1099511628211);
    for (size_t index = 0; index < size; ++index) {
        hash ^= (uint64_t)data[index];
        hash *= prime;
    }
    return hash;
}

void summarize_face_input(const uint8_t *rgb, std::vector<uint8_t> &previous_luma,
                          FaceInputSummary *summary)
{
    const int step = 8;
    const size_t sample_count = (size_t)(IMG_W / step) * (size_t)(IMG_H / step);
    const uint64_t prime = UINT64_C(1099511628211);
    uint64_t hash = UINT64_C(14695981039346656037);
    uint64_t sum = 0u;
    uint64_t sum_squared = 0u;
    uint64_t delta_sum = 0u;
    int delta_max = 0;
    size_t sample_index = 0u;
    const bool had_previous = previous_luma.size() == sample_count;

    if (!had_previous)
        previous_luma.resize(sample_count);

    for (int y = step / 2; y < IMG_H; y += step) {
        for (int x = step / 2; x < IMG_W; x += step) {
            const uint8_t *pixel = rgb + ((size_t)y * IMG_W + (size_t)x) * 3u;
            const uint8_t luma = (uint8_t)((77u * pixel[0] + 150u * pixel[1] + 29u * pixel[2] + 128u) >> 8);
            for (size_t channel = 0u; channel < 3u; ++channel) {
                hash ^= (uint64_t)pixel[channel];
                hash *= prime;
            }
            sum += luma;
            sum_squared += (uint64_t)luma * (uint64_t)luma;
            if (had_previous) {
                const int delta = abs((int)luma - (int)previous_luma[sample_index]);
                delta_sum += (uint64_t)delta;
                if (delta > delta_max)
                    delta_max = delta;
            }
            previous_luma[sample_index++] = luma;
        }
    }

    const double mean = (double)sum / (double)sample_count;
    double variance = (double)sum_squared / (double)sample_count - mean * mean;
    if (variance < 0.0)
        variance = 0.0;
    summary->sample_hash = hash;
    summary->luma_mean = (float)mean;
    summary->luma_stddev = (float)sqrt(variance);
    summary->luma_delta_mean = had_previous ? (float)((double)delta_sum / (double)sample_count) : -1.0f;
    summary->luma_delta_max = had_previous ? delta_max : -1;
}

class FaceAccelBackend : public DetectorBackend {
public:
    explicit FaceAccelBackend(const app_config_t *cfg);
    ~FaceAccelBackend() override { release(); }

    const char *name() const override { return "face_accel"; }
    bool recoverable() const override { return true; }
    int open() override;
    void close() override { release(); }
    int reset() override;
    bool reads_capture_buffers() const override { return direct_; }
    void identity(detector_identity_t *id) const override;
    int run(const DetectorJob &job, std::vector<BBox> &boxes, DetectorJobInfo *info) override;
    void clear_temporal_state() override;

private:
    void release();
    void setup_direct();
    void stop_direct();
    int run_once(const uint8_t *rgb, uint32_t frame_id, uf_face_t *faces, face_accel_metrics_t *metrics);
    int run_direct(const DetectorJob &job, uf_face_t *faces, face_accel_metrics_t *metrics);
    int verify_direct(const DetectorJob &job, uf_face_t *faces, face_accel_metrics_t *metrics);
    FaceRepeatSummary run_repeat_phase(const uint8_t *rgb, int repeat_count, bool reset_each,
                                       uint32_t frame_id_base);
    void run_repeat_diagnostic(const uint8_t *rgb);
    void log_frame_diagnostics(const DetectorJob &job, const std::vector<BBox> &boxes, int raw_count,
                               const FaceInputSummary &input, const uf_tensor_summary_t &tensor,
                               const face_accel_metrics_t &metrics);

    face_accel_client_t *client_;
    bool zero_copy_;
    bool direct_;               /* jobs read capture buffers through handles_ */
    bool direct_suspect_;       /* the last direct job failed: a reopen stays on the copy path */
    bool direct_unverified_;    /* the next direct job is compared with the copy path */
    uint32_t handles_[CAPTURE_BUF_CNT];
    float score_threshold_;
    float track_score_threshold_;
    float nms_threshold_;
    unsigned int timeout_ms_;
    int confirm_frames_;
    int max_misses_;
    float temporal_iou_;
    int diag_log_every_;
    int diag_repeat_;
    int diag_reset_repeat_;
    int diag_frame_count_;
    bool diag_repeat_done_;
    std::vector<uint8_t> diag_previous_luma_;
    FaceTemporalTracker tracker_;
};

FaceAccelBackend::FaceAccelBackend(const app_config_t *cfg)
    : client_(NULL),
      zero_copy_(cfg->face_zero_copy != 0),
      direct_(false),
      direct_suspect_(false),
      direct_unverified_(false),
      handles_(),
      score_threshold_(cfg->face_score_thresh),
      track_score_threshold_(cfg->face_track_score_thresh),
      nms_threshold_(cfg->face_nms_thresh),
      timeout_ms_((unsigned int)cfg->face_timeout_ms),
      confirm_frames_(cfg->face_confirm_frames),
      max_misses_(cfg->face_max_misses),
      temporal_iou_(cfg->face_temporal_iou),
      diag_log_every_(cfg->face_diag_log_every),
      diag_repeat_(cfg->face_diag_repeat),
      diag_reset_repeat_(cfg->face_diag_reset_repeat),
      diag_frame_count_(0),
      diag_repeat_done_(false)
{
    /* Ranges are validated by app_config. */
    if (track_score_threshold_ > score_threshold_) {
        LOG_W("tracker score %.3f exceeds public score %.3f; clamping tracker score",
              track_score_threshold_, score_threshold_);
        track_score_threshold_ = score_threshold_;
    }
    LOG_I("face thresholds public=%.3f tracker=%.3f nms=%.3f timeout=%ums "
          "confirm=%d max_misses=%d temporal_iou=%.2f",
          score_threshold_, track_score_threshold_, nms_threshold_, timeout_ms_,
          confirm_frames_, max_misses_, temporal_iou_);
    if (diag_log_every_ > 0 || diag_repeat_ > 0 || diag_reset_repeat_ > 0)
        cam_log_write(CAM_LOG_INFO, "face-diag",
                      "enabled log_every=%d steady_repeat=%d reset_repeat=%d persistence=log-only",
                      diag_log_every_, diag_repeat_, diag_reset_repeat_);
}

int FaceAccelBackend::run_once(const uint8_t *rgb, uint32_t frame_id, uf_face_t *faces,
                               face_accel_metrics_t *metrics)
{
    return face_accel_client_run(client_, rgb, FRAME_SIZE, frame_id, timeout_ms_, track_score_threshold_,
                                 nms_threshold_, faces, UF_CANDIDATE_LIMIT, metrics);
}

int FaceAccelBackend::open()
{
    int ret;

    release();
    ret = face_accel_client_open(&client_, NULL);
    if (ret < 0) {
        client_ = NULL;
        LOG_E("face accelerator open failed: %s", strerror(-ret));
        return ret;
    }

    /* The driver verified the R9 weights; two jobs on a black frame confirm the data path
     * before the device is used for detection. */
    std::vector<uint8_t> dummy(FRAME_SIZE, 0);
    for (unsigned int w = 0; w < 2; w++) {
        uf_face_t faces[UF_CANDIDATE_LIMIT];
        face_accel_metrics_t metrics;

        ret = run_once(dummy.data(), 0xFFFFFF00u + w, faces, &metrics);
        if (ret < 0) {
            LOG_E("face accelerator warmup failed: %s; closing the device", strerror(-ret));
            release();
            return ret;
        }
    }
    setup_direct();
    return 0;
}

/* Any failure here leaves detection on the copy path, which needs nothing from this. */
void FaceAccelBackend::setup_direct()
{
    if (!zero_copy_ || direct_suspect_ ||
        !(face_accel_client_caps(client_)->flags & CAM_FACE_ACCEL_CAP_DMABUF_INPUT)) {
        LOG_I("input path: copy (zero_copy=%d suspect=%d)", zero_copy_ ? 1 : 0, direct_suspect_ ? 1 : 0);
        return;
    }
    for (int i = 0; i < CAPTURE_BUF_CNT; i++) {
        const int fd = frmbuf_export_dmabuf(i);
        if (fd < 0) {
            LOG_W("input path: copy (exporting capture buffer %d: %s)", i, strerror(-fd));
            return;
        }
        const int ret = face_accel_client_import(client_, fd, &handles_[i]);
        ::close(fd);
        if (ret < 0) {
            LOG_W("input path: copy (importing capture buffer %d: %s)", i, strerror(-ret));
            return;
        }
    }
    direct_ = true;
    direct_unverified_ = true;
    frame_mailbox_set_direct(&g_state, 1);
    LOG_I("input path: direct");
}

void FaceAccelBackend::stop_direct()
{
    if (direct_) {
        frame_mailbox_set_direct(&g_state, 0);
        direct_ = false;
    }
}

void FaceAccelBackend::identity(detector_identity_t *id) const
{
    if (!client_)
        return;
    const struct cam_face_accel_caps *caps = face_accel_client_caps(client_);

    id->driver_uapi = caps->uapi_version;
    id->hw_revision = caps->hw_revision;
    if (caps->uapi_version >= 2u) {
        for (unsigned int i = 0; i < CAM_FACE_ACCEL_WEIGHT_SHA256_SIZE; i++)
            snprintf(id->weight_sha256 + 2 * i, 3, "%02x", caps->weight_sha256[i]);
    }
}

void FaceAccelBackend::release()
{
    stop_direct();
    if (client_) {
        face_accel_client_close(client_);
        client_ = NULL;
    }
}

int FaceAccelBackend::reset()
{
    return client_ ? face_accel_client_reset(client_) : -ENODEV;
}

int FaceAccelBackend::run_direct(const DetectorJob &job, uf_face_t *faces, face_accel_metrics_t *metrics)
{
    int count = face_accel_client_run_imported(client_, handles_[job.capture_idx], job.frame_seq, timeout_ms_,
                                               track_score_threshold_, nms_threshold_, faces, UF_CANDIDATE_LIMIT,
                                               metrics);

    direct_suspect_ = count < 0;
    if (count >= 0 && direct_unverified_)
        count = verify_direct(job, faces, metrics);
    return count;
}

/* The accelerator is deterministic, so the same frame through the copy path must give the same
 * tensors.  A difference means the direct path reads something else than the frame the CPU sees;
 * that cannot be told from the detections, so it is checked once on a real frame. */
int FaceAccelBackend::verify_direct(const DetectorJob &job, uf_face_t *faces, face_accel_metrics_t *metrics)
{
    const size_t size = CAM_FACE_ACCEL_OUTPUT_SIZE - FA_OUT_SCORES_OFFSET;
    const uint8_t *output = face_accel_client_last_output(client_) + FA_OUT_SCORES_OFFSET;
    const std::vector<uint8_t> direct_tensors(output, output + size);
    const int count = run_once(job.rgb, job.frame_seq, faces, metrics);

    if (count < 0)
        return count;
    direct_unverified_ = false;
    if (memcmp(direct_tensors.data(), output, size) != 0) {
        LOG_E("direct input path gives different tensors from the copy path on frame %u; "
              "staying on the copy path", job.frame_seq);
        direct_suspect_ = true;
        stop_direct();
    } else {
        LOG_I("direct input path verified on frame %u", job.frame_seq);
    }
    return count;
}

void FaceAccelBackend::clear_temporal_state()
{
    tracker_.clear();
    diag_previous_luma_.clear();
    diag_frame_count_ = 0;
}

int FaceAccelBackend::run(const DetectorJob &job, std::vector<BBox> &boxes, DetectorJobInfo *info)
{
    uf_face_t faces[UF_CANDIDATE_LIMIT];
    face_accel_metrics_t metrics = {};
    uf_tensor_summary_t tensor = {};
    FaceInputSummary input = {};
    const bool repeat_pending = !diag_repeat_done_ && (diag_repeat_ > 0 || diag_reset_repeat_ > 0);
    const bool collect = diag_log_every_ > 0 || repeat_pending;

    boxes.clear();
    if (!client_)
        return -ENODEV;
    if (collect)
        summarize_face_input(job.rgb, diag_previous_luma_, &input);

    const double t0 = now_ms();
    int count = job.capture_idx >= 0 ? run_direct(job, faces, &metrics)
                                     : run_once(job.rgb, job.frame_seq, faces, &metrics);
    info->pre_ms = 0.0;
    info->infer_ms = now_ms() - t0;
    info->post_ms = 0.0;
    if (count < 0)
        return count;
    info->hw_ms = (double)metrics.perf_cycles / 100000.0;
    if (collect && face_accel_client_summarize_last(client_, &tensor) < 0)
        return -EBADMSG;

    if (count > job.max_det)
        count = job.max_det;
    for (int i = 0; i < count; i++) {
        const uf_face_t &face = faces[i];
        const float x1 = face.x * (float)IMG_W;
        const float y1 = face.y * (float)IMG_H;
        boxes.push_back(BBox(x1, y1, x1 + face.w * (float)IMG_W, y1 + face.h * (float)IMG_H,
                             face.confidence, DETECTION_CLASS_FACE));
    }

    if (repeat_pending) {
        run_repeat_diagnostic(job.rgb);
        diag_repeat_done_ = true;
    }

    info->raw_count = (int)boxes.size();
    info->raw_max_conf = 0.0;
    for (size_t i = 0; i < boxes.size(); i++)
        info->raw_max_conf = std::max(info->raw_max_conf, (double)boxes[i].score);

    tracker_.update(boxes, job.view, score_threshold_, temporal_iou_, confirm_frames_, max_misses_);

    if (diag_log_every_ > 0 && ++diag_frame_count_ >= diag_log_every_) {
        log_frame_diagnostics(job, boxes, info->raw_count, input, tensor, metrics);
        diag_frame_count_ = 0;
    }
    return 0;
}

void FaceAccelBackend::log_frame_diagnostics(const DetectorJob &job, const std::vector<BBox> &boxes,
                                             int raw_count, const FaceInputSummary &input,
                                             const uf_tensor_summary_t &tensor,
                                             const face_accel_metrics_t &metrics)
{
    int published_count = 0;
    const BBox *lead = NULL;

    for (size_t index = 0u; index < boxes.size(); ++index) {
        if (boxes[index].publish)
            published_count++;
        if (!lead || boxes[index].score > lead->score)
            lead = &boxes[index];
    }
    const float lead_cx = lead ? 0.5f * (lead->x1 + lead->x2) / (float)IMG_W : -1.0f;
    const float lead_cy = lead ? 0.5f * (lead->y1 + lead->y2) / (float)IMG_H : -1.0f;
    const float lead_w = lead ? (lead->x2 - lead->x1) / (float)IMG_W : 0.0f;
    const float lead_h = lead ? (lead->y2 - lead->y1) / (float)IMG_H : 0.0f;

    cam_log_write(CAM_LOG_INFO, "face-diag",
                  "seq=%u cap=%.6f view=(%.4f,%.4f,%.4f,%.4f) "
                  "in_samp=%016llx y_mean=%.3f y_sd=%.3f dy_mean=%.3f dy_max=%d "
                  "tensor=%016llx top=%.6f prior=%u q=(%d,%d) boxq=(%d,%d,%d,%d) raw=%d pub=%d "
                  "lead=(%.4f,%.4f,%.4f,%.4f,%.6f,%d,%d) hw=%.3fms",
                  job.frame_seq, job.view.capture_time_sec,
                  job.view.roi_x, job.view.roi_y, job.view.roi_w, job.view.roi_h,
                  (unsigned long long)input.sample_hash, input.luma_mean, input.luma_stddev,
                  input.luma_delta_mean, input.luma_delta_max,
                  (unsigned long long)tensor.tensor_hash, tensor.top_confidence,
                  (unsigned int)tensor.top_prior_index, (int)tensor.top_background_q, (int)tensor.top_face_q,
                  (int)tensor.top_box_q[0], (int)tensor.top_box_q[1], (int)tensor.top_box_q[2],
                  (int)tensor.top_box_q[3], raw_count, published_count,
                  lead_cx, lead_cy, lead_w, lead_h, lead ? lead->score : 0.0f, lead ? lead->track_id : -1,
                  lead ? lead->publish : 0, (double)metrics.perf_cycles / 100000.0);
}

FaceRepeatSummary FaceAccelBackend::run_repeat_phase(const uint8_t *rgb, int repeat_count, bool reset_each,
                                                     uint32_t frame_id_base)
{
    FaceRepeatSummary summary = {};
    std::vector<uint64_t> unique_hashes;
    uint16_t first_prior = 0u;

    summary.requested = repeat_count;
    summary.top_min = 1.0f;
    summary.top_max = 0.0f;
    summary.elapsed_min_ms = 1.0e9;

    for (int iteration = 0; iteration < repeat_count; ++iteration) {
        uf_face_t faces[UF_CANDIDATE_LIMIT];
        face_accel_metrics_t metrics = {};
        uf_tensor_summary_t tensor = {};

        if (reset_each && face_accel_client_reset(client_) < 0) {
            summary.errors++;
            continue;
        }
        const double started_ms = now_ms();
        const int count = run_once(rgb, frame_id_base + (uint32_t)iteration, faces, &metrics);
        const double elapsed_ms = now_ms() - started_ms;
        if (count < 0 || face_accel_client_summarize_last(client_, &tensor) < 0) {
            summary.errors++;
            continue;
        }
        if (summary.completed == 0) {
            summary.first_tensor_hash = tensor.tensor_hash;
            first_prior = tensor.top_prior_index;
        } else {
            if (tensor.tensor_hash != summary.first_tensor_hash)
                summary.mismatches++;
            if (tensor.top_prior_index != first_prior)
                summary.prior_changes++;
        }
        if (std::find(unique_hashes.begin(), unique_hashes.end(), tensor.tensor_hash) == unique_hashes.end())
            unique_hashes.push_back(tensor.tensor_hash);
        summary.top_min = std::min(summary.top_min, tensor.top_confidence);
        summary.top_max = std::max(summary.top_max, tensor.top_confidence);
        summary.elapsed_sum_ms += elapsed_ms;
        summary.elapsed_min_ms = std::min(summary.elapsed_min_ms, elapsed_ms);
        summary.elapsed_max_ms = std::max(summary.elapsed_max_ms, elapsed_ms);
        summary.completed++;
    }

    summary.unique_tensors = unique_hashes.size();
    if (summary.completed == 0) {
        summary.top_min = 0.0f;
        summary.elapsed_min_ms = 0.0;
    }
    cam_log_write(CAM_LOG_INFO, "face-diag-selftest",
                  "phase=%s requested=%d completed=%d errors=%d unique=%zu mismatches=%d prior_changes=%d "
                  "first_tensor=%016llx top_min=%.6f top_max=%.6f "
                  "time_avg=%.3fms time_min=%.3fms time_max=%.3fms",
                  reset_each ? "reset-each" : "steady", summary.requested, summary.completed, summary.errors,
                  summary.unique_tensors, summary.mismatches, summary.prior_changes,
                  (unsigned long long)summary.first_tensor_hash, summary.top_min, summary.top_max,
                  summary.completed > 0 ? summary.elapsed_sum_ms / (double)summary.completed : 0.0,
                  summary.elapsed_min_ms, summary.elapsed_max_ms);
    return summary;
}

void FaceAccelBackend::run_repeat_diagnostic(const uint8_t *rgb)
{
    const uint64_t input_hash = fnv1a64(rgb, FRAME_SIZE);

    cam_log_write(CAM_LOG_INFO, "face-diag-selftest",
                  "begin input_hash=%016llx bytes=%u steady=%d reset_each=%d image_persisted=0",
                  (unsigned long long)input_hash, (unsigned int)FRAME_SIZE, diag_repeat_, diag_reset_repeat_);

    const FaceRepeatSummary steady = run_repeat_phase(rgb, diag_repeat_, false, UINT32_C(0xe0000000));
    FaceRepeatSummary reset = {};
    if (diag_reset_repeat_ > 0)
        reset = run_repeat_phase(rgb, diag_reset_repeat_, true, UINT32_C(0xe1000000));

    const bool steady_stable = diag_repeat_ == 0 ||
        (steady.completed == steady.requested && steady.errors == 0 && steady.unique_tensors == 1u);
    const bool reset_stable = diag_reset_repeat_ == 0 ||
        (reset.completed == reset.requested && reset.errors == 0 && reset.unique_tensors == 1u);
    const bool cross_match = diag_reset_repeat_ == 0 ||
        (steady.completed > 0 && reset.completed > 0 && steady.first_tensor_hash == reset.first_tensor_hash);
    const char *verdict;
    if (steady_stable && reset_stable && cross_match)
        verdict = "BIT_EXACT_STABLE";
    else if (!steady_stable && reset_stable)
        verdict = "RESET_SENSITIVE_STATE";
    else if (steady_stable && reset_stable && !cross_match)
        verdict = "RESET_CHANGES_RESULT";
    else
        verdict = "NONDETERMINISTIC_OR_IO_ERROR";

    cam_log_write(CAM_LOG_INFO, "face-diag-selftest",
                  "end verdict=%s steady_stable=%d reset_stable=%d cross_match=%d input_hash=%016llx",
                  verdict, steady_stable ? 1 : 0, reset_stable ? 1 : 0, cross_match ? 1 : 0,
                  (unsigned long long)input_hash);
}

} /* namespace */

DetectorBackend *detector_backend_face_accel_create(const app_config_t *cfg)
{
    return new FaceAccelBackend(cfg);
}
