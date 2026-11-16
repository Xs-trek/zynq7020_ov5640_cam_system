/* backend_cpu.cpp - CPU model candidates kept for board comparison: YOLOv6Lite-S (ncnn),
 * UltraFace RFB-320 (ncnn), YuNet (libfacedetection) and SCRFD (ncnn).
 *
 * Built only with CAM_SYSTEM_ENABLE_CPU_DETECTORS=ON; the product image uses the PL backend.
 */
#define LOG_TAG "detector"

#include "detect/detector_backend.h"

#include "ncnn/mat.h"
#include "ncnn/net.h"
#include "UltraFace.hpp"
#include "facedetectcnn.h"

extern "C" {
#include "cam_system/config.h"
#include "common/log.h"
}

#include <algorithm>
#include <cmath>
#include <cstring>
#include <time.h>

namespace {

enum CpuModel {
    CPU_YOLOV6 = 0,
    CPU_ULTRAFACE_RFB_320,
    CPU_YUNET_320,
    CPU_YUNET_384,
    CPU_SCRFD_500M_640,
    CPU_SCRFD_1G_640,
    CPU_SCRFD_2_5G_KPS_640,
};

struct CpuSpec {
    CpuModel model;
    const char *name;
    int input_w;
    int input_h;
    const char *param_path;
    const char *bin_path;
};

const CpuSpec k_specs[] = {
    {CPU_YOLOV6, "yolov6lite_s", YOLO_INPUT_W, YOLO_INPUT_H, MODEL_PARAM_PATH, MODEL_BIN_PATH},
    {CPU_ULTRAFACE_RFB_320, "ultraface_rfb_320", 320, 240, ULTRAFACE_RFB320_PARAM_PATH, ULTRAFACE_RFB320_BIN_PATH},
    {CPU_YUNET_320, "yunet_libfacedetection_320x240", 320, 240, NULL, NULL},
    {CPU_YUNET_384, "yunet_libfacedetection_384x288", 384, 288, NULL, NULL},
    {CPU_SCRFD_500M_640, "scrfd_500m_640", 640, 640, SCRFD_500M_640_PARAM_PATH, SCRFD_500M_640_BIN_PATH},
    {CPU_SCRFD_1G_640, "scrfd_1g_640", 640, 640, SCRFD_1G_640_PARAM_PATH, SCRFD_1G_640_BIN_PATH},
    {CPU_SCRFD_2_5G_KPS_640, "scrfd_2_5g_kps_640", 640, 640, SCRFD_2_5G_KPS_640_PARAM_PATH,
     SCRFD_2_5G_KPS_640_BIN_PATH},
};

struct NameAlias {
    const char *name;
    CpuModel model;
};

const NameAlias k_aliases[] = {
    {"yolo", CPU_YOLOV6}, {"yolov6", CPU_YOLOV6},
    {"ultraface", CPU_ULTRAFACE_RFB_320}, {"ultraface_rfb_320", CPU_ULTRAFACE_RFB_320},
    {"yunet", CPU_YUNET_320}, {"yunet_320", CPU_YUNET_320}, {"yunet_320x240", CPU_YUNET_320},
    {"yunet_384", CPU_YUNET_384}, {"yunet_384x288", CPU_YUNET_384},
    {"scrfd_500m", CPU_SCRFD_500M_640}, {"scrfd_500m_640", CPU_SCRFD_500M_640},
    {"scrfd_1g", CPU_SCRFD_1G_640}, {"scrfd_1g_640", CPU_SCRFD_1G_640},
    {"scrfd_2.5g", CPU_SCRFD_2_5G_KPS_640}, {"scrfd_2_5g", CPU_SCRFD_2_5G_KPS_640},
    {"scrfd_2_5g_kps", CPU_SCRFD_2_5G_KPS_640},
};

struct ScrfdLayer {
    const char *score_blob;
    const char *bbox_blob;
    int stride;
};

double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1000000.0;
}

int clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float clamp_float(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float compute_iou(const BBox &a, const BBox &b)
{
    float x1 = std::max(a.x1, b.x1);
    float y1 = std::max(a.y1, b.y1);
    float x2 = std::min(a.x2, b.x2);
    float y2 = std::min(a.y2, b.y2);
    float inter = std::max(0.f, x2 - x1) * std::max(0.f, y2 - y1);
    float area_a = std::max(0.f, a.x2 - a.x1) * std::max(0.f, a.y2 - a.y1);
    float area_b = std::max(0.f, b.x2 - b.x1) * std::max(0.f, b.y2 - b.y1);
    return inter / (area_a + area_b - inter + 1e-6f);
}

void nms_sorted(std::vector<BBox> &bboxes, float threshold, int max_det)
{
    if (max_det <= 0) {
        bboxes.clear();
        return;
    }
    std::sort(bboxes.begin(), bboxes.end(), [](const BBox &a, const BBox &b) { return a.score > b.score; });

    std::vector<bool> removed(bboxes.size(), false);
    for (size_t i = 0; i < bboxes.size(); i++) {
        if (removed[i])
            continue;
        for (size_t j = i + 1; j < bboxes.size(); j++) {
            if (!removed[j] && compute_iou(bboxes[i], bboxes[j]) > threshold)
                removed[j] = true;
        }
    }
    std::vector<BBox> result;
    result.reserve(bboxes.size());
    for (size_t i = 0; i < bboxes.size(); i++)
        if (!removed[i])
            result.push_back(bboxes[i]);
    if ((int)result.size() > max_det)
        result.resize(max_det);
    bboxes.swap(result);
}

void resize_rgb_to_bgr(const uint8_t *src, int src_w, int src_h, uint8_t *dst, int dst_w, int dst_h)
{
    const float sx = (float)src_w / (float)dst_w;
    const float sy = (float)src_h / (float)dst_h;

    for (int y = 0; y < dst_h; y++) {
        float fy = ((float)y + 0.5f) * sy - 0.5f;
        int y0 = (int)floorf(fy);
        float wy = fy - (float)y0;
        if (y0 < 0) { y0 = 0; wy = 0.f; }
        int y1 = y0 + 1;
        if (y1 >= src_h) y1 = src_h - 1;

        for (int x = 0; x < dst_w; x++) {
            float fx = ((float)x + 0.5f) * sx - 0.5f;
            int x0 = (int)floorf(fx);
            float wx = fx - (float)x0;
            if (x0 < 0) { x0 = 0; wx = 0.f; }
            int x1 = x0 + 1;
            if (x1 >= src_w) x1 = src_w - 1;

            const uint8_t *p00 = src + (y0 * src_w + x0) * 3;
            const uint8_t *p01 = src + (y0 * src_w + x1) * 3;
            const uint8_t *p10 = src + (y1 * src_w + x0) * 3;
            const uint8_t *p11 = src + (y1 * src_w + x1) * 3;
            uint8_t *q = dst + (y * dst_w + x) * 3;

            for (int c = 0; c < 3; c++) {
                float v0 = p00[c] * (1.f - wx) + p01[c] * wx;
                float v1 = p10[c] * (1.f - wx) + p11[c] * wx;
                int v = (int)(v0 * (1.f - wy) + v1 * wy + 0.5f);
                q[2 - c] = (uint8_t)clamp_int(v, 0, 255);
            }
        }
    }
}

class CpuBackend : public DetectorBackend {
public:
    explicit CpuBackend(const CpuSpec *spec) : spec_(spec), net_(NULL), ultraface_(NULL) {}
    ~CpuBackend() override { release(); }

    const char *name() const override { return spec_->name; }
    int open() override;
    void close() override { release(); }
    int run(const DetectorJob &job, std::vector<BBox> &boxes, DetectorJobInfo *info) override;

private:
    void release();
    int load_ncnn_model();
    void detect_yolo(const DetectorJob &job, std::vector<BBox> &proposals, DetectorJobInfo *info);
    void detect_ultraface(const DetectorJob &job, std::vector<BBox> &proposals, DetectorJobInfo *info);
    void detect_yunet(const DetectorJob &job, std::vector<BBox> &proposals, DetectorJobInfo *info);
    void detect_scrfd(const DetectorJob &job, std::vector<BBox> &proposals, DetectorJobInfo *info);
    void scrfd_proposals(const ScrfdLayer &layer, const ncnn::Mat &score_blob, const ncnn::Mat &bbox_blob,
                         float threshold, std::vector<BBox> &out) const;

    const CpuSpec *spec_;
    ncnn::Net *net_;
    UltraFace *ultraface_;
    std::vector<unsigned char> yunet_buffer_;
    std::vector<uint8_t> yunet_bgr_;
};

int CpuBackend::load_ncnn_model()
{
    net_ = new ncnn::Net();
    net_->opt.use_vulkan_compute = false;
    net_->opt.num_threads = 1;
    net_->opt.use_packing_layout = true;

    LOG_I("Loading model: %s", spec_->param_path);
    if (net_->load_param(spec_->param_path) != 0) {
        LOG_E("Failed to load param: %s", spec_->param_path);
        release();
        return -ENOENT;
    }
    if (net_->load_model(spec_->bin_path) != 0) {
        LOG_E("Failed to load bin: %s", spec_->bin_path);
        release();
        return -ENOENT;
    }
    return 0;
}

int CpuBackend::open()
{
    release();
    if (spec_->model == CPU_ULTRAFACE_RFB_320) {
        ultraface_ = new UltraFace(spec_->bin_path, spec_->param_path, spec_->input_w, spec_->input_h, 1, 0.7f,
                                   YOLO_NMS_THRESH, DETECTOR_MAX_DET);
    } else if (spec_->model == CPU_YUNET_320 || spec_->model == CPU_YUNET_384) {
        yunet_buffer_.resize(FACEDETECTION_RESULT_BUFFER_SIZE);
    } else if (load_ncnn_model() < 0) {
        return -ENOENT;
    }

    LOG_I("Warming up...");
    std::vector<uint8_t> dummy(FRAME_SIZE, 0);
    std::vector<BBox> warmup;
    DetectorJob job = {};
    DetectorJobInfo info = {};

    job.capture_idx = -1;
    job.rgb = dummy.data();
    job.max_det = DETECTOR_MAX_DET;
    job.conf_thresh = DETECTOR_CONF_THRESH;
    for (int w = 0; w < 2; w++)
        run(job, warmup, &info);
    LOG_I("Model loaded and warmed up");
    return 0;
}

void CpuBackend::release()
{
    delete ultraface_;
    ultraface_ = NULL;
    delete net_;
    net_ = NULL;
    yunet_buffer_.clear();
}

int CpuBackend::run(const DetectorJob &job, std::vector<BBox> &boxes, DetectorJobInfo *info)
{
    boxes.clear();
    switch (spec_->model) {
    case CPU_YOLOV6:
        detect_yolo(job, boxes, info);
        break;
    case CPU_ULTRAFACE_RFB_320:
        detect_ultraface(job, boxes, info);
        break;
    case CPU_YUNET_320:
    case CPU_YUNET_384:
        detect_yunet(job, boxes, info);
        break;
    default:
        detect_scrfd(job, boxes, info);
        break;
    }
    info->hw_ms = 0.0;
    info->raw_count = (int)boxes.size();
    info->raw_max_conf = 0.0;
    for (size_t i = 0; i < boxes.size(); i++)
        info->raw_max_conf = std::max(info->raw_max_conf, (double)boxes[i].score);
    return 0;
}

void CpuBackend::detect_yolo(const DetectorJob &job, std::vector<BBox> &proposals, DetectorJobInfo *info)
{
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};

    double t0 = now_ms();
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(job.rgb, ncnn::Mat::PIXEL_RGB2BGR, IMG_W, IMG_H,
                                                 YOLO_INPUT_W, YOLO_INPUT_H);
    in.substract_mean_normalize(mean_vals, norm_vals);
    double t1 = now_ms();

    ncnn::Extractor ex = net_->create_extractor();
    ex.input("in0", in);
    ncnn::Mat out;
    ex.extract("out0", out);
    double t2 = now_ms();

    for (int i = 0; i < out.h; i++) {
        const float *row = out.row(i);
        float obj_conf = row[4];
        float max_cls_score = -1.f;
        int max_cls_id = -1;
        for (int c = 0; c < 80; c++) {
            if (row[5 + c] > max_cls_score) {
                max_cls_score = row[5 + c];
                max_cls_id = c;
            }
        }
        float score = obj_conf * max_cls_score;
        if (score <= job.conf_thresh)
            continue;
        if (job.enabled_count > 0 && job.enabled_count < 80) {
            bool found = false;
            for (int k = 0; k < job.enabled_count; k++)
                if (job.enabled_classes[k] == max_cls_id)
                    found = true;
            if (!found)
                continue;
        }
        float cx = row[0], cy = row[1], w = row[2], h = row[3];
        proposals.push_back(BBox(cx - w / 2.f, cy - h / 2.f, cx + w / 2.f, cy + h / 2.f, score, max_cls_id));
    }
    nms_sorted(proposals, YOLO_NMS_THRESH, job.max_det);
    double t3 = now_ms();

    info->pre_ms = t1 - t0;
    info->infer_ms = t2 - t1;
    info->post_ms = t3 - t2;
}

void CpuBackend::detect_ultraface(const DetectorJob &job, std::vector<BBox> &proposals, DetectorJobInfo *info)
{
    double t0 = now_ms();
    ncnn::Mat in = ncnn::Mat::from_pixels(job.rgb, ncnn::Mat::PIXEL_RGB, IMG_W, IMG_H);
    double t1 = now_ms();

    std::vector<FaceInfo> faces;
    ultraface_->detect(in, faces);
    double t2 = now_ms();

    for (size_t i = 0; i < faces.size(); i++) {
        const FaceInfo &f = faces[i];
        proposals.push_back(BBox(f.x1, f.y1, f.x2, f.y2, f.score, DETECTION_CLASS_FACE));
    }
    nms_sorted(proposals, YOLO_NMS_THRESH, job.max_det);
    double t3 = now_ms();

    info->pre_ms = t1 - t0;
    info->infer_ms = t2 - t1;
    info->post_ms = t3 - t2;
}

void CpuBackend::detect_yunet(const DetectorJob &job, std::vector<BBox> &proposals, DetectorJobInfo *info)
{
    yunet_bgr_.resize((size_t)spec_->input_w * (size_t)spec_->input_h * 3);

    double t0 = now_ms();
    resize_rgb_to_bgr(job.rgb, IMG_W, IMG_H, yunet_bgr_.data(), spec_->input_w, spec_->input_h);
    double t1 = now_ms();

    int *results = facedetect_cnn(yunet_buffer_.data(), yunet_bgr_.data(), spec_->input_w, spec_->input_h,
                                  spec_->input_w * 3);
    double t2 = now_ms();

    int count = results ? results[0] : 0;
    const float sx = (float)IMG_W / (float)spec_->input_w;
    const float sy = (float)IMG_H / (float)spec_->input_h;
    for (int i = 0; i < count && i < FACEDETECTION_RESULT_MAX_FACES; i++) {
        short *p = ((short *)(results + 1)) + FACEDETECTION_RESULT_STRIDE_SHORTS * i;
        proposals.push_back(BBox((float)p[1] * sx, (float)p[2] * sy, (float)(p[1] + p[3]) * sx,
                                 (float)(p[2] + p[4]) * sy, (float)p[0] / 100.0f, DETECTION_CLASS_FACE));
    }
    nms_sorted(proposals, YOLO_NMS_THRESH, job.max_det);
    double t3 = now_ms();

    info->pre_ms = t1 - t0;
    info->infer_ms = t2 - t1;
    info->post_ms = t3 - t2;
}

void CpuBackend::scrfd_proposals(const ScrfdLayer &layer, const ncnn::Mat &score_blob, const ncnn::Mat &bbox_blob,
                                 float threshold, std::vector<BBox> &out) const
{
    const int fw = spec_->input_w / layer.stride;
    const int fh = spec_->input_h / layer.stride;
    const int num_anchors = 2;

    for (int y = 0; y < fh; y++) {
        for (int x = 0; x < fw; x++) {
            for (int q = 0; q < num_anchors; q++) {
                int index = ((y * fw + x) * num_anchors) + q;
                float score = (score_blob.w == 1) ? score_blob.row(index)[0] : score_blob[index];
                if (score < threshold)
                    continue;
                const float *dist = bbox_blob.row(index);
                const float cx = (float)x * (float)layer.stride;
                const float cy = (float)y * (float)layer.stride;
                out.push_back(BBox(cx - dist[0] * (float)layer.stride, cy - dist[1] * (float)layer.stride,
                                   cx + dist[2] * (float)layer.stride, cy + dist[3] * (float)layer.stride,
                                   score, DETECTION_CLASS_FACE));
            }
        }
    }
}

void CpuBackend::detect_scrfd(const DetectorJob &job, std::vector<BBox> &proposals, DetectorJobInfo *info)
{
    const float mean_vals[3] = {127.5f, 127.5f, 127.5f};
    const float norm_vals[3] = {1 / 128.f, 1 / 128.f, 1 / 128.f};
    const float prob_threshold = 0.3f;
    /* The frame is wider than tall, so the width fills the square input. */
    const float scale = (float)spec_->input_w / (float)IMG_W;
    const int resized_w = spec_->input_w;
    const int resized_h = (int)(IMG_H * scale);

    double t0 = now_ms();
    ncnn::Mat in = ncnn::Mat::from_pixels_resize(job.rgb, ncnn::Mat::PIXEL_RGB, IMG_W, IMG_H, resized_w, resized_h);
    int wpad = spec_->input_w - resized_w;
    int hpad = spec_->input_h - resized_h;
    ncnn::Mat in_pad;
    ncnn::copy_make_border(in, in_pad, hpad / 2, hpad - hpad / 2, wpad / 2, wpad - wpad / 2,
                           ncnn::BORDER_CONSTANT, 0.f);
    in_pad.substract_mean_normalize(mean_vals, norm_vals);
    double t1 = now_ms();

    ncnn::Extractor ex = net_->create_extractor();
    ex.input("input.1", in_pad);
    static const ScrfdLayer layers[] = {
        {"score_8", "bbox_8", 8},
        {"score_16", "bbox_16", 16},
        {"score_32", "bbox_32", 32},
    };
    ncnn::Mat score_blob[3], bbox_blob[3];
    for (int i = 0; i < 3; i++) {
        ex.extract(layers[i].score_blob, score_blob[i]);
        ex.extract(layers[i].bbox_blob, bbox_blob[i]);
    }
    double t2 = now_ms();

    for (int i = 0; i < 3; i++)
        scrfd_proposals(layers[i], score_blob[i], bbox_blob[i], prob_threshold, proposals);
    for (size_t i = 0; i < proposals.size(); i++) {
        BBox &b = proposals[i];
        b.x1 = clamp_float((b.x1 - (float)(wpad / 2)) / scale, 0.f, (float)IMG_W - 1.f);
        b.y1 = clamp_float((b.y1 - (float)(hpad / 2)) / scale, 0.f, (float)IMG_H - 1.f);
        b.x2 = clamp_float((b.x2 - (float)(wpad / 2)) / scale, 0.f, (float)IMG_W - 1.f);
        b.y2 = clamp_float((b.y2 - (float)(hpad / 2)) / scale, 0.f, (float)IMG_H - 1.f);
    }
    nms_sorted(proposals, YOLO_NMS_THRESH, job.max_det);
    double t3 = now_ms();

    info->pre_ms = t1 - t0;
    info->infer_ms = t2 - t1;
    info->post_ms = t3 - t2;
}

} /* namespace */

DetectorBackend *detector_backend_cpu_create(const char *name)
{
    for (size_t i = 0; i < sizeof(k_aliases) / sizeof(k_aliases[0]); i++) {
        if (strcmp(name, k_aliases[i].name) == 0) {
            for (size_t j = 0; j < sizeof(k_specs) / sizeof(k_specs[0]); j++)
                if (k_specs[j].model == k_aliases[i].model)
                    return new CpuBackend(&k_specs[j]);
        }
    }
    return NULL;
}
