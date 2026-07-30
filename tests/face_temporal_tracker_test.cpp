#include "vision/face_temporal_tracker.h"
#include "cam_system/config.h"

#include <algorithm>
#include <cstdio>
#include <vector>

static frame_view_t make_view(float x, float y, float width, float height,
                              int scaler)
{
    frame_view_t view = {};
    view.roi_x = x;
    view.roi_y = y;
    view.roi_w = width;
    view.roi_h = height;
    view.output_w = IMG_W;
    view.output_h = IMG_H;
    view.flip_x = 1;
    view.flip_y = 1;
    view.scaler_enabled = scaler;
    return view;
}

static FaceTemporalBox make_abs_box(float center_x, float center_y,
                                    float width, float height, float score,
                                    const frame_view_t &view)
{
    float left = (center_x - width * 0.5f - view.roi_x) / view.roi_w;
    float right = (center_x + width * 0.5f - view.roi_x) / view.roi_w;
    float top = (center_y - height * 0.5f - view.roi_y) / view.roi_h;
    float bottom = (center_y + height * 0.5f - view.roi_y) / view.roi_h;

    if (view.flip_x) {
        float next_left = 1.0f - right;
        right = 1.0f - left;
        left = next_left;
    }
    if (view.flip_y) {
        float next_top = 1.0f - bottom;
        bottom = 1.0f - top;
        top = next_top;
    }
    left = std::max(0.0f, std::min(1.0f, left));
    right = std::max(0.0f, std::min(1.0f, right));
    top = std::max(0.0f, std::min(1.0f, top));
    bottom = std::max(0.0f, std::min(1.0f, bottom));
    return FaceTemporalBox(left * IMG_W, top * IMG_H,
                           right * IMG_W, bottom * IMG_H,
                           score, DETECTION_CLASS_FACE);
}

int main()
{
    FaceTemporalTracker tracker;
    frame_view_t full = make_view(0, 0, SUBSAMPLED_1X_W, SUBSAMPLED_1X_H, 1);
    int stable_id = -1;

    for (int frame = 0; frame < 3; frame++) {
        std::vector<FaceTemporalBox> boxes;
        boxes.push_back(FaceTemporalBox(288, 216, 352, 264,
                                        0.90f, DETECTION_CLASS_FACE));
        tracker.update(boxes, full, 0.80f, 0.15f, 3, 2);
        if (frame < 2) {
            if (boxes.size() != 1 || boxes[0].publish ||
                boxes[0].track_id < 0) {
                std::fprintf(stderr,
                             "tentative candidate lost/published frame=%d size=%zu pub=%d id=%d\n",
                             frame, boxes.size(),
                             boxes.empty() ? -1 : boxes[0].publish,
                             boxes.empty() ? -1 : boxes[0].track_id);
                return 1;
            }
        }
        if (frame == 2) {
            if (boxes.size() != 1 || !boxes[0].publish) {
                std::fprintf(stderr, "face was not confirmed\n");
                return 1;
            }
            stable_id = boxes[0].track_id;
        }
    }

    /* Same absolute face after a viewport move: screen box changes heavily. */
    frame_view_t crop = make_view(400, 246, IMG_W, IMG_H, 0);
    std::vector<FaceTemporalBox> low_boxes;
    low_boxes.push_back(FaceTemporalBox(318.4f, 191.4f, 449.6f, 288.6f,
                                        0.65f, DETECTION_CLASS_FACE));
    tracker.update(low_boxes, crop, 0.80f, 0.15f, 3, 2);
    if (low_boxes.size() != 1 || low_boxes[0].publish ||
        low_boxes[0].track_id != stable_id) {
        std::fprintf(stderr,
                     "low-score absolute continuation failed size=%zu pub=%d id=%d/%d\n",
                     low_boxes.size(), low_boxes.empty() ? -1 : low_boxes[0].publish,
                     low_boxes.empty() ? -1 : low_boxes[0].track_id, stable_id);
        return 1;
    }

    tracker.clear();
    std::vector<FaceTemporalBox> false_positive;
    false_positive.push_back(FaceTemporalBox(100, 100, 160, 160,
                                              0.65f, DETECTION_CLASS_FACE));
    tracker.update(false_positive, full, 0.80f, 0.15f, 3, 2);
    if (false_positive.size() != 1 || false_positive[0].publish ||
        false_positive[0].track_id != -1) {
        std::fprintf(stderr,
                     "private low-score candidate contract failed size=%zu pub=%d id=%d\n",
                     false_positive.size(),
                     false_positive.empty() ? -1 : false_positive[0].publish,
                     false_positive.empty() ? -2 : false_positive[0].track_id);
        return 1;
    }

    /* Board regression: a fast selected face produced short high-score gaps,
     * then returned as a private 0.60--0.80 candidate.  A last-box-only IoU
     * tracker deleted the confirmed source ID after two misses, fragmenting
     * the logical target into ID/-1/new-ID sequences.  Motion prediction and
     * dormant confirmed-track retention must bridge this bounded gap. */
    FaceTemporalTracker motion_tracker;
    frame_view_t moving_view = make_view(200, 180, IMG_W, IMG_H, 0);
    int motion_id = -1;
    for (int frame = 0; frame < 3; frame++) {
        std::vector<FaceTemporalBox> boxes;
        moving_view.capture_time_sec = 10.0 + frame / 30.0;
        boxes.push_back(make_abs_box(420.0f + 14.0f * frame, 430.0f,
                                     80.0f, 100.0f, 0.95f, moving_view));
        motion_tracker.update(boxes, moving_view, 0.80f, 0.15f, 3, 2);
        if (frame == 2) {
            motion_id = boxes[0].track_id;
            if (!boxes[0].publish || motion_id < 0) {
                std::fprintf(stderr,
                             "fast temporal seed was not confirmed id=%d pub=%d\n",
                             motion_id, boxes[0].publish);
                return 1;
            }
        }
    }
    for (int frame = 3; frame < 11; frame++) {
        std::vector<FaceTemporalBox> empty;
        moving_view.capture_time_sec = 10.0 + frame / 30.0;
        motion_tracker.update(empty, moving_view, 0.80f, 0.15f, 3, 2);
    }
    {
        std::vector<FaceTemporalBox> low_return;
        moving_view.capture_time_sec = 10.0 + 11.0 / 30.0;
        low_return.push_back(make_abs_box(420.0f + 14.0f * 11.0f, 430.0f,
                                          80.0f, 100.0f, 0.65f,
                                          moving_view));
        motion_tracker.update(low_return, moving_view,
                              0.80f, 0.15f, 3, 2);
        if (low_return[0].publish || low_return[0].track_id != motion_id) {
            std::fprintf(stderr,
                         "private fast return fragmented id=%d/%d pub=%d\n",
                         low_return[0].track_id, motion_id,
                         low_return[0].publish);
            return 1;
        }
    }
    {
        std::vector<FaceTemporalBox> high_return;
        moving_view.capture_time_sec = 10.4;
        high_return.push_back(make_abs_box(588.0f, 430.0f, 80.0f, 100.0f,
                                           0.95f, moving_view));
        motion_tracker.update(high_return, moving_view,
                              0.80f, 0.15f, 3, 2);
        if (!high_return[0].publish ||
            high_return[0].track_id != motion_id) {
            std::fprintf(stderr,
                         "confirmed fast return fragmented id=%d/%d pub=%d\n",
                         high_return[0].track_id, motion_id,
                         high_return[0].publish);
            return 1;
        }
    }

    /* Near-face regression: one bounded but noisy centre step must not create
     * an unbounded four-edge velocity.  After a short gap the face reverses
     * direction and changes scale; the confirmed identity must still win. */
    FaceTemporalTracker reversal_tracker;
    int reversal_id = -1;
    for (int frame = 0; frame < 4; frame++) {
        std::vector<FaceTemporalBox> boxes;
        float center_x = frame < 3 ? 400.0f + 14.0f * frame : 488.0f;
        float height = frame < 3 ? 100.0f : 125.0f;
        moving_view.capture_time_sec = 20.0 + frame / 30.0;
        boxes.push_back(make_abs_box(center_x, 430.0f,
                                     height * 0.78f, height,
                                     0.95f, moving_view));
        reversal_tracker.update(boxes, moving_view,
                                0.80f, 0.15f, 3, 2);
        if (frame == 2)
            reversal_id = boxes[0].track_id;
        if (frame >= 2 &&
            (!boxes[0].publish || boxes[0].track_id != reversal_id)) {
            std::fprintf(stderr,
                         "reversal seed fragmented frame=%d id=%d/%d pub=%d\n",
                         frame, boxes[0].track_id, reversal_id,
                         boxes[0].publish);
            return 1;
        }
    }
    for (int frame = 4; frame < 9; frame++) {
        std::vector<FaceTemporalBox> empty;
        moving_view.capture_time_sec = 20.0 + frame / 30.0;
        reversal_tracker.update(empty, moving_view,
                                0.80f, 0.15f, 3, 2);
    }
    {
        std::vector<FaceTemporalBox> reversed;
        moving_view.capture_time_sec = 20.3;
        reversed.push_back(make_abs_box(500.0f, 425.0f, 117.0f, 150.0f,
                                        0.95f, moving_view));
        reversal_tracker.update(reversed, moving_view,
                                0.80f, 0.15f, 3, 2);
        if (!reversed[0].publish || reversed[0].track_id != reversal_id) {
            std::fprintf(stderr,
                         "bounded reversal/scale fragmented id=%d/%d pub=%d\n",
                         reversed[0].track_id, reversal_id,
                         reversed[0].publish);
            return 1;
        }
    }
    {
        std::vector<FaceTemporalBox> reversed_again;
        moving_view.capture_time_sec = 20.0 + 10.0 / 30.0;
        reversed_again.push_back(make_abs_box(476.0f, 422.0f,
                                              124.8f, 160.0f,
                                              0.95f, moving_view));
        reversal_tracker.update(reversed_again, moving_view,
                                0.80f, 0.15f, 3, 2);
        if (!reversed_again[0].publish ||
            reversed_again[0].track_id != reversal_id) {
            std::fprintf(stderr,
                         "post-reversal identity fragmented id=%d/%d pub=%d\n",
                         reversed_again[0].track_id, reversal_id,
                         reversed_again[0].publish);
            return 1;
        }
    }

    std::puts("face_temporal_tracker_test: OK");
    return 0;
}
