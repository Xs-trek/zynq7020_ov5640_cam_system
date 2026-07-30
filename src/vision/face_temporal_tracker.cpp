#include "vision/face_temporal_tracker.h"

#include "cam_system/config.h"

#include <algorithm>
#include <cmath>

FaceTemporalBox::FaceTemporalBox()
    : x1(0), y1(0), x2(0), y2(0), score(0), label(0), track_id(-1),
      publish(1)
{
}

FaceTemporalBox::FaceTemporalBox(float x1_, float y1_, float x2_, float y2_,
                                 float score_, int label_, int track_id_)
    : x1(x1_), y1(y1_), x2(x2_), y2(y2_), score(score_), label(label_),
      track_id(track_id_), publish(1)
{
}

FaceTemporalTracker::FaceTemporalTracker() : next_id_(1), frame_index_(0)
{
}

void FaceTemporalTracker::clear()
{
    tracks_.clear();
    frame_index_ = 0;
}

static float box_iou(const FaceTemporalBox &a, const FaceTemporalBox &b)
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

static FaceTemporalBox to_absolute(const FaceTemporalBox &box,
                                   const frame_view_t &view)
{
    float left = box.x1 / (float)IMG_W;
    float right = box.x2 / (float)IMG_W;
    float top = box.y1 / (float)IMG_H;
    float bottom = box.y2 / (float)IMG_H;
    if (view.flip_x) {
        float new_left = 1.0f - right;
        right = 1.0f - left;
        left = new_left;
    }
    if (view.flip_y) {
        float new_top = 1.0f - bottom;
        bottom = 1.0f - top;
        top = new_top;
    }
    FaceTemporalBox absolute(view.roi_x + left * view.roi_w,
                             view.roi_y + top * view.roi_h,
                             view.roi_x + right * view.roi_w,
                             view.roi_y + bottom * view.roi_h,
                             box.score, box.label, box.track_id);
    absolute.publish = box.publish;
    return absolute;
}

void FaceTemporalTracker::update(std::vector<FaceTemporalBox> &boxes,
                                 const frame_view_t &view,
                                 float public_threshold,
                                 float iou_threshold,
                                 int confirm_frames,
                                 int max_misses)
{
    const double observation_time = view.capture_time_sec > 0.0 ?
        view.capture_time_sec : (double)frame_index_ / 30.0;
    const int confirmed_retain_misses = std::max(
        max_misses,
        (int)std::ceil(TRACK_TEMPORAL_RETAIN_SEC * TRACK_CONTROL_FPS));
    std::vector<bool> track_used(tracks_.size(), false);
    std::vector<bool> box_used(boxes.size(), false);
    std::vector<FaceTemporalBox> absolute_boxes;
    frame_index_++;
    absolute_boxes.reserve(boxes.size());
    for (size_t i = 0; i < boxes.size(); i++) {
        /* All detector boxes remain private tracker candidates.  Only a
         * confirmed high-score track is eligible for frontend publication. */
        boxes[i].publish = 0;
        absolute_boxes.push_back(to_absolute(boxes[i], view));
    }

    /* Cascaded association in the absolute sensor plane.  Confirmed tracks
     * consume high-score detections before tentative tracks, then low-score
     * detections may only continue a confirmed track.  Predict the bbox centre
     * with bounded velocity while retaining measured size: independently
     * extrapolating all four edges made near-face scale changes look like
     * explosive motion. */
    struct MatchCandidate {
        size_t box_index;
        size_t track_index;
        float cost;
    };
    auto predict_box = [observation_time](const Track &track) {
        FaceTemporalBox predicted = track.box;
        float dt = (float)std::max(
            0.0, std::min((double)TRACK_TEMPORAL_PREDICT_MAX_SEC,
                          observation_time - track.last_time));
        predicted.x1 += track.vx * dt;
        predicted.y1 += track.vy * dt;
        predicted.x2 += track.vx * dt;
        predicted.y2 += track.vy * dt;
        return predicted;
    };
    auto associate = [&](const std::vector<size_t> &indices,
                         int confirmation_mode) {
        std::vector<MatchCandidate> candidates;
        for (size_t n = 0; n < indices.size(); n++) {
            size_t i = indices[n];
            const FaceTemporalBox &measurement = absolute_boxes[i];
            float mw = std::max(1.0f, measurement.x2 - measurement.x1);
            float mh = std::max(1.0f, measurement.y2 - measurement.y1);
            float mcx = 0.5f * (measurement.x1 + measurement.x2);
            float mcy = 0.5f * (measurement.y1 + measurement.y2);
            for (size_t j = 0; j < tracks_.size(); j++) {
                if (track_used[j] ||
                    (confirmation_mode > 0 && !tracks_[j].confirmed) ||
                    (confirmation_mode == 0 && tracks_[j].confirmed))
                    continue;
                FaceTemporalBox predicted = predict_box(tracks_[j]);
                float pw = std::max(1.0f, predicted.x2 - predicted.x1);
                float ph = std::max(1.0f, predicted.y2 - predicted.y1);
                float pcx = 0.5f * (predicted.x1 + predicted.x2);
                float pcy = 0.5f * (predicted.y1 + predicted.y2);
                float distance = std::hypot(mcx - pcx, mcy - pcy);
                float gap = (float)std::max(
                    0.0, observation_time - tracks_[j].last_time);
                float distance_gate = std::max(
                    TRACK_EXACT_GATE_BASE_PX,
                    std::min(TRACK_EXACT_GATE_MAX_PX,
                             TRACK_EXACT_GATE_BASE_PX +
                                 TRACK_EXACT_GATE_SPEED_PX_S * gap));
                float iou = box_iou(measurement, predicted);
                float scale_ratio = mh / ph;
                if (scale_ratio < 0.40f || scale_ratio > 2.50f ||
                    (iou < iou_threshold && distance > distance_gate))
                    continue;
                float normalized_distance = distance / distance_gate;
                float scale_cost = std::fabs(std::log(scale_ratio)) +
                    0.5f * std::fabs(std::log(mw / pw));
                candidates.push_back({
                    i, j, normalized_distance + (1.0f - iou) + scale_cost
                });
            }
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const MatchCandidate &a, const MatchCandidate &b) {
                      return a.cost < b.cost;
                  });
        for (size_t n = 0; n < candidates.size(); n++) {
            size_t i = candidates[n].box_index;
            size_t j = candidates[n].track_index;
            if (box_used[i] || track_used[j])
                continue;
            Track &track = tracks_[j];
            double dt = observation_time - track.last_time;
            if (dt > 1e-3) {
                float previous_cx = 0.5f * (track.box.x1 + track.box.x2);
                float previous_cy = 0.5f * (track.box.y1 + track.box.y2);
                float current_cx = 0.5f *
                    (absolute_boxes[i].x1 + absolute_boxes[i].x2);
                float current_cy = 0.5f *
                    (absolute_boxes[i].y1 + absolute_boxes[i].y2);
                float measured_vx = (current_cx - previous_cx) / (float)dt;
                float measured_vy = (current_cy - previous_cy) / (float)dt;
                float speed = std::hypot(measured_vx, measured_vy);
                float blend = track.hits > 1 ?
                    TRACK_TEMPORAL_VELOCITY_BLEND : 1.0f;
                float keep = 1.0f - blend;

                if (speed > TRACK_TEMPORAL_MAX_SPEED_PX_S) {
                    float scale = TRACK_TEMPORAL_MAX_SPEED_PX_S / speed;
                    measured_vx *= scale;
                    measured_vy *= scale;
                }
                track.vx = keep * track.vx + blend * measured_vx;
                track.vy = keep * track.vy + blend * measured_vy;
            }
            if (track.misses > 0 && !track.confirmed)
                track.hits = 0;
            track.box = absolute_boxes[i];
            track.last_time = observation_time;
            track.hits++;
            track.misses = 0;
            if (track.hits >= confirm_frames)
                track.confirmed = 1;
            boxes[i].track_id = track.id;
            box_used[i] = true;
            track_used[j] = true;
        }
    };

    std::vector<size_t> high_indices;
    std::vector<size_t> low_indices;
    for (size_t i = 0; i < boxes.size(); i++) {
        if (boxes[i].score >= public_threshold)
            high_indices.push_back(i);
        else
            low_indices.push_back(i);
    }
    associate(high_indices, 1);
    associate(high_indices, 0);
    associate(low_indices, 1);

    /* Only unmatched high-score detections may start a new public identity. */
    for (size_t n = 0; n < high_indices.size(); n++) {
        size_t i = high_indices[n];
        if (box_used[i])
            continue;
        Track track = {
            absolute_boxes[i], 0.0f, 0.0f,
            observation_time, 1, 0, next_id_++,
            confirm_frames <= 1 ? 1 : 0
        };
        track.box.track_id = track.id;
        boxes[i].track_id = track.id;
        tracks_.push_back(track);
        track_used.push_back(true);
        box_used[i] = true;
    }

    for (size_t i = 0; i < tracks_.size(); i++)
        if (!track_used[i])
            tracks_[i].misses++;
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [max_misses, confirmed_retain_misses](const Track &track) {
                           int limit = track.confirmed ?
                               confirmed_retain_misses : max_misses;
                           return track.misses > limit;
                       }),
        tracks_.end());

    for (size_t i = 0; i < boxes.size(); i++) {
        for (size_t j = 0; j < tracks_.size(); j++) {
            if (boxes[i].track_id == tracks_[j].id && tracks_[j].confirmed) {
                boxes[i].publish = boxes[i].score >= public_threshold ? 1 : 0;
                break;
            }
        }
    }
}
