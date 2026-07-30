#ifndef FACE_TEMPORAL_TRACKER_H
#define FACE_TEMPORAL_TRACKER_H

extern "C" {
#include "cam_system/app_state.h"
}

#include <vector>

struct FaceTemporalBox {
    float x1, y1, x2, y2;
    float score;
    int label;
    int track_id;
    int publish;

    FaceTemporalBox();
    FaceTemporalBox(float x1_, float y1_, float x2_, float y2_,
                    float score_, int label_, int track_id_ = -1);
};

class FaceTemporalTracker {
public:
    FaceTemporalTracker();
    void clear();
    void update(std::vector<FaceTemporalBox> &boxes,
                const frame_view_t &view,
                float public_threshold,
                float iou_threshold,
                int confirm_frames,
                int max_misses);

private:
    struct Track {
        FaceTemporalBox box;
        float vx, vy;
        double last_time;
        int hits;
        int misses;
        int id;
        int confirmed;
    };

    std::vector<Track> tracks_;
    int next_id_;
    unsigned int frame_index_;
};

#endif
