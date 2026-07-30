#ifndef FRAME_VIEW_TIMELINE_H
#define FRAME_VIEW_TIMELINE_H

#include "cam_system/app_state.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FRAME_VIEW_TIMELINE_CAPACITY 16

typedef struct {
    /* The view command was issued after this captured V4L2 sequence. */
    uint32_t issued_sequence;
    frame_view_t view;
    int valid;
} frame_view_timeline_entry_t;

typedef struct {
    frame_view_timeline_entry_t entries[FRAME_VIEW_TIMELINE_CAPACITY];
    unsigned int next;
} frame_view_timeline_t;

void frame_view_timeline_init(frame_view_timeline_t *timeline);
void frame_view_timeline_schedule(frame_view_timeline_t *timeline,
                                  uint32_t issued_sequence,
                                  const frame_view_t *view);
void frame_view_timeline_resolve(const frame_view_timeline_t *timeline,
                                 uint32_t frame_sequence,
                                 const frame_view_t *fallback,
                                 frame_view_t *resolved);

#ifdef __cplusplus
}
#endif

#endif
