#include "cam_system/frame_view_timeline.h"

#include "cam_system/config.h"

#include <limits.h>
#include <string.h>

void frame_view_timeline_init(frame_view_timeline_t *timeline)
{
    memset(timeline, 0, sizeof(*timeline));
}

void frame_view_timeline_schedule(frame_view_timeline_t *timeline,
                                  uint32_t issued_sequence,
                                  const frame_view_t *view)
{
    frame_view_timeline_entry_t *entry =
        &timeline->entries[timeline->next % FRAME_VIEW_TIMELINE_CAPACITY];

    entry->issued_sequence = issued_sequence;
    entry->view = *view;
    entry->valid = 1;
    timeline->next++;
}

void frame_view_timeline_resolve(const frame_view_timeline_t *timeline,
                                 uint32_t frame_sequence,
                                 const frame_view_t *fallback,
                                 frame_view_t *resolved)
{
    uint32_t best_age = UINT_MAX;
    int found = 0;

    *resolved = *fallback;
    for (unsigned int i = 0; i < FRAME_VIEW_TIMELINE_CAPACITY; i++) {
        const frame_view_timeline_entry_t *entry = &timeline->entries[i];
        uint32_t effective_sequence;
        int32_t signed_age;
        uint32_t age;

        if (!entry->valid)
            continue;
        effective_sequence = entry->issued_sequence +
                             (uint32_t)ISP_APPLY_DELAY_FRAMES;
        signed_age = (int32_t)(frame_sequence - effective_sequence);
        if (signed_age < 0)
            continue;
        age = (uint32_t)signed_age;
        if (!found || age < best_age) {
            best_age = age;
            *resolved = entry->view;
            found = 1;
        }
    }
}
