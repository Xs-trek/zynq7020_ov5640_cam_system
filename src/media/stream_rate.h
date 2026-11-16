/* stream_rate.h - which frames the page stream encodes (stream.max_fps).
 *
 * With a limit, a frame is encoded only when at least 1/max_fps has passed since the last
 * encoded frame; the other frames are not encoded at all, so the limit lowers the encoder's CPU
 * time and the Wi-Fi bitrate together, and fps_stream shows the limited rate.  Only the page
 * stream: capture, detection and tracking see every frame.
 *
 * The times are the frames' capture times.  A slack of STREAM_RATE_SLACK_SEC absorbs timestamp
 * jitter, so that two frames one interval apart are not taken as too close.  The camera runs at
 * 30 fps, so the encoded rate is 30/n, the highest one not above the limit: 15 and 20 encode
 * every second frame (15 fps), 10 and 14 every third (10 fps), 8 every fourth.  Within the slack,
 * a limit just under 30 (28, 29) can still encode every frame.
 *
 * Pure logic: the caller supplies the time.
 */
#ifndef CAM_STREAM_RATE_H
#define CAM_STREAM_RATE_H

#define STREAM_RATE_SLACK_SEC 0.003

typedef struct {
    double interval_sec;    /* 0: no limit */
    double last_sec;
    int have_last;
} stream_rate_t;

/* max_fps 0 means no limit. */
void stream_rate_init(stream_rate_t *rate, int max_fps);

/* The next frame is encoded whatever its time; for a new client, or after a pause. */
void stream_rate_reset(stream_rate_t *rate);

/* Returns 1 when the frame captured at time_sec is to be encoded, and then counts it as the
 * last encoded frame; 0 when it is skipped.  A time before the last one (a new stream's clock)
 * is encoded. */
int stream_rate_take(stream_rate_t *rate, double time_sec);

#endif /* CAM_STREAM_RATE_H */
