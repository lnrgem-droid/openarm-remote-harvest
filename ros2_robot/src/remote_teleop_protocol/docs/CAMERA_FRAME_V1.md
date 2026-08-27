# Camera frame and shared-memory contract v1

This contract covers the three Jetson-owned RGB-D cameras. The camera service is
the only process allowed to open a camera. Recorder and preview processes consume
metadata plus shared-memory buffers and never open camera devices themselves.
The Jetson also owns the two follower/harvesting arms; therefore follower feedback,
applied actions, RGB, and depth are timestamped and recorded on the Jetson. The x86
leader host receives compressed RGB previews but is not the authoritative recorder.

## Stream identity

Camera roles are configured by serial number, never by USB enumeration order:

| Role | Expected model | Serial number |
|---|---|---|
| `left_wrist` | Gemini 305 | pending physical left/right confirmation |
| `right_wrist` | Gemini 305 | pending physical left/right confirmation |
| `chest` | Gemini 335L | `CP3294Y0001E` |

The two available Gemini 305 serial numbers are `CV2L360000NR` and
`CV2L360000D9`; their left/right roles must be physically confirmed before the
configuration is frozen.

## Metadata record

Each RGB or depth buffer is announced with one metadata record containing:

| Field | Type | Meaning |
|---|---|---|
| `schema_version` | `u16` | Must equal 1 |
| `stream_role` | enum | `left_wrist`, `right_wrist`, or `chest` |
| `camera_serial` | string | Physical device serial number |
| `frameset_sequence` | `u64` | Camera-service sequence shared by RGB/depth from one frameset |
| `frame_sequence` | `u64` | Monotonic sequence for this individual stream |
| `kind` | enum | `rgb` or `depth` |
| `camera_device_timestamp` | `f64` | Unmodified device timestamp |
| `camera_clock_domain` | string | SDK-reported device clock-domain identifier |
| `camera_host_timestamp_ns` | `u64` | Jetson monotonic time at frame receipt |
| `width`, `height` | `u32` | Pixel dimensions |
| `stride_bytes` | `u32` | Bytes between adjacent image rows |
| `pixel_format` | enum | Explicit SDK/pipeline format such as `RGB8`, `MJPG`, or `Y16` |
| `depth_scale_m` | `f64` | Metres per stored depth unit; zero for RGB |
| `shm_name` | string | Shared-memory object name |
| `buffer_slot` | `u32` | Slot index within the shared-memory ring |
| `buffer_generation` | `u64` | Incremented every time the producer reuses this slot |
| `buffer_offset` | `u64` | Byte offset from the shared-memory object start |
| `buffer_bytes` | `u64` | Valid byte count for this frame |

All fields are mandatory. Unknown schema versions, formats, serials, roles, or
clock domains are rejected explicitly rather than guessed.

## Buffer lifetime

The camera service owns and writes the ring. A consumer may read a slot only while
its generation equals the announced `buffer_generation`. It checks the generation
before and after copying; a change invalidates the copy and the frame is marked
dropped. Consumers never retain raw pointers after the copy and never write to a
slot. Restarting the producer creates a new shared-memory name and resets sequences;
consumers must reopen it instead of assuming old memory remains valid.

Preview uses its own bounded latest-frame queue and may drop freely. Recording uses
a separate bounded queue and reports every overflow. A slow preview must never block
camera acquisition or recording.

## Time alignment and conversion

Device and Jetson timestamps remain in their named clock domains. Recorder receipt
time is diagnostic only. The raw OpenArm dataset retains native rates and original
timestamps. During LeRobot conversion at 30 Hz, qpos may interpolate, actions use
zero-order hold, and RGB/depth use nearest-frame matching. Maximum pairing error is
configured only after phase-0 measurement; over-limit pairs are marked or dropped,
never silently accepted.
