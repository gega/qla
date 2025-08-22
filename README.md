# Quick & Light Animation Format

The format is using the QLI codec: https://github.com/gega/qli and the assumption is that the previous content of the display
is there while we are decoding the subsequent frame. This is the behavior of the ST7789 LCD driver IC family.

---

## Features

- **Streaming-friendly** - designed for continuous data processing without stalls
- **Optimized for embedded MCUs** - specifically for ST7789 SPI family but works for generic cases too
- **Assymetric design** - decoder for low specs embedded, encoder for generic PC
- **Zero heap usage** - no `malloc`, no dynamic allocation, no surprises
- **Minimal RAM footprint** – works in constrained environments
- **Deterministic performance** – predictable execution, no hidden blocking
- **Configurable buffer sizes** – adapt to your application’s needs
- **Portable C implementation** – no platform-specific dependencies
- **Robust error handling** – built-in status codes for reliability

## Decoder API

### `int qla_init_decode(struct qla_anim *qla, uint16_t width, uint16_t height, uint8_t *data, uint32_t data_size);`

Initialize a decoder context with raw animation data.

* **qla** - decoder context pointer.
* **width, height** - target resolution.
* **data** - pointer to the animation bitstream.
* **data\_size** - size of the animation bitstream in bytes.

---

### `int qla_init_header(struct qla_anim *qla, uint8_t *hdr, uint32_t hdr_size, uint8_t *data, uint32_t data_size);`

Initialize a decoder from a header + data split.

* **qla** - decoder context pointer.
* **hdr** - pointer to header bytes.
* **hdr\_size** - size of header.
* **data** - pointer to animation data.
* **data\_size** - size of animation data.

---

### `qla_status_t qla_decode(struct qla_anim *qla, uint8_t *dest, int bufsize);`

Decode the next portion of the animation into a pixel buffer.

* **qla** - decoder context pointer.
* **dest** - output buffer for decoded pixels.
* **bufsize** - size of `dest` in bytes.

#### Return value (`qla_status_t`)

The return packs **status flags** and **decoded bytes count**:

* **Upper 24 bits** - number of decoded bytes.
* **Lower 8 bits** - status flags:

  * `QLA_NEWFRAME` - a new frame has started.
  * `QLA_NEWRECT` - a new rectangle/window region is ready.
  * `QLA_ERROR` - an error occurred.
  * `QLA_NEWCHUNK` - input buffer is empty, new chunk needs to be filled. Caller needs to provide a new input buffer using the qla_new_chunk() call.

Use provided macros to extract byte count and flags. NOTE: multiple status values can be set at the same time!

---

## Encoder API

### `int qla_init_encode(struct qla_encode *qle, uint16_t width, uint16_t height, qla_buffer_release_cb_t buf_rel);`

Initialize an encoder context.

* **qle** - encoder context pointer.
* **width, height** - target resolution.
* **buf\_rel** - callback for buffer release.

---

### `int qla_generate_header(struct qla_encode *qla, uint8_t *data);`

Generate the file header into a user-provided buffer.

* **qla** - encoder context pointer.
* **data** - output buffer for header.

---

### `int qla_encode_frame(struct qla_encode *qle, uint32_t *rgb, uint16_t delay_ms, uint8_t *buf, size_t bufsize);`

Encode a single frame of animation.

* **qle** - encoder context pointer.
* **rgb** - input pixels (32-bit RGB format).
* **delay\_ms** - frame delay in milliseconds.
* **buf** - output buffer.
* **bufsize** - size of the output buffer.

---

## Notes

* Always check return values.
* Decoder returns are designed for **streaming use**: you may get partial output per call.
* Encoders require buffers large enough for encoded frames + headers.

## Demo

![demo](VID_20250821_110954219.gif)
