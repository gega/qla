#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#define QLI_ENDIAN QLI_BIG_ENDIAN
#define QLI_PIXEL_FORMAT QLI_PF_RGB444
#define QLI_SINGLETON 1
#define QLI_DEBUG 0
#define QLI_STRIDE 0
#define QLI_IMPLEMENTATION
#include "qli.h"

#define QLA_DEBUG 0
#define QLA_PIXEL_FORMAT QLI_PIXEL_FORMAT
#define QLA_HOLE 1
#define QLA_IMPLEMENTATION
#include "qla.h"

#define W 64
#define H 48
#define FRAMES 5
#define FONT_LEN 37
#define OUTBUF 17

static int failures;

#define CHECK(c, ...) do { if(!(c)) { \
                             fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); failures++; \
                           } } while(0)

static uint32_t pixel(unsigned frame, unsigned x, unsigned y)
{
  uint8_t r = (uint8_t)((x * 17u + frame * 61u) & 0xff);
  uint8_t g = (uint8_t)((y * 29u + frame * 43u) & 0xff);
  uint8_t b = (uint8_t)(((x ^ y) * 13u + frame * 97u) & 0xff);
  return ((uint32_t)r << 24) | ((uint32_t)g << 16) | ((uint32_t)b << 8);
}

static void make_frame(uint32_t *p, unsigned frame)
{
  for(unsigned y = 0; y < H; y++)
    for(unsigned x = 0; x < W; x++)p[y * W + x] = pixel(frame, x, y);
}

static int intersects(const struct qla_rect *a, const qla_rect_t *b)
{
  return !(a->x + a->w <= b->x || b->x + b->w <= a->x ||
           a->y + a->h <= b->y || b->y + b->h <= a->y);
}

static void release_frame(uint32_t *p)
{
  free(p);
}

static void test_header(void)
{
  struct qla_encode e;
  struct qla_anim d;
  qla_rect_t hole = { 13, 9, 23, 17 };
  uint8_t hdr[QLA_MAX_HEADER_LEN];

  CHECK(qla_init_encode(&e, W, H, NULL, QLAF_HOLE, &hole) == 0, "qla_init_encode(header)");
  int n = qla_generate_header(&e, hdr, FONT_LEN);
  CHECK(n == QLA_MAX_HEADER_LEN, "hole header length %d, expected %d", n, QLA_MAX_HEADER_LEN);

  int r = qla_init_header(&d, hdr, QLA_MIN_HEADER_LEN, NULL, 0);
  CHECK(r == QLA_HOLE_HEADER_LEN, "partial header requested %d extra bytes, expected %d", r, QLA_HOLE_HEADER_LEN);

  r = qla_init_header(&d, hdr, n, NULL, 0);
  CHECK(r == 0, "full header rejected: %d", r);
  CHECK(d.width == W && d.height == H, "decoded dimensions wrong");
  CHECK((d.flags & QLAF_HOLE) != 0, "QLAF_HOLE missing after decode");
  CHECK(d.hole_x == hole.x && d.hole_y == hole.y && d.hole_w == hole.w && d.hole_h == hole.h, "decoded hole differs from encoded hole");
  CHECK(d.font_len == FONT_LEN, "font_len=%u, expected %u", d.font_len, FONT_LEN);
  qla_destroy_encode(&e);
}

static void test_hole_stream(void)
{
  qla_rect_t hole = { 13, 9, 23, 17 };
  struct qla_encode e;
  struct qla_anim d;
  uint8_t hdr[QLA_MAX_HEADER_LEN];
  const size_t one_max = QLA_MAX_HEADER_LEN + W * H * 4u + 1024u;
  uint8_t *one = malloc(one_max);
  uint8_t *stream = malloc(one_max * FRAMES + 2);
  size_t stream_len = 0;

  CHECK(one && stream, "allocation failed");
  if(!one || !stream)  exit(2);

  CHECK(qla_init_encode(&e, W, H, release_frame, QLAF_HOLE, &hole) == 0, "qla_init_encode(stream)");
  int hlen = qla_generate_header(&e, hdr, FONT_LEN);
  CHECK(hlen == QLA_MAX_HEADER_LEN, "unexpected generated header length %d", hlen);

  uint32_t *first_hole = malloc((size_t)hole.w * hole.h * sizeof(*first_hole));
  CHECK(first_hole != NULL, "first_hole allocation failed");
  if(!first_hole) exit(2);

  for(unsigned f = 0; f < FRAMES; f++)
  {
    uint32_t *frame = malloc((size_t)W * H * sizeof(*frame));
    CHECK(frame != NULL, "frame allocation failed");
    if(!frame)  exit(2);
    make_frame(frame, f);     /* The source hole changes radically every frame. */

    if(f == 0)
    {
      for(unsigned y = 0; y < hole.h; y++)
        memcpy(&first_hole[y * hole.w],
               &frame[(hole.y + y) * W + hole.x],
               hole.w * sizeof(*frame));
    }

    int n = qla_encode_frame(&e, frame, (uint16_t)(100 + f), one, one_max);
    CHECK(n > 0, "encoding frame %u failed: %d", f, n);
    if(n <= 0)  exit(2);
    memcpy(stream + stream_len, one, (size_t)n);
    stream_len += (size_t)n;

    /* Encoder promises to normalize later source frames back to frame-0 hole. */
    if(f > 0)
    {
      for(unsigned y = 0; y < hole.h; y++)
        CHECK(memcmp(&frame[(hole.y + y) * W + hole.x], &first_hole[y * hole.w], hole.w * sizeof(*frame)) == 0,
              "encoder did not restore hole on source frame %u row %u", f, y);
    }
  }

  int n = qla_encode_frame(&e, NULL, 0, one, one_max);
  CHECK(n == 2, "EOS length %d, expected 2", n);
  memcpy(stream + stream_len, one, (size_t)n);
  stream_len += (size_t)n;

  CHECK(qla_init_header(&d, hdr, hlen, stream, (uint32_t)stream_len) == 0, "decoder header initialization failed");

  unsigned frame_no = 0;
  unsigned rects = 0;
  unsigned post_first_rects = 0;
  uint8_t out[OUTBUF];
  unsigned guard = 0;

  for(;;)
  {
    CHECK(++guard < 1000000u, "decoder made no progress / guard expired");
    if(guard >= 1000000u)  break;

    qla_status_t packed = qla_decode(&d, out, sizeof(out));
    int status = QLA_GET_STATUS(packed);

    if(status & QLA_NEWCHUNK)
    {
      CHECK(0, "unexpected QLA_NEWCHUNK with complete in-memory stream");
      break;
    }
    if(status & QLA_NEWFRAME)
    {
      frame_no++;
    }
    else if(status & QLA_NEWRECT)
    {
      rects++;
      if(frame_no > 1)
      {
        post_first_rects++;
        CHECK(!intersects(&d.rect, &hole), "frame %u dirty rect %d,%d %dx%d intersects hole %u,%u %ux%u",
              frame_no, d.rect.x, d.rect.y, d.rect.w, d.rect.h, hole.x, hole.y, hole.w, hole.h);
      }
    }
    if(status & QLA_EOS)  break;
  }

  CHECK(frame_no == FRAMES, "decoded %u frames, expected %u", frame_no, FRAMES);
  CHECK(rects > 0, "decoder produced no rectangles");
  CHECK(post_first_rects > 0, "test did not exercise post-first-frame rectangles");

  qla_destroy_encode(&e);
  free(first_hole);
  free(one);
  free(stream);
}

int main(void)
{
  test_header();
  test_hole_stream();

  if(failures)
  {
    fprintf(stderr, "qla_hole_test: %d failure(s)\n", failures);
    return 1;
  }
  puts("qla_hole_test: PASS");
  return 0;
}
