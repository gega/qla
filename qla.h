/* quick & light animation format
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Gergely Gati
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef QLA_H
#define QLA_H

#include <stdint.h>

#define QLA_MAGIC0 ('q')
#define QLA_MAGIC1 ('l')
#define QLA_MAGIC2 ('a')
#define QLA_MAGIC3 ('1')

#define QLA_HEADER_LEN (4+4+2)

#ifndef QLA_ENCODE
#ifndef QLA_DECODE
#define QLA_ENCODE
#define QLA_DECODE
#endif
#endif

struct qla_rect
{
  int x, y, w, h;
};


#ifdef QLA_DECODE


#define QLAF_NEWFRAME (1L<<0)
#define QLAF_NEWRECT  (1L<<1)

#define QLA_NEWFRAME (-1)
#define QLA_NEWRECT  (-2)
#define QLA_ERROR    (-99)


struct qla_anim
{
  uint16_t width;
  uint16_t height;
  uint8_t extended;
  uint8_t flags;
  uint16_t delay;
  uint32_t pos;
  struct qla_rect rect;
  uint32_t frame_pixels;
  uint32_t rect_pixels;
  struct qli_image qli;
  uint8_t *data;
  uint32_t data_size;
};

#endif


#ifdef QLA_ENCODE

typedef void (*qla_buffer_release_cb_t)(uint32_t *);


struct qla_encode
{
  uint16_t width;
  uint16_t height;
  uint32_t *curr_frame;
  qla_buffer_release_cb_t buf_rel_cb;
  uint8_t *xor_buffer;
};

#endif

#ifdef QLA_DECODE
int qla_decode_frame(struct qla_anim *qla, uint8_t *dest, int bufsize, int *new_chunk);
typedef size_t qla_read_t(void *fd, uint8_t *buf, size_t count);
int qla_init_decode(struct qla_anim *qla, uint16_t width, uint16_t height, uint8_t *data, uint32_t data_size);
int qla_init_header(struct qla_anim *qla, uint8_t *hdr, uint32_t hdr_size, uint8_t *data, uint32_t data_size);

#endif

#ifdef QLA_ENCODE
int qla_generate_header(struct qla_encode *qla, uint8_t *data);
int qla_init_encode(struct qla_encode *qle, uint16_t width, uint16_t height, qla_buffer_release_cb_t buf_rel);
int qla_encode_frame(struct qla_encode *qle, uint32_t *rgb, uint16_t delay_ms, uint8_t *buf, size_t bufsize);

#endif

#endif

#ifdef QLA_IMPLEMENTATION

#ifdef QLA_DECODE

int qla_init_decode(struct qla_anim *qla, uint16_t width, uint16_t height, uint8_t *data, uint32_t data_size)
{
  if(NULL==qla||width==0||height==0) return(-1);
  memset(qla,0,sizeof(struct qla_anim));
  qla->extended=(qla->width>255 || qla->height>255);
  qla->data=data;
  qla->data_size=data_size;
  qla->width=width;
  qla->height=height;
  qla->flags|=QLAF_NEWFRAME;
  return(0);
}

void qla_new_chunk(struct qla_anim *qla, uint8_t *data, uint32_t data_size)
{
  if(NULL==qla||NULL==data) return;
  qla->data=data;
  qla->data_size=data_size;
  qla->pos=0;
  printf("QLA_NEW_CHUNK %d\n",data_size);
  qli_new_chunk(&qla->qli, data, data_size);
}

/*
 * decodes some part of the current rectangle. Puts the bytes to dest buffer no more than bufsize bytes.
 * it decodes pixels so the length will be on pixel boundary. (if a pixel is two bytes, it will decode 
 * even bytes) It will return the decoded bytes written in dest. This could be less than the available
 * space even if aligned with pixel witdh. When a rectangle is fully decoded, decoder stops and returns
 * to caller to let them push the rectangle to screen and the next call will start the next rectangle.
 * the current data of the rectangle (coordinates and dimensions) can be found in the struct qla_anim.
 * when the frame is fully decoded, the next call will return zero. At this time the delay field is valid
 * in the struct qla_anim struct. The caller should measure the time elapsed from the beginning of the 
 * frame and wait (delay - elapsed) milliseconds before calling this API again to go to the next frame.
 * In this case the frame_no field contains the number of the _next_ frame. If this is zero that 
 * indicates a loop, so the caller should rewind the file pointers or reset its memory pointers back to
 * the first frame.
 * 
 */
 // buffer handling: use limited ram buffer
 // worst case "compression" size for X output pixels: 2 byte delay + 4 byte rect (8 if ext) + size * 1.33
int qla_decode_frame(struct qla_anim *qla, uint8_t *dest, int bufsize, int *new_chunk)
{
  int pixel_count=0;
  
  if(QLAF_NEWFRAME==(qla->flags&QLAF_NEWFRAME))
  {
    // new frame boundary
    if(qla->pos+2 > qla->data_size)
    {
      if(NULL!=new_chunk) *new_chunk=1;
      return(0);
    }
    // clear new frame flag
    qla->flags&=~QLAF_NEWFRAME;
    // frame pixel count
    qla->frame_pixels=qla->width * qla->height;
    // update delay
    qla->delay=(((uint16_t)qla->data[qla->pos+0])<<8) | ((uint16_t)qla->data[qla->pos+1]);
    qla->pos+=2;
    qla->flags|=QLAF_NEWRECT;
    return(QLA_NEWFRAME);
  }
  if(QLAF_NEWRECT==(qla->flags&QLAF_NEWRECT))
  {
    if((qla->pos+(4*((!!qla->extended)*2))) > qla->data_size)
    {
      if(NULL!=new_chunk) *new_chunk=1;
      return(0);
    }
    // clear flag
    qla->flags&=~QLAF_NEWRECT;
    // update rect info
    if(qla->extended)
    {
      qla->rect.x=qla->data[qla->pos+0]<<8 | qla->data[qla->pos+1];
      qla->rect.y=qla->data[qla->pos+2]<<8 | qla->data[qla->pos+3];
      qla->rect.w=qla->data[qla->pos+4]<<8 | qla->data[qla->pos+5];
      qla->rect.h=qla->data[qla->pos+6]<<8 | qla->data[qla->pos+7];
      qla->pos+=8;
    }
    else
    {
      qla->rect.x=qla->data[qla->pos+0];
      qla->rect.y=qla->data[qla->pos+1];
      qla->rect.w=qla->data[qla->pos+2];
      qla->rect.h=qla->data[qla->pos+3];
      qla->pos+=4;
    }
    qla->rect_pixels=qla->rect.w * qla->rect.h;
    printf("  NEWRECT\n  <rect_pixels=%d -- %dx%d>\n",qla->rect_pixels,qla->rect.w,qla->rect.h);
    printf("  <qla pos=%d>\n  <size=%d>\n",qla->pos,qla->data_size - qla->pos);
    qli_init(&qla->qli, qla->rect.w, qla->rect.h, qla->width*QLI_BPP, &qla->data[qla->pos], qla->data_size - qla->pos);
    return(QLA_NEWRECT);
  }
  // decode
  pixel_count=qli_decode(&qla->qli, dest, bufsize/QLI_BPP, new_chunk);
  printf("  [pixel_count=%d]\n",pixel_count);

  // housekeeping
  qla->frame_pixels-=pixel_count;
  qla->rect_pixels-=pixel_count;
  printf("  [frame_pixels=%d]\n",qla->frame_pixels);
  printf("  [rect_pixels=%d]\n",qla->rect_pixels);
  // check if frame boundary
  if(qla->frame_pixels==0) qla->flags|=QLAF_NEWFRAME;
  // check if rect finished
  if(qla->rect_pixels==0) qla->flags|=QLAF_NEWRECT;

  return(pixel_count*QLI_BPP);
}

int qla_init_header(struct qla_anim *qla, uint8_t *hdr, uint32_t hdr_size, uint8_t *data, uint32_t data_size)
{
  if(hdr_size<QLA_HEADER_LEN||NULL==hdr||NULL==qla) return(-1);
  if(	hdr[0]!=QLA_MAGIC0
    ||	hdr[1]!=QLA_MAGIC1
    ||	hdr[2]!=QLA_MAGIC2
    ||	hdr[3]!=QLA_MAGIC3 ) return(-1);
  qla->width =hdr[4]<<8 | hdr[5];
  qla->height=hdr[6]<<8 | hdr[7];
  if(hdr[8]!=QLI_PIXEL_FORMAT) return(-1);
  if(hdr[9]!=qli_index_code[QLI_INDEX_SIZE]) return(-1);
  qla->data=data;
  qla->data_size=data_size;
  qla_init_decode(qla, qla->width, qla->height, data, data_size);
  return(0);
}

#endif

#ifdef QLA_ENCODE

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>




#define QLA_GET_24BIT_PIXEL(rgb,w,x,y) (uint32_t)((rgb)[(y)*(w)+(x)])
#define QLA_RGB32_RED(p32)   (uint8_t)(((uint32_t)p32)>>24)
#define QLA_RGB32_GREEN(p32) (uint8_t)((((uint32_t)p32)>>16)&0xff)
#define QLA_RGB32_BLUE(p32)  (uint8_t)((((uint32_t)p32)>>8)&0xff)


int qla_init_encode(struct qla_encode *qle, uint16_t width, uint16_t height, qla_buffer_release_cb_t buf_rel)
{
  if(qle==NULL || width==0 || height==0) return(-1);
  qle->width=width;
  qle->height=height;
  qle->xor_buffer=malloc(width*height);
  qle->curr_frame=NULL;
  qle->buf_rel_cb=buf_rel;
  return(0);
}

void qla_destroy_encode(struct qla_encode *qle)
{
  if(qle==NULL) return;
  if(NULL!=qle->xor_buffer) free(qle->xor_buffer);
  qle->xor_buffer=NULL;
  if(NULL!=qle->buf_rel_cb && NULL!=qle->curr_frame) qle->buf_rel_cb(qle->curr_frame);
}

int qla_generate_header(struct qla_encode *qle, uint8_t *data)
{
  if(NULL==qle || NULL==data) return(-1);
  int i=0;
  data[i++]=QLA_MAGIC0;
  data[i++]=QLA_MAGIC1;
  data[i++]=QLA_MAGIC2;
  data[i++]=QLA_MAGIC3;
  data[i++]=qle->width>>8;
  data[i++]=qle->width&0xff;
  data[i++]=qle->height>>8;
  data[i++]=qle->height&0xff;
  data[i++]=QLI_PIXEL_FORMAT;
  data[i++]=qli_index_code[QLI_INDEX_SIZE];
  return(0);
}

static void qla_largest_histogram(int *heights, int w, int row, struct qla_rect *best_rect)
{
  int stack[w + 1];
  int top = -1;
  int i = 0;

  while (i <= w)
  {
    int h = (i == w) ? 0 : heights[i];
    if (top == -1 || h >= heights[stack[top]])
    {
      stack[++top] = i++;
    }
    else
    {
      int height = heights[stack[top--]];
      int width = (top == -1) ? i : i - stack[top] - 1;
      int area = height * width;
      if (area > best_rect->w * best_rect->h)
      {
        best_rect->x = (top == -1) ? 0 : stack[top] + 1;
        best_rect->y = row - height + 1;
        best_rect->w = width;
        best_rect->h = height;
      }
    }
  }
}

static int qla_largest_rect(uint8_t *img, int w, int h, struct qla_rect *r)
{
  int *height = calloc(w, sizeof (int));
  r->w = r->h = r->x = r->y = 0;

  for (int y = 0; y < h; y++)
  {
    for (int x = 0; x < w; x++)
    {
      if (img[y * w + x] == 0) height[x]++;
      else                     height[x] = 0;
    }
    qla_largest_histogram(height, w, y, r);
  }

  free(height);
  
  return(r->w*r->h);
}

static int qla_xor(uint8_t *xor8, int width, int height, uint32_t *rgb_a, uint32_t *rgb_b)
{
  int x,y,i;
  int same;

  if(NULL==xor8 || NULL==rgb_a || NULL==rgb_b) return(-1);
  for(i=y=same=0;y<height;y++)
  {
    for(x=0;x<width;x++)
    {
      uint32_t pix_a = QLA_GET_24BIT_PIXEL(rgb_a, width, x, y);
      qli_pixel_t ppx_a = QLI_RGB_PACK( QLA_RGB32_GREEN(pix_a), QLA_RGB32_GREEN(pix_a), QLA_RGB32_BLUE(pix_a));
      uint32_t pix_b = QLA_GET_24BIT_PIXEL(rgb_b, width, x, y);
      qli_pixel_t ppx_b = QLI_RGB_PACK( QLA_RGB32_GREEN(pix_b), QLA_RGB32_GREEN(pix_b), QLA_RGB32_BLUE(pix_b));
      uint32_t xor = ppx_a ^ ppx_b;
      xor8[i++]=(xor==0?0:0xff);
      if(xor==0) same++;
    }
  }
  return(same);
}

static void qla_bytemap_fillrect(uint8_t *buf, int width, int height, int v, int x, int y, int w, int h)
{
  if(NULL==buf || x<0 || y<0 || w<=0 || h<=0 || x>=width || y>=height || (x+w)>width || (y+h)>height) return;
  if(v<=0) v=1;
  if(v>=0xff) v=0xfe;
  for(int y_=y;y_<y+h;y_++) for(int x_=x;x_<x+w;x_++) buf[y_*width+x_]=v;
}

int qla_encode_frame(struct qla_encode *qle, uint32_t *rgb, uint16_t delay_ms, uint8_t *buf, size_t bufsize)
{
  int area,val,hdr,i;
  int pos=0;
  struct qla_rect rct;
  int extended;
  uint8_t *singles;

  if(qle==NULL || NULL==rgb) return(-1);
  if(NULL==buf) bufsize = SIZE_MAX;
  if(NULL==( singles = calloc(1, qle->width*qle->height) )) return(-1);
  extended=(qle->width>255 || qle->height>255);
  hdr=(extended ? sizeof(uint16_t)*4 : sizeof(uint8_t)*4);
  // write frame header (delay after frame) -- this is where the qla_anim pointer should be set
  buf[pos++] = (delay_ms>>8) & 0xff;
  buf[pos++] =  delay_ms     & 0xff;
  // encode frame
  if(qle->curr_frame==NULL)
  {
    // first frame
    // store dirty rectangle as 0,0,width,height
    if(extended) buf[pos++] = 0;
    buf[pos++] = 0;
    if(extended) buf[pos++] = 0;
    buf[pos++] = 0;
    if(extended) buf[pos++] = qle->width>>8&0xff;
    buf[pos++] = qle->width&0xff;
    if(extended) buf[pos++] = qle->height>>8&0xff;
    buf[pos++] = qle->height&0xff;
    // store qli encoded frame
    pos+=qli_encode(&rgb[0], qle->width, qle->height, qle->width*sizeof(uint32_t), &buf[pos], bufsize-pos);
    // close dirty rects with 0,0,0,0
    for(i=0; i<hdr; i++) buf[pos++]=0;
  }
  else
  {
    // subsequent frames
    //int ref_size=
    qli_encode(&rgb[0], qle->width, qle->height, qle->width*sizeof(uint32_t), NULL, 0);
    // 1. make xor with curr_frame and rgb
    if(0>(qla_xor(qle->xor_buffer, qle->width, qle->height, qle->curr_frame, rgb))) return(-1);
    // 2. get the list of largest rectangles
    // 3. loop through rectangles
    for(val=1,area=1; area>0 && (bufsize-pos)>hdr; val++)
    {
      area=qla_largest_rect(qle->xor_buffer, qle->width, qle->height, &rct);
      qla_bytemap_fillrect(qle->xor_buffer, qle->width, qle->height, val, rct.x, rct.y, rct.w, rct.h);

      // block for XOR indicates UNCHAGED area
      // create 4 dirty rects
      struct qla_rect dirty[4];
      // 1. above
      dirty[0].x=0;
      dirty[0].y=0;
      dirty[0].w=qle->width;
      dirty[0].h=rct.y;
      if(dirty[0].w<=0 || dirty[0].h<=0) dirty[0].x=-1;
      // 2. below
      dirty[1].x=0;
      dirty[1].y=rct.y + rct.h;
      dirty[1].w=qle->width;
      dirty[1].h=qle->height - rct.h - rct.y;
      if(dirty[1].w<=0 || dirty[1].h<=0) dirty[1].x=-1;
      // 3. left
      dirty[2].x=0;
      dirty[2].y=rct.y;
      dirty[2].w=rct.x - 1;
      dirty[2].h=rct.h;
      if(dirty[2].w<=0 || dirty[2].h<=0) dirty[2].x=-1;
      // 4. right
      dirty[3].x=rct.x + rct.w;
      dirty[3].y=rct.y;
      dirty[3].w=qle->width - rct.w - rct.x - 1;
      dirty[3].h=rct.h;
      if(dirty[3].w<=0 || dirty[3].h<=0) dirty[3].x=-1;
      for(i=0; i<4; i++)
      {
        if(dirty[i].x<0) continue;
        if(extended) buf[pos++]	= (dirty[i].x >>8) &0xff;
        buf[pos++] 		=  dirty[i].x      &0xff;
        if(extended) buf[pos++]	= (dirty[i].y >>8) &0xff;
        buf[pos++] 		=  dirty[i].y      &0xff;
        if(extended) buf[pos++] = (dirty[i].w >>8) &0xff;
        buf[pos++] 		=  dirty[i].w      &0xff;
        if(extended) buf[pos++] = (dirty[i].h >>8) &0xff;
        buf[pos++] 		=  dirty[i].h      &0xff;
        pos+=qli_encode(&rgb[qle->width*dirty[i].y + dirty[i].x], dirty[i].w, dirty[i].h, qle->width*sizeof(uint32_t), &buf[pos], bufsize-pos);
      }
      break;
    }
    if(bufsize-pos<hdr) return(-1);
    // close dirty rect section
    for(i=0; i<hdr; i++) buf[pos++]=0;
    if(NULL!=qle->buf_rel_cb) qle->buf_rel_cb(qle->curr_frame);
  }

  qle->curr_frame=rgb;
  free(singles);

  return(pos);
}

#endif

#endif
