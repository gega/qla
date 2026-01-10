/* Copyright 2025 Gergely Gati
 *
 * gati.gergely@yahoo.com
 * github.com/gega
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS _AS IS_
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include <string.h>

#define QLI_PF_RGB565 0
#define QLI_PF_RGB888 1
#define QLI_PF_RGB444 2

// configuration value
#ifndef QLI_NOSTDIO
#define QLI_NOSTDIO 0
#endif

// configuration value
#ifndef QLI_PIXEL_FORMAT
#define QLI_PIXEL_FORMAT QLI_PF_RGB565
#endif

// configuration value
#ifndef QLI_USERDATA
#define QLI_USERDATA
#endif

// configuration value
#ifndef QLI_POSTFIX
#define QLI_POSTFIX
#endif

#ifndef QLI_SINGLETON
#define QLI_SINGLETON 0
#endif

// configuration value
#ifndef QLI_STRIDE
#define QLI_STRIDE 0
#endif

// configuration value
#ifndef QLI_DEBUG
#define QLI_DEBUG 0
#endif

#if QLI_NOSTDIO == 1
#if QLI_DEBUG > 0
#error "If QLI_DEBUG is enabled, QLI_NOSTDIO shouldn't be used!"
#endif
#endif

#if QLI_DEBUG == 0
  #define DBG_PRINT(level, fmt, ...) ((void)0)
#else
  #define DBG_PRINT(level, fmt, ...)                      \
    do {                                                  \
      if ((level) <= QLI_DEBUG)                           \
        fprintf(stderr, (fmt), ##__VA_ARGS__);            \
    } while (0)
#endif

#if QLI_PIXEL_FORMAT == QLI_PF_RGB565
 #define QLI_BPP (2)
 #define QLI_BPP2 (4)
 #define QLI_BYTE_TO_PIXEL(b) ((b)/2)
 #define QLI_PIXEL_TO_BYTE(p) ((p)*2)
 #define QLI_MIN_OUTPUT_BUFFER_SIZE (2)
#elif QLI_PIXEL_FORMAT == QLI_PF_RGB888
 #define QLI_BPP (3)
 #define QLI_BPP2 (6)
 #define QLI_BYTE_TO_PIXEL(b) ((b)/3)
 #define QLI_PIXEL_TO_BYTE(p) ((p)*3)
 #define QLI_MIN_OUTPUT_BUFFER_SIZE (3)
#elif QLI_PIXEL_FORMAT == QLI_PF_RGB444
 #define QLI_BPP (0)
 #define QLI_BPP2 (3)
 #define QLI_BYTE_TO_PIXEL(b) (((b)*2)/3)
 #define QLI_PIXEL_TO_BYTE(p) ((((p)*3)+1)/2)
 #define QLI_MIN_OUTPUT_BUFFER_SIZE (3)
#else
 #error "Unsupported pixel format"
#endif

#define QLI_LITTLE_ENDIAN (QLI_BPP-1)
#define QLI_BIG_ENDIAN    (0)

// configuration value
#ifndef QLI_ENDIAN
#define QLI_ENDIAN QLI_LITTLE_ENDIAN
#endif

#if QLI_PIXEL_FORMAT == QLI_PF_RGB444
  #if QLI_ENDIAN == QLI_LITTLE_ENDIAN
    #error "For RGB444, only QLI_BIG_ENDIAN supported"
  #endif
#endif

#ifndef QLI_ENCODE
#ifndef QLI_DECODE
#define QLI_ENCODE
#define QLI_DECODE
#endif
#endif

#ifdef QLI_ENCODE
#ifndef QLI_DECODE
#define QLI_DECODE
#endif
#endif

// configuration value
#ifndef QLI_INDEX_SIZE
#define QLI_INDEX_SIZE 6
#endif

#define QLI_MAGIC0 'q'
#define QLI_MAGIC1 'l'
#define QLI_MAGIC2 'i'
#define QLI_MAGIC3 '1'

#if QLI_NOSTDIO == 0
#include <stdio.h>
#endif

#define QLI_MAX_TOKEN_LEN ((((QLI_BPP2)+1)/2)+1)
#define QLI_REMBUFSIZ (QLI_MAX_TOKEN_LEN)

#define QLI_RF_MORE_DATA     (1<<0)
#define QLI_RF_END_OF_STREAM (1<<1)


#define QLI_CONCAT(a, b) QLI_CONCAT_INNER(a, b)
#define QLI_CONCAT_INNER(a, b) a##b
#define QLI_FUNC_NAME(name, post) QLI_CONCAT(name, post)
#define QLI_STRUCT_NAME(name, post) QLI_CONCAT(name, post)
#define QLI_TYPE(name, post) QLI_CONCAT(name, post)


#if QLI_DEBUG > 1
  #define TRACE_MAX_LINE (4000)
  static int debug_trace_buf[TRACE_MAX_LINE];
  static char debug_trace_tag[TRACE_MAX_LINE][5];
  static char debug_function[255];
  static int debug_decode_cnt;
  static int debug_emitted_total_bytes;
  #define TRACE(tag) do { \
      int line = __LINE__ ; \
      const char *func = __FUNCTION__; \
      if(strcmp(debug_function,func)!=0) { \
        memset(debug_trace_buf,0,sizeof(int)*TRACE_MAX_LINE); \
        strncpy(debug_function,func,sizeof(debug_function)-1); \
      } \
      if(TRACE_MAX_LINE > line ) { \
        strncpy(debug_trace_tag[line],tag,5); \
        debug_trace_buf[line]++; \
      } else { \
        fprintf(stderr,"TRACE_MAX_LINE too small! %d overflows!\n",line); \
        exit(1); \
      } \
    } while(0)
  #define TRACE_SUM() do { \
      int first=1; \
      for(int i=0;i<TRACE_MAX_LINE;i++) { \
        if(debug_trace_buf[i]>=1) { fprintf(stderr,"%s%s",(first?"":"->"),debug_trace_tag[i]); first=0; } \
        if(debug_trace_buf[i]>1)  fprintf(stderr,"[%d]",debug_trace_buf[i]); \
      } \
      if(first) fprintf(stderr,"<EMPTY>\n"); \
      else fprintf(stderr,"\n"); \
      memset(debug_trace_buf,0,sizeof(int)*TRACE_MAX_LINE); \
    } while(0)
  #define TRACE_HDR(qli) do { \
      int pf = QLI_PIXEL_FORMAT; \
      fprintf(stderr,"%dx%d\n",qli->width,qli->height); \
      fprintf(stderr,"type,\tno,\t size,\t pos,\trun,\tpixels_left,\tremcnt,\t    rem,\t  px,\t"); \
      if(pf==QLI_PF_RGB444) fprintf(stderr,"dest2fill,\tdest2,\t"); \
      fprintf(stderr,"emitted,\tnewchunk\n"); \
    } while(0)
#if QLI_PIXEL_FORMAT == QLI_PF_RGB444
  #define TRACE_QLI(qli, end, byte_cnt, new_chunk, emitted, ret) do { \
      if(!end) fprintf(stderr,"\nD-IN,\t%2d,\t",++debug_decode_cnt); \
      else fprintf(stderr,"D-%d,\t%2d,\t",ret,debug_decode_cnt); \
      fprintf(stderr,"%5d,\t%5d,\t%2d,\t%8d,\t%5d,\t",qli->size,qli->pos,qli->run,qli->pixels_left,qli->remcnt); \
      for(int i=0;i<QLI_REMBUFSIZ;i++) fprintf(stderr,"%02x",qli->rem[i]); \
      fprintf(stderr,",\t"); \
      fprintf(stderr,"%04x,\t%8d,\t",qli->px,qli->dest2fill); \
      for(int i=0;i<3;i++) fprintf(stderr,"%02x",qli->dest2[i]); \
      fprintf(stderr,",\t%d,\t\t%x\n",emitted,new_chunk); \
      if(end) { debug_emitted_total_bytes+=emitted; \
      float cpl=qli->width*qli->height-(2*debug_emitted_total_bytes/3.0f); \
      if((float)(qli->pixels_left)!=cpl) \
        fprintf(stderr,"*** bytes_emitted=%d pixels_left=%d expected pixels left=%.1f DIFF=%.1f\n",debug_emitted_total_bytes, \
        qli->pixels_left, cpl, cpl-qli->pixels_left); \
      } \
    } while(0)
#else
  #define TRACE_QLI(qli, end, byte_cnt, new_chunk, emitted, ret) do { \
      if(!end) fprintf(stderr,"\nD-IN,\t%2d,\t",++debug_decode_cnt); \
      else fprintf(stderr,"D-%d,\t%2d,\t",ret,debug_decode_cnt); \
      fprintf(stderr,"%5d,\t%5d,\t%2d,\t%8d,\t%5d,\t",qli->size,qli->pos,qli->run,qli->pixels_left,qli->remcnt); \
      for(int i=0;i<QLI_REMBUFSIZ;i++) fprintf(stderr,"%02x",qli->rem[i]); \
      fprintf(stderr,",\t"); \
      fprintf(stderr,"%x",qli->px); \
      fprintf(stderr,",\t%d,\t\t%x\n",emitted,new_chunk); \
      if(end) { debug_emitted_total_bytes+=emitted; \
      float cpl=qli->width*qli->height-(debug_emitted_total_bytes*QLI_BPP); \
      if((float)(qli->pixels_left)!=cpl) \
        fprintf(stderr,"*** bytes_emitted=%d pixels_left=%d expected pixels left=%.1f DIFF=%.1f\n",debug_emitted_total_bytes, \
        qli->pixels_left, cpl, cpl-qli->pixels_left); \
      } \
    } while(0)
#endif
#else
  #define TRACE_QLI(qli, end, byte_cnt, new_chunk, emitted, ret)
  #define TRACE(tag)
  #define TRACE_SUM()
  #define TRACE_HDR(qli)
#endif

struct QLI_STRUCT_NAME(qli_image, QLI_POSTFIX);
typedef struct QLI_TYPE(qli_image, QLI_POSTFIX) QLI_TYPE(qli_image_t, QLI_POSTFIX);

#ifdef QLI_DECODE
int QLI_FUNC_NAME(qli_init, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, uint16_t width, uint16_t height, uint8_t *data, int32_t size, uint16_t stride);
int QLI_FUNC_NAME(qli_init_header, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, uint8_t *header, uint16_t stride);
void QLI_FUNC_NAME(qli_rewind, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli);
int QLI_FUNC_NAME(qli_decode, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, uint8_t *dest, int32_t byte_cnt, int *new_chunk, int32_t *emitted);
void QLI_FUNC_NAME(qli_new_chunk, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, uint8_t *data, int32_t data_size);
int QLI_FUNC_NAME(qli_get_next_byte, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, int *new_chunk);
int QLI_FUNC_NAME(qli_pixels_to_bytes, QLI_POSTFIX) (int pixels);
#endif

#ifdef QLI_ENCODE
int QLI_FUNC_NAME(qli_encode,QLI_POSTFIX) (uint32_t *rgb, int width, int height, uint8_t *buf, size_t bufsize, int stride);
#if QLI_NOSTDIO == 0
int QLI_FUNC_NAME(qli_save,QLI_POSTFIX) (uint32_t *rgb, int width, int height, char *file);
#endif
#endif

#define QLI_HEADER_LEN (10)


/***************************************************************************************/

#ifdef QLI_IMPLEMENTATION

#define QLI_MAX_RUN_VALUE (63)
#define QLI_CMD_MASK ((uint8_t)(0xc0))

// opcodes from QOI
#define QLI_OP_RGB     (0xff)
#define QLI_OP_INDEX   (0x00)
#define QLI_OP_DIFF    (0x40)
#define QLI_OP_LUMA    (0x80)
#define QLI_OP_RUN     (0xc0)
#define QLI_OP_INVALID (uint8_t)(~QLI_CMD_MASK)

#ifndef ABS
#define ABS(N) ((N) < 0 ? -(N) : (N))
#endif


static const uint8_t QLI_TYPE(qli_index_code, QLI_POSTFIX) []={0,0,0,0,1,2,3,0};


#if QLI_PIXEL_FORMAT == QLI_PF_RGB565

 #define qli_pixel_t uint16_t
 /*         --------
  * 8421842184218421
  * rrrrrggggggbbbbb
  *        /\
  */
 #define QLI_R_FACTOR (8)
 #define QLI_G_FACTOR (4)
 #define QLI_B_FACTOR (8)
 #define QLI_CLAMP255(x) (((x) > 255) ? 255 : (x))
 #define QLI_RGB_PACK(r, g, b) ( \
    ((QLI_CLAMP255((r) + 4) >> 3) << 11) | \
    ((QLI_CLAMP255((g) + 2) >> 2) << 5)  | \
    ((QLI_CLAMP255((b) + 4) >> 3)) )
 #define QLI_PACK_GET_RED(rgb565)   (uint8_t)(((rgb565)>>8L)&~7L)
 #define QLI_PACK_GET_GREEN(rgb565) (uint8_t)(((rgb565)>>3L)&~3L)
 #define QLI_PACK_GET_BLUE(rgb565)  (uint8_t)(((rgb565)<<3L)&~7L)
 #define QLI_PX_GET_RED(px_)   QLI_PACK_GET_RED((qli_pixel_t)(px_))
 #define QLI_PX_GET_GREEN(px_) QLI_PACK_GET_GREEN((qli_pixel_t)(px_))
 #define QLI_PX_GET_BLUE(px_)  QLI_PACK_GET_BLUE((qli_pixel_t)(px_))
 #define QLI_COLOR_HASH(r,g,b) ((r)*3 + (g)*5 + (b)*7)
 #define QLI_RGB_TO_PX(qli,rgb) (qli)->px = (rgb)
 #define QLI_EMIT_PX(qli, dest, byem) do { \
       if(QLI_ENDIAN==QLI_BIG_ENDIAN) { \
         *(dest)++ = ((qli)->px >> 8) & 0xff; \
         *(dest)++ = ((qli)->px & 0xff); \
       } else { \
         *(dest)++ = ((qli)->px & 0xff); \
         *(dest)++ = ((qli)->px >> 8) & 0xff; \
       } \
       --(qli)->run; \
       (byem)+=QLI_BPP; \
       (qli)->pixels_left--; \
    } while(0)
 #define QLI_FRACTION_PIXELS_LEFT(qli) (0)
 #define QLI_EMIT_UNIT_BYTES (2)
 #define QLI_EMIT_PX_FLUSH(qli, dest, byem)
 
#elif QLI_PIXEL_FORMAT == QLI_PF_RGB888

 #define qli_pixel_t uint32_t
 #define QLI_R_FACTOR (1)
 #define QLI_G_FACTOR (1)
 #define QLI_B_FACTOR (1)
 #define QLI_RGB_PACK(r, g, b) ( ((qli_pixel_t)r&0xff)<<16 | ((qli_pixel_t)g&0xff)<<8 | ((qli_pixel_t)b&0xff) )
 #define QLI_PACK_GET_RED(rgb888)   (uint8_t)((rgb888>>16)&0xff)
 #define QLI_PACK_GET_GREEN(rgb888) (uint8_t)((rgb888>>8)&0xff)
 #define QLI_PACK_GET_BLUE(rgb888)  (uint8_t)((rgb888>>0)&0xff)
 #define QLI_PX_GET_RED(px_)   (QLI_PACK_GET_RED(px_))
 #define QLI_PX_GET_GREEN(px_) (QLI_PACK_GET_GREEN(px_))
 #define QLI_PX_GET_BLUE(px_)  (QLI_PACK_GET_BLUE(px_))
 // hash function is from QOI (alpha removed)
 #define QLI_COLOR_HASH(r,g,b) (r*3 + g*5 + b*7)
 #define QLI_RGB_TO_PX(qli,rgb) (qli)->px = (rgb)
 #define QLI_EMIT_PX(qli, dest, byem) do { \
       if(QLI_ENDIAN==QLI_BIG_ENDIAN) { \
         *(dest)++ = QLI_PACK_GET_RED((qli)->px); \
         *(dest)++ = QLI_PACK_GET_GREEN((qli)->px); \
         *(dest)++ = QLI_PACK_GET_BLUE((qli)->px); \
       } else { \
         *(dest)++ = QLI_PACK_GET_BLUE((qli)->px); \
         *(dest)++ = QLI_PACK_GET_GREEN((qli)->px); \
         *(dest)++ = QLI_PACK_GET_RED((qli)->px); \
       } \
       --(qli)->run; \
       (byem)+=QLI_BPP; \
       (qli)->pixels_left--; \
    } while(0)
 #define QLI_FRACTION_PIXELS_LEFT(qli) (0)
 #define QLI_EMIT_UNIT_BYTES (3)
 #define QLI_EMIT_PX_FLUSH(qli, dest, byem)

#elif QLI_PIXEL_FORMAT == QLI_PF_RGB444

 #define qli_pixel_t uint16_t
 #define QLI_R_FACTOR (16)
 #define QLI_G_FACTOR (16)
 #define QLI_B_FACTOR (16)
 #define QLI_CLAMP255(x) ((((x) > 255) ? 255 : (x))&0xff)
 #define QLI_RGB_PACK(r, g, b) (( \
    ((QLI_CLAMP255((r) + 8) >> 4) << 8)  | \
    ((QLI_CLAMP255((g) + 8) >> 4) << 4)  | \
    ((QLI_CLAMP255((b) + 8) >> 4)) ) & 0x0fff)
 #define QLI_PACK_GET_RED(rgb444)   (uint8_t)(((rgb444)>>4)&0xf0)
 #define QLI_PACK_GET_GREEN(rgb444) (uint8_t)(((rgb444)>>0)&0xf0)
 #define QLI_PACK_GET_BLUE(rgb444)  (uint8_t)(((rgb444)<<4)&0xf0)
 #define QLI_PX_GET_RED(px_)   QLI_PACK_GET_RED(px_)
 #define QLI_PX_GET_GREEN(px_) QLI_PACK_GET_GREEN(px_)
 #define QLI_PX_GET_BLUE(px_)  QLI_PACK_GET_BLUE(px_)
 #define QLI_GET_INDEX(p) (QLI_COLOR_HASH(QLI_PX_GET_RED(p),QLI_PX_GET_GREEN(p),QLI_PX_GET_BLUE(p)) % (1L<<(QLI_INDEX_SIZE)))
 #define QLI_COLOR_HASH(r,g,b) ((uint32_t)((((r>>4) * 236u) ^ ((g>>4) * 97829u) ^ ((b>>4) * 42023u))^7393913) ^ (r*1u+g*3u+b*5u))
 #define QLI_RGB_TO_PX(qli,rgb) (qli)->px = (rgb)
 #define QLI_EMIT_PX(qli, dest, byem) do { \
        if((qli)->dest2fill==0) { \
          (qli)->dest2[0]=((qli)->px>>4)&0xff; \
          (qli)->dest2[1]=((qli)->px<<4)&0xf0; \
          (qli)->dest2fill++; \
          --(qli)->run; \
          (qli)->pixels_left--; \
        } else { \
          (qli)->dest2[1]|=(((qli)->px>>8)&0x0f); \
          (qli)->dest2[2]=((qli)->px&0xff); \
          *(dest)++ = (qli)->dest2[0]; \
          *(dest)++ = (qli)->dest2[1]; \
          *(dest)++ = (qli)->dest2[2]; \
          (qli)->pixels_left--; \
          (byem)+=3; \
          (qli)->dest2fill=0; \
          --(qli)->run; \
        } \
     } while(0)
 #define QLI_FRACTION_PIXELS_LEFT(qli) ((qli)->dest2fill)
 #define QLI_EMIT_UNIT_BYTES (3)
 #define QLI_EMIT_PX_FLUSH(qli, dest, byem) do { \
       if((qli)->dest2fill!=0) { \
         *(dest)++ = (qli)->dest2[0]; \
         *(dest)++ = (qli)->dest2[1]; \
         (byem)+=2; \
         --(qli)->run; \
         (qli)->dest2fill=0; \
       } \
     } while(0)

#else
 #error "Unsupported pixel format"
#endif


struct QLI_STRUCT_NAME(qli_image, QLI_POSTFIX)
{
  uint8_t *data;
  int32_t size;
  int32_t pos;
  uint32_t run;
  int32_t pixels_left;
  uint16_t width;
  uint16_t height;
#if QLI_STRIDE == 1
  uint16_t stride;
  uint16_t x;
#endif
  qli_pixel_t px;
#if QLI_PIXEL_FORMAT == QLI_PF_RGB444
  uint8_t dest2[3]; // two pixel dest buffer
  uint8_t dest2fill;
#endif
  int8_t remcnt;
  uint8_t rem[QLI_REMBUFSIZ];
  qli_pixel_t index[1L<<QLI_INDEX_SIZE];
  QLI_USERDATA;
};



#ifndef QLI_GET_INDEX
#define QLI_GET_INDEX(p) (QLI_COLOR_HASH(QLI_PX_GET_RED(p),QLI_PX_GET_GREEN(p),QLI_PX_GET_BLUE(p)) % (1L<<(QLI_INDEX_SIZE)))
#endif
#ifndef QLI_UPDATE_INDEX
 #define QLI_UPDATE_INDEX(q,p) do { \
   int idx = QLI_GET_INDEX(p); \
   (q)->index[idx] = p; \
 } while(0)
#endif

/* rewind position pointer
 */
void QLI_FUNC_NAME(qli_rewind, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli)
{
  if(NULL!=qli)
  {
    qli->pos=0;
    memset(qli->index,0,sizeof(qli->index));
    qli->run=0;
#if QLI_STRIDE == 1
    qli->x=0;
#endif
    memset(&qli->px,0,sizeof(qli->px));
  }
}

void QLI_FUNC_NAME(qli_new_chunk, QLI_POSTFIX) ( QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, uint8_t *data, int32_t data_size)
{
  if(NULL==qli) return;
  qli->data=data;
  qli->size=data_size;
  DBG_PRINT(2,"NEWCHUNK qli->size=%d [",qli->size);
  for(int i=0;i<qli->size;i++) DBG_PRINT(2,"%s%02x",i==0?"":" ",qli->data[i]);
  DBG_PRINT(2,"]\n");
  qli->pos=0;
}

/* init user allocated qli struct
 */
int QLI_FUNC_NAME(qli_init, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, uint16_t width, uint16_t height, uint8_t *data, int32_t size, uint16_t stride)
{
  if(NULL==qli) return(-1);
  memset(qli,0,sizeof(QLI_TYPE(qli_image_t, QLI_POSTFIX)));
  qli->width = width;
  qli->height = height;
  qli->pixels_left = width * height;
#if QLI_STRIDE == 1
  qli->stride = stride;
#endif
  DBG_PRINT(2,"init: size=%d\n",size);
  QLI_FUNC_NAME(qli_rewind,QLI_POSTFIX) (qli);
  QLI_FUNC_NAME(qli_new_chunk,QLI_POSTFIX) (qli, data, size);
  TRACE_HDR(qli);
  return(0);
}

int QLI_FUNC_NAME(qli_init_header, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, uint8_t *header, uint16_t stride)
{
  if(NULL==qli||NULL==header) return(-1);
  if(   header[0]!=QLI_MAGIC0
     || header[1]!=QLI_MAGIC1
     || header[2]!=QLI_MAGIC2
     || header[3]!=QLI_MAGIC3 ) return(-1);
  header+=4;
  if(QLI_PIXEL_FORMAT!=header[4]) return(-1);
  int flags=header[5];
  if( QLI_TYPE(qli_index_code, QLI_POSTFIX) [QLI_INDEX_SIZE]!=((flags)&3)) return(-1);
  QLI_FUNC_NAME(qli_init,QLI_POSTFIX) (qli, header[0]<<8|header[1], header[2]<<8|header[3], NULL, 0, stride);
  QLI_FUNC_NAME(qli_rewind,QLI_POSTFIX) (qli);
  QLI_FUNC_NAME(qli_new_chunk,QLI_POSTFIX) (qli, NULL, 0);
  return(0);
}

int QLI_FUNC_NAME(qli_pixels_to_bytes, QLI_POSTFIX) (int pixels)
{
  return( ( QLI_BPP2 * pixels ) / 2 );
}

#ifdef QLI_DECODE

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif


#define QLI_GETNEXTBYTE_QUICK(qli,nc) ((qli)->data[(qli)->pos++])
#define QLI_GETNEXTBYTE_REM(qli,nc) QLI_FUNC_NAME(qli_get_next_byte, QLI_POSTFIX) (qli,nc)
#define QLI_GETNEXTBYTE_SLOW(qli,nc) QLI_FUNC_NAME(qli_get_next_byte, QLI_POSTFIX) (qli,nc)

#define QLI_OPLEN(op) (op==0xff ? (1+((QLI_BPP2+1)/2)) : (op&0xc0) == 0x80 ? 2 : 1 )

/*
 * return the next available byte for metadata processing
 * because of input buffer handling, the leftover buffer may contain valid data, it is opaque for the caller so we provide a way
 * to read the remaining data from the stream
 *
 * arguments:
 *  qli       - context
 *  new_chunk - request new chunk if needed
 *  
 * return:
 *  error code:
 *   positive: the byte read
 *   negative: error code (not enough data in buffer or argument error)
 */
int QLI_FUNC_NAME(qli_get_next_byte, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, int *new_chunk)
{
  int ret;

  if(!qli) return(-1);
  if(qli->remcnt==0)
  {
    if(qli->size>qli->pos)
    {
      ret=qli->data[qli->pos++];
    }
    else
    {
      if(new_chunk) *new_chunk=QLI_RF_MORE_DATA; 
      if(new_chunk) DBG_PRINT(2,"NEWCHUNK %d\n",__LINE__);
      ret=-1;
    }
  }
  else
  {
    ret=qli->rem[0];
    qli->remcnt--;
    memmove(&qli->rem[0],&qli->rem[1],qli->remcnt);
  }
  
  return(ret);
}

/* decoding bytes_cnt bytes to the supplied destination area
 *
 * RETURN: number of bytes used
 */

#define QLI_OP_PROC_DIFF(qli,d1,nc) do { \
        int r = QLI_PX_GET_RED(qli->px); \
        int g = QLI_PX_GET_GREEN(qli->px); \
        int b = QLI_PX_GET_BLUE(qli->px); \
        r += QLI_R_FACTOR * ((((d1) >> 4) & 0x03) - 2); \
        g += QLI_G_FACTOR * ((((d1) >> 2) & 0x03) - 2); \
        b += QLI_B_FACTOR * (( (d1)       & 0x03) - 2); \
        qli_pixel_t rgb = QLI_RGB_PACK(r,g,b); \
        QLI_RGB_TO_PX(qli, rgb); \
        DBG_PRINT(1,"QLI_OP_DIFF [%02x%02x%02x]\n",QLI_PACK_GET_RED(rgb),QLI_PACK_GET_GREEN(rgb),QLI_PACK_GET_BLUE(rgb)); \
        QLI_UPDATE_INDEX(qli, qli->px); \
        qli->run=1; \
    } while(0)

#define QLI_OP_PROC_LUMA(qli,d1,nc) do { \
        int r = QLI_PX_GET_RED(qli->px); \
        int g = QLI_PX_GET_GREEN(qli->px); \
        int b = QLI_PX_GET_BLUE(qli->px); \
        int d2 = QLI_NEXTBYTE(qli,nc); \
        int vg = ((d1) & 0x3f) - 32; \
        r += QLI_R_FACTOR * (vg - 8 + ((d2 >> 4) & 0x0f)); \
        g += QLI_G_FACTOR * (vg); \
        b += QLI_B_FACTOR * (vg - 8 +  (d2       & 0x0f)); \
        qli_pixel_t rgb = QLI_RGB_PACK(r,g,b); \
        QLI_RGB_TO_PX(qli, rgb); \
        DBG_PRINT(1,"QLI_OP_LUMA [%02x%02x%02x]\n",QLI_PACK_GET_RED(rgb),QLI_PACK_GET_GREEN(rgb),QLI_PACK_GET_BLUE(rgb)); \
        QLI_UPDATE_INDEX(qli, qli->px); \
        qli->run=1; \
    } while(0)

#define QLI_OP_PROC_INDEX(qli,d1,nc) do { \
        (d1)&=(1L<<QLI_INDEX_SIZE)-1; \
        qli_pixel_t rgb = (qli)->index[(d1)]; \
        QLI_RGB_TO_PX(qli, rgb); \
        DBG_PRINT(1,"QLI_OP_INDEX %d [%02x%02x%02x]\n",d1,QLI_PX_GET_RED((qli)->px),QLI_PX_GET_GREEN((qli)->px),QLI_PX_GET_BLUE((qli)->px)); \
        (qli)->run=1; \
    } while(0)

#define QLI_OP_PROC_RGB(qli,d1,nc) do { \
        qli_pixel_t rgb = 0; \
        for(int i=0;i<(QLI_BPP2+1)/2;i++) { rgb<<=8; rgb|=QLI_NEXTBYTE(qli,nc); } \
        QLI_RGB_TO_PX(qli, rgb); \
        DBG_PRINT(1,"QLI_OP_RGB [%02x%02x%02x]\n",QLI_PX_GET_RED(qli->px),QLI_PX_GET_GREEN(qli->px),QLI_PX_GET_BLUE(qli->px)); \
        QLI_UPDATE_INDEX(qli, qli->px); \
        qli->run=1; \
   } while(0)

#define QLI_OP_PROC_RUN(qli,d1,nc) do { \
        qli->run=1+(d1&0x3f); \
        DBG_PRINT(1,"QLI_OP_RUN %d [%02x%02x%02x]\n",(d1&0x3f),QLI_PX_GET_RED(qli->px),QLI_PX_GET_GREEN(qli->px),QLI_PX_GET_BLUE(qli->px)); \
   } while(0)

#define QLI_OP_PROC(qli,d1,nc) do { \
        uint8_t cm = d1 & QLI_CMD_MASK; \
        if(d1 == QLI_OP_RGB) QLI_OP_PROC_RGB(qli,d1,nc); \
        else if(QLI_OP_INDEX == cm) QLI_OP_PROC_INDEX(qli,d1,nc); \
        else if(QLI_OP_RUN == cm) QLI_OP_PROC_RUN(qli,d1,nc); \
        else if(QLI_OP_DIFF == cm) QLI_OP_PROC_DIFF(qli,d1,nc); \
        else if(QLI_OP_LUMA == cm) QLI_OP_PROC_LUMA(qli,d1,nc); \
   } while(0)

int QLI_FUNC_NAME(qli_decode, QLI_POSTFIX) (QLI_TYPE(qli_image_t, QLI_POSTFIX) *qli, uint8_t *dest, int32_t bytes_cnt, int *new_chunk, int32_t *emitted)
{
  uint8_t d1;
  uint8_t ln;
  int32_t pos_1;
  int32_t dest_left;

  TRACE_QLI(qli, 0, bytes_cnt, *new_chunk, *emitted, 0);

  dest_left = bytes_cnt;
  *new_chunk = 0;
  *emitted = 0;
  pos_1 = qli->pos;

  #undef QLI_NEXTBYTE

  //--- REM PROC
  //--- when: if remcnt>0
  //--- what: one token proc only
  //--- exit: if not enough input for one token -> NEWCHUNK
  //--- NEXTBYTE: get from rem, overflow to data
  #define QLI_NEXTBYTE(qli, nc) QLI_GETNEXTBYTE_REM(qli, nc)
  if(qli->remcnt > 0)
  {
    TRACE("rem1");
    d1 = QLI_NEXTBYTE(qli, new_chunk);
    ln = QLI_OPLEN(d1) - 1;
    if( ln > qli->remcnt + (qli->size-qli->pos) )
    {
      // rem has not enugh data for this opcode, ask for more data
      // and move the rem back and restore the opcode
      TRACE("remB");
      DBG_PRINT(2,"to rem[1] %d bytes from rem\n", qli->remcnt);
      memmove(&qli->rem[1], &qli->rem[0], qli->remcnt);
      qli->remcnt++;
      qli->rem[0]=d1;
      memcpy(&qli->rem[qli->remcnt], qli->data, (qli->size-qli->pos));
      qli->remcnt+=(qli->size-qli->pos);
      DBG_PRINT(2,"to rem[%d] %d bytes from data\n", qli->remcnt, (qli->size-qli->pos));
      *new_chunk|=QLI_RF_MORE_DATA;
      TRACE_SUM();
      TRACE_QLI(qli, 1, bytes_cnt, *new_chunk, *emitted, 0);
      return(0);
    }
    QLI_OP_PROC(qli, d1, new_chunk);
  }
  #undef QLI_NEXTBYTE

  //--- check for remaining half-pixels stuck in
  if(qli->pixels_left == 0)
  {
    TRACE("end2");
    if(QLI_FRACTION_PIXELS_LEFT(qli)) QLI_EMIT_PX_FLUSH(qli, dest, *emitted);
    *new_chunk|=QLI_RF_END_OF_STREAM;
    TRACE_SUM();
    TRACE_QLI(qli, 1, bytes_cnt, *new_chunk, *emitted, 0);
    return(qli->pos - pos_1);
  }

  //--- FAST LOOP
  //--- when: if size-pos >= QLI_MAX_TOKEN_LEN
  //--- what: loop -> token proc & pixel emit
  //--- exit: 1) dest not large enough for a full emit unit -> RET
  //---       2) all pixels emitted -> EOS
  //--- NEXTBYTE: get from data only, no overflow check
  #define QLI_NEXTBYTE(qli,nc) QLI_GETNEXTBYTE_QUICK(qli,nc)

  do
  {
    TRACE("fslL");
    while( qli->run > 0 && (dest_left - *emitted) >= QLI_EMIT_UNIT_BYTES )
    {
      TRACE("femP");
      // emit pixels with exit path 1 and 2
      QLI_EMIT_PX(qli, dest, *emitted);
    }
    if( qli->run > 0 )
    {
      TRACE("dstF");
      TRACE_SUM();
      TRACE_QLI(qli, 1, bytes_cnt, *new_chunk, *emitted, qli->pos - pos_1);
      return(qli->pos - pos_1);
    }
    if(qli->size - qli->pos < QLI_MAX_TOKEN_LEN || qli->pixels_left <= 0) break;
    d1 = QLI_NEXTBYTE(qli, new_chunk);
    QLI_OP_PROC(qli, d1, new_chunk);
  }
  while(1);
  #undef QLI_NEXTBYTE

  //--- SLOW LOOP
  //--- when: if size-pos > 0
  //--- what: loop -> token proc & pixel emit
  //--- exit: 1) out of token -> NEWCHUNK
  //          2) dest not large enough for a full emit unit -> RET
  //          3) last token is broken -> rem store -> NEWCHUNK
  //          4) all pixels emitted -> EOS
  //--- NEXTBYTE: get from data only, check overflow, if opcode isn't fully available,
  //                 store to rem and issue NEWCHUNK
  #define QLI_NEXTBYTE(qli,nc) QLI_GETNEXTBYTE_SLOW(qli,nc)
  while( qli->size - qli->pos > 0 && qli->pixels_left > 0)
  {
    TRACE("slwL");
    d1 = QLI_NEXTBYTE(qli, new_chunk);
    ln = QLI_OPLEN(d1) - 1;
    if( ln > qli->size - qli->pos )
    {
      TRACE("sREM");
      // this opcode is too large, need more data to process, copy all the bytes
      // including opcode to rem buffer and ask for more data
      qli->rem[0]=d1;
      memcpy(&qli->rem[1], &qli->data[qli->pos], MIN((qli->size - qli->pos), (sizeof(qli->rem)-1) ));
      qli->remcnt = MIN(1 + qli->size - qli->pos, sizeof(qli->rem));
      *new_chunk|=QLI_RF_MORE_DATA;
      TRACE_SUM();
      TRACE_QLI(qli, 1, bytes_cnt, *new_chunk, *emitted, qli->pos - pos_1);
      return(qli->pos - pos_1);
    }
    QLI_OP_PROC(qli, d1, new_chunk);
    while( qli->run > 0 && (dest_left - *emitted) >= QLI_EMIT_UNIT_BYTES )
    {
      TRACE("slwP");
      // emit pixels with exit path 2 and 4
      QLI_EMIT_PX(qli, dest, *emitted);
    }
    if( qli->run > 0 )
    {
      // dest too small, we need new buffer
      TRACE_SUM();
      TRACE_QLI(qli, 1, bytes_cnt, *new_chunk, *emitted, qli->pos - pos_1);
      return(qli->pos - pos_1);
    }
  }
  #undef QLI_NEXTBYTE

  //--- check for remaining half-pixels stuck in
  if(qli->pixels_left == 0)
  {
    TRACE("end3");
    if(QLI_FRACTION_PIXELS_LEFT(qli)) QLI_EMIT_PX_FLUSH(qli, dest, *emitted);
    *new_chunk|=QLI_RF_END_OF_STREAM;
    TRACE_SUM();
    TRACE_QLI(qli, 1, bytes_cnt, *new_chunk, *emitted, 0);
    return(qli->pos - pos_1);
  }

  *new_chunk|=QLI_RF_MORE_DATA;

  TRACE("end1");
  TRACE_SUM();
  TRACE_QLI(qli, 1, bytes_cnt, *new_chunk, *emitted, qli->pos - pos_1);

  return(qli->pos - pos_1);
}

#endif


#ifdef QLI_ENCODE

#include <stdlib.h>

#define QLI_RGB32_RED(p32)   (uint8_t)(((uint32_t)p32)>>24)
#define QLI_RGB32_GREEN(p32) (uint8_t)((((uint32_t)p32)>>16)&0xff)
#define QLI_RGB32_BLUE(p32)  (uint8_t)((((uint32_t)p32)>>8)&0xff)


/* encoding RGB32 buffer with RGBx format and given dmensions to buf
 *
 * if buf is NULL it will return the size for future allocation
 *
 * RETURN: the number of bytes written to buf (or would be written to if supplied)
 *         or negative on error
 */
int QLI_FUNC_NAME(qli_encode, QLI_POSTFIX) (uint32_t *rgb, int width, int height, uint8_t *buf, size_t bufsize, int stride)
{
  int res;
  QLI_TYPE(qli_image_t, QLI_POSTFIX) img;
  int pos,opos;
  int out_cnt=0;
  uint32_t pix=0;
  int i;
  qli_pixel_t px;
  qli_pixel_t ppx=0,ppx_prev=0;
  int run=0,end;

  if(rgb==NULL||width<=0||height<=0) return(-1);
  if(0!=QLI_FUNC_NAME(qli_init,QLI_POSTFIX) (&img, width, height, buf, bufsize, 0)) return(-1);
  if(stride==0) stride = width*sizeof(uint32_t);
  end=width*height;
  if(NULL==buf) bufsize=(width*height*QLI_BPP2)/2+1;
  DBG_PRINT(1,"ENC %dx%d\n",width,height);
  DBG_PRINT(2,"bufsize=%ld\n",bufsize);
  for(pos=opos=0;pos<end;pos++,ppx_prev=ppx)
  {
    pix=rgb[ (pos / width) * (stride/sizeof(uint32_t)) + (pos % width) ];
    ppx=QLI_RGB_PACK(QLI_RGB32_RED(pix), QLI_RGB32_GREEN(pix), QLI_RGB32_BLUE(pix));
    px = ppx;
    if(ppx==ppx_prev && pos<end-2)
    {
      run++;
      continue;
    }
    int idx=QLI_GET_INDEX(px);
    while(run>0)
    {
      DBG_PRINT(1,"QLI_OP_RUN %d [%02x%02x%02x]\n",((run-QLI_MAX_RUN_VALUE)>0 ? QLI_MAX_RUN_VALUE : run) - 1, QLI_PACK_GET_RED(ppx_prev),QLI_PACK_GET_GREEN(ppx_prev),QLI_PACK_GET_BLUE(ppx_prev) );
      if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_RUN|( ((run-QLI_MAX_RUN_VALUE)>0 ? QLI_MAX_RUN_VALUE : run) - 1 );
      else DBG_PRINT(2,"  buffer FULL!!! opos=%d bufsize=%ld\n",opos,bufsize);
      run-=QLI_MAX_RUN_VALUE;
      out_cnt++;
    }
    run=0;
    res = img.index[idx] != px;
    if(!res)
    {
      DBG_PRINT(1,"QLI_OP_INDEX %d [%02x%02x%02x]\n",idx, QLI_PACK_GET_RED(ppx),QLI_PACK_GET_GREEN(ppx),QLI_PACK_GET_BLUE(ppx) );
      if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_INDEX|(idx);
      else DBG_PRINT(2,"  buffer FULL!!! opos=%d bufsize=%ld\n",opos,bufsize);
      out_cnt++;
      continue;
    }
    QLI_UPDATE_INDEX(&img,px);
    signed int vr = ( QLI_PACK_GET_RED(ppx)   - QLI_PACK_GET_RED(ppx_prev)   ) / QLI_R_FACTOR;
    signed int vg = ( QLI_PACK_GET_GREEN(ppx) - QLI_PACK_GET_GREEN(ppx_prev) ) / QLI_G_FACTOR;
    signed int vb = ( QLI_PACK_GET_BLUE(ppx)  - QLI_PACK_GET_BLUE(ppx_prev)  ) / QLI_B_FACTOR;
    if( vr>-3 && vr<2 && vg>-3 && vg<2 && vb>-3 && vb<2 )
    {
      DBG_PRINT(1,"QLI_OP_DIFF [%02x%02x%02x]\n",QLI_PACK_GET_RED(ppx),QLI_PACK_GET_GREEN(ppx),QLI_PACK_GET_BLUE(ppx));
      if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_DIFF|( (vr + 2) << 4 | (vg + 2) << 2 | (vb + 2) );
      else DBG_PRINT(2,"  buffer FULL!!! opos=%d bufsize=%ld\n",opos,bufsize);
      out_cnt++;
      continue;
    }
    signed int vg_r = vr-vg;
    signed int vg_b = vb-vg;
    if( vg_r>-9 && vg_r<8 && vg>-33 && vg<32 && vg_b>-9 && vg_b<8 )
    {
      if(NULL!=buf&&opos<bufsize-1)
      {
        DBG_PRINT(1,"QLI_OP_LUMA [%02x%02x%02x]\n",QLI_PACK_GET_RED(ppx),QLI_PACK_GET_GREEN(ppx),QLI_PACK_GET_BLUE(ppx));
        buf[opos++]=QLI_OP_LUMA | (vg+32);
        buf[opos++]=(vg_r+8)<<4 | (vg_b+8);
      }
      else DBG_PRINT(2,"  buffer FULL!!! opos=%d bufsize=%ld\n",opos,bufsize);
      out_cnt+=2;
      continue;
    }
    if(NULL!=buf&&opos<bufsize-(((QLI_BPP2)+1)/2)-0)
    {
      DBG_PRINT(1,"QLI_OP_RGB [%02x%02x%02x]\n",QLI_PACK_GET_RED(ppx),QLI_PACK_GET_GREEN(ppx),QLI_PACK_GET_BLUE(ppx));
      buf[opos++]=QLI_OP_RGB;
      for(i=((QLI_BPP2+1)/2)-1;i>=0;--i) buf[opos++] = (px >> (i*8)) & 0xff;
    }
    else DBG_PRINT(2,"  buffer FULL!!! opos=%d bufsize=%ld\n",opos,bufsize);
    out_cnt+=((((QLI_BPP2)+1)/2)+1);
  }
  return(out_cnt);
}

#if QLI_NOSTDIO == 0
int QLI_FUNC_NAME(qli_save, QLI_POSTFIX) (uint32_t *rgb, int width, int height, char *file)
{
  uint8_t *buf;
  int ret=0;
  
  if(rgb==NULL||width<=0||height<=0||NULL==file) return(-1);
  int length=(width*height*QLI_BPP2)/1; // double size to make sure the "compressed" image fits in
  buf=calloc(1,length);
  ret=QLI_FUNC_NAME(qli_encode, QLI_POSTFIX) (rgb, width, height, buf, length, 0);
  if(ret>0)
  {
    FILE *f=fopen(file,"wb");
    if(NULL!=f)
    {
      fputc(QLI_MAGIC0,f);
      fputc(QLI_MAGIC1,f);
      fputc(QLI_MAGIC2,f);
      fputc(QLI_MAGIC3,f);
      fputc(width>>8,f);
      fputc(width&0xff,f);
      fputc(height>>8,f);
      fputc(height&0xff,f);
      fputc(QLI_PIXEL_FORMAT,f);
      uint8_t flags=0;
      flags|= QLI_TYPE(qli_index_code, QLI_POSTFIX) [QLI_INDEX_SIZE];
      fputc(flags,f);
      fwrite(buf,1,ret,f);
    }
    fclose(f);
  }
  free(buf);
  return(ret);
}
#endif

#endif

#if QLI_SINGLETON == 0
#undef QLI_NOSTDIO
#undef QLI_PIXEL_FORMAT
#undef QLI_USERDATA
#undef QLI_POSTFIX
#undef QLI_STRIDE
#undef QLI_ENDIAN
#undef QLI_R_FACTOR
#undef QLI_G_FACTOR
#undef QLI_B_FACTOR
#undef QLI_PACK_GET_RED
#undef QLI_PACK_GET_GREEN
#undef QLI_PACK_GET_BLUE
#undef QLI_PX_GET_RED
#undef QLI_PX_GET_GREEN
#undef QLI_PX_GET_BLUE
#undef QLI_RGB_PACK
#undef QLI_CLAMP255
#undef QLI_COLOR_HASH
#undef QLI_EMIT_PX
#undef QLI_FRACTION_PIXELS_LEFT
#undef QLI_EMIT_UNIT_BYTES
#undef QLI_EMIT_PX_FLUSH
#undef QLI_USERDATA
#undef QLI_CONCAT
#undef QLI_CONCAT_INNER
#undef QLI_FUNC_NAME
#undef QLI_STRUCT_NAME
#undef QLI_TYPE
#undef QLI_BPP
#undef QLI_BPP2
#undef QLI_BYTE_TO_PIXEL
#undef QLI_PIXEL_TO_BYTE
#undef QLI_MIN_OUTPUT_BUFFER_SIZE
#undef qli_pixel_t
#undef TRACE_QLI
#endif

#endif
