/*
 * Copyright 2025 Gergely Gati
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
#ifndef QLI_H
#define QLI_H


#include <stdint.h>
#include <string.h>


#define QLI_PF_RGB565 0
#define QLI_PF_RGB888 1

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

#if QLI_PIXEL_FORMAT == QLI_PF_RGB565
 #define QLI_BPP (2)
#elif QLI_PIXEL_FORMAT == QLI_PF_RGB888
 #define QLI_BPP (3)
#else
 #error "Unsupported pixel format"
#endif

#define QLI_LITTLE_ENDIAN (QLI_BPP-1)
#define QLI_BIG_ENDIAN    (0)

// configuration value
#ifndef QLI_ENDIAN
#define QLI_ENDIAN QLI_LITTLE_ENDIAN
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

#define QLI_REMBUFSIZ (QLI_BPP+1)


struct qli_image
{
  uint8_t *data;
  int32_t pos;
  uint32_t run;
  uint32_t pixels_left;
  uint32_t size;
  uint16_t width;
  uint16_t height;
  uint16_t stride;
  uint16_t x;
  uint8_t px[QLI_BPP];
  int8_t remcnt;
  uint8_t rem[QLI_REMBUFSIZ];
  uint8_t index[QLI_BPP][1L<<QLI_INDEX_SIZE];
  QLI_USERDATA
};


#define QLI_CONCAT(a, b) QLI_CONCAT_INNER(a, b)
#define QLI_CONCAT_INNER(a, b) a##b
#define QLI_FUNC_NAME(name, post) QLI_CONCAT(name, post)


#ifdef QLI_DECODE
int QLI_FUNC_NAME(qli_init, QLI_POSTFIX) (struct qli_image *qli, uint16_t width, uint16_t height, uint16_t stride, uint8_t *data, uint32_t data_size);
int QLI_FUNC_NAME(qli_init_header, QLI_POSTFIX) (struct qli_image *qli, uint8_t *header, uint32_t size);
void QLI_FUNC_NAME(qli_rewind, QLI_POSTFIX) (struct qli_image *qli);
int QLI_FUNC_NAME(qli_decode, QLI_POSTFIX) (struct qli_image *qli, uint8_t *dest, uint32_t pixel_cnt, int *new_chunk);
void QLI_FUNC_NAME(qli_new_chunk, QLI_POSTFIX) (struct qli_image *qli, uint8_t *data, uint32_t data_size);
#endif

#ifdef QLI_ENCODE
int QLI_FUNC_NAME(qli_encode,QLI_POSTFIX) (uint32_t *rgb, int width, int height, int stride, uint8_t *buf, size_t bufsize);
#if QLI_NOSTDIO == 0
int QLI_FUNC_NAME(qli_save,QLI_POSTFIX) (uint32_t *rgb, int width, int height, char *file);
#endif
#endif

#define QLI_HEADER_LEN (10)

#endif

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


static const uint8_t qli_index_code[]={0,0,0,0,1,2,3,0};


#if QLI_PIXEL_FORMAT == QLI_PF_RGB565
 typedef uint16_t qli_pixel_t;
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
 #define QLI_PX_GET_RED(px_)   QLI_PACK_GET_RED   ((((qli_pixel_t)(px_[0]))<<8L|px_[1]))
 #define QLI_PX_GET_GREEN(px_) QLI_PACK_GET_GREEN ((((qli_pixel_t)(px_[0]))<<8L|px_[1]))
 #define QLI_PX_GET_BLUE(px_)  QLI_PACK_GET_BLUE  ((((qli_pixel_t)(px_[0]))<<8L|px_[1]))

#elif QLI_PIXEL_FORMAT == QLI_PF_RGB888
 typedef uint32_t qli_pixel_t;
 #define QLI_R_FACTOR (1)
 #define QLI_G_FACTOR (1)
 #define QLI_B_FACTOR (1)
 #define QLI_RGB_PACK(r, g, b) ( ((qli_pixel_t)r&0xff)<<16 | ((qli_pixel_t)g&0xff)<<8 | ((qli_pixel_t)g&0xff) )
 #define QLI_PACK_GET_RED(rgb888)   (uint8_t)((rgb888>>16)&0xff)
 #define QLI_PACK_GET_GREEN(rgb888) (uint8_t)((rgb888>>8)&0xff)
 #define QLI_PACK_GET_BLUE(rgb888)  (uint8_t)((rgb888>>0)&0xff)
 #define QLI_PX_GET_RED(px_)   QLI_PACK_GET_RED   (px_[0])
 #define QLI_PX_GET_GREEN(px_) QLI_PACK_GET_GREEN (px_[1])
 #define QLI_PX_GET_BLUE(px_)  QLI_PACK_GET_BLUE  (px_[2])

#else
 #error "Unsupported pixel format"
#endif

// hash function is from QOI (alpha removed)
#define QLI_COLOR_HASH(r,g,b) ((r)*3 + (g)*5 + (b)*7)
#define QLI_GET_INDEX(p) QLI_COLOR_HASH(QLI_PX_GET_RED(p),QLI_PX_GET_GREEN(p),QLI_PX_GET_BLUE(p)) % (1L<<QLI_INDEX_SIZE)
#define QLI_UPDATE_INDEX(q,p) \
  do { \
    int idx = QLI_GET_INDEX(p); \
    for(int i=0;i<QLI_BPP;i++) (q)->index[i][idx] = p[i]; \
  } while(0)


/* rewind position pointer
 */
void QLI_FUNC_NAME(qli_rewind, QLI_POSTFIX) (struct qli_image *qli)
{
  if(NULL!=qli)
  {
    qli->pos=0;
    memset(qli->index,0,sizeof(qli->index));
    qli->run=0;
    qli->x=0;
    memset(qli->px,0,sizeof(qli->px));
    qli->pixels_left=qli->width * qli->height;
  }
}

/* init user allocated qli struct
 */
int QLI_FUNC_NAME(qli_init, QLI_POSTFIX) (struct qli_image *qli, uint16_t width, uint16_t height, uint16_t stride, uint8_t *data, uint32_t data_size)
{
  if(NULL==qli) return(-1);
  qli->width = width;
  qli->height = height;
  qli->stride = stride;
  qli->data = data;
  qli->size = data_size;
  QLI_FUNC_NAME(qli_rewind,QLI_POSTFIX) (qli);
  return(0);
}

int QLI_FUNC_NAME(qli_init_header, QLI_POSTFIX) (struct qli_image *qli, uint8_t *header, uint32_t size)
{
  if(NULL==qli||NULL==header) return(-1);
  if(   header[0]!=QLI_MAGIC0
     || header[1]!=QLI_MAGIC1
     || header[2]!=QLI_MAGIC2
     || header[3]!=QLI_MAGIC3 ) return(-1);
  header+=4;
  qli->width=header[0]<<8|header[1];
  qli->height=header[2]<<8|header[3];
  qli->stride=qli->width*QLI_BPP;
  if(QLI_PIXEL_FORMAT!=header[4]) return(-1);
  int flags=header[5];
  if(qli_index_code[QLI_INDEX_SIZE]!=((flags)&3)) return(-1);
  qli->size = size;
  QLI_FUNC_NAME(qli_rewind,QLI_POSTFIX) (qli);
  return(0);
}


#ifdef QLI_DECODE

int DBG_POS[2000];
int DBG_POS_SIZE=sizeof(DBG_POS);

#define QLI_GETNEXTBYTE(qli) ( ((qli)->pos>=0) ? ((qli)->data[(qli)->pos++]) : ((qli)->rem[QLI_REMBUFSIZ+((qli)->pos++)]) )


void QLI_FUNC_NAME(qli_new_chunk, QLI_POSTFIX) (struct qli_image *qli, uint8_t *data, uint32_t data_size)
{
  if(NULL==qli) return;
  qli->data=data;
  qli->size=data_size;
  qli->pos=-qli->remcnt;
  DBG_POS[__LINE__]++;
  qli->remcnt=0;
}

/* decoding pixel_cnt pixels to the supplied destination area
 *
 * RETURN: number of pixels extracted
 */
int QLI_FUNC_NAME(qli_decode, QLI_POSTFIX) (struct qli_image *qli, uint8_t *dest, uint32_t pixel_cnt, int *new_chunk)
{
  int ret=0;
  uint8_t d1;
  int i;

  if(!qli || !dest) return(-1);
  if(qli->pixels_left == 0) return(0);
  if(qli->pixels_left < pixel_cnt) pixel_cnt=qli->pixels_left;

//printf("qli_decode: stride=%d w/bpp=%d\n",qli->stride,qli->width*QLI_BPP);
//printf("qli_decode: pixel_cnt=%d dest=%p\n",pixel_cnt,dest);
  while(pixel_cnt>0 && ( qli->pos<0 || qli->pos<=qli->size) )
  {
    if(qli->run>0)
    {
  //    printf("  qli_decode: pixel_cnt=%5d run=%3d dest=%p\n",pixel_cnt,qli->run,dest);
      for(i=0;i<QLI_BPP;i++) *dest++ = qli->px[ABS(QLI_ENDIAN-i)];
    //  printf("  qli_decode:                         dest=%p\n",dest);
      qli->x++;
      if(qli->x==qli->width)
      {
        qli->x=0;
        if(0!=qli->stride) dest+=qli->stride-(qli->width*QLI_BPP);
      }
      ret++;
      --pixel_cnt;
      --qli->run;
      continue;
    }

    if(qli->pos>0 && qli->pos >= qli->size) break;
    d1 = QLI_GETNEXTBYTE(qli);
if(qli->pos<=1) fprintf(stderr,"==HURKA %d pos=%d data 0x%02x\n",__LINE__,qli->pos,d1);
    DBG_POS[__LINE__]++;
    uint8_t cm = d1&QLI_CMD_MASK;

    if (QLI_OP_RGB == d1)
    {
/* hiba itt */
      if((qli->pos+QLI_BPP) > qli->size)
      {
        fprintf(stderr,"--HURKA pos=%d size=%d data = 0x%02x ",qli->pos,qli->size,QLI_OP_RGB);
        qli->remcnt=qli->size-qli->pos+1;
        i=sizeof(qli->rem)-qli->remcnt;
        qli->rem[i++]=QLI_OP_RGB;
        for(;i<sizeof(qli->rem);i++)
        {
          qli->rem[i]=QLI_GETNEXTBYTE(qli);
          fprintf(stderr," 0x%02x ",qli->rem[i]);
          DBG_POS[__LINE__]++;
        }
        fprintf(stderr,"\n--HURKA remcnt=%d BPP=%d\n",qli->remcnt,QLI_BPP);
        qli->run=0;
/*
        qli->rem[sizeof(qli->rem)-1]=QLI_OP_RGB;
        qli->remcnt=1;
        for(i=sizeof(qli->rem)-1; i>=0 && qli->pos<qli->size; i--,qli->remcnt++)
        {
          memmove(&qli->rem[0],&qli->rem[1],QLI_REMBUFSIZ-1);
          qli->rem[sizeof(qli->rem)-1]=QLI_GETNEXTBYTE(qli);
          fprintf(stderr," 0x%02x ",qli->rem[sizeof(qli->rem)-1]);
          DBG_POS[__LINE__]++;
        }
        fprintf(stderr,"\n--HURKA remcnt=%d BPP=%d\n",qli->remcnt,QLI_BPP);
        qli->run=0;
*/
      }
      else
      {
/* hiba vege */
        for(i=0;i<QLI_BPP;i++)
        {
          qli->px[i] = QLI_GETNEXTBYTE(qli);
if(qli->pos<=2) fprintf(stderr,"==HURKA %d pos=%d data 0x%02x\n",__LINE__,qli->pos,qli->px[i]);
          DBG_POS[__LINE__]++; 
        }
        QLI_UPDATE_INDEX(qli, qli->px);
        qli->run=1;
      }
    }
    else if(QLI_OP_INDEX == cm)
    {
      d1&=(1L<<QLI_INDEX_SIZE)-1;
      for(int i=0;i<QLI_BPP;i++) qli->px[i] = qli->index[i][d1];
      qli->run=1;
    }
    else if(QLI_OP_DIFF == cm)
    {
      int r = QLI_PX_GET_RED(qli->px);
      int g = QLI_PX_GET_GREEN(qli->px);
      int b = QLI_PX_GET_BLUE(qli->px);
      r += QLI_R_FACTOR * (((d1 >> 4) & 0x03) - 2);
      g += QLI_G_FACTOR * (((d1 >> 2) & 0x03) - 2);
      b += QLI_B_FACTOR * (( d1       & 0x03) - 2);
      qli_pixel_t rgb = QLI_RGB_PACK(r,g,b);
      for(int i=0;i<QLI_BPP;i++) qli->px[i] = (rgb>>(8*(QLI_BPP-i-1)))&0xff;
      QLI_UPDATE_INDEX(qli, qli->px);
      qli->run=1;
    }
    else if(QLI_OP_LUMA == cm)
    {
      int r = QLI_PX_GET_RED(qli->px);
      int g = QLI_PX_GET_GREEN(qli->px);
      int b = QLI_PX_GET_BLUE(qli->px);
      if((qli->pos+1) > qli->size)
      {
        fprintf(stderr,"--HURKA LUMA pos=%d size=%d data = 0x%02x ",qli->pos,qli->size,d1);
        qli->remcnt=qli->size-qli->pos+1;
        i=sizeof(qli->rem)-qli->remcnt;
        qli->rem[i++]=d1;
        for(;i<sizeof(qli->rem);i++)
        {
          qli->rem[i]=QLI_GETNEXTBYTE(qli);
          fprintf(stderr," 0x%02x ",qli->rem[i]);
          DBG_POS[__LINE__]++;
        }
        fprintf(stderr,"\n--HURKA LUMA remcnt=%d\n",qli->remcnt);
        qli->run=0;

/*
        fprintf(stderr,"--HURKA LUMA pos=%d size=%d data = ",qli->pos,qli->size);
        qli->rem[sizeof(qli->rem)-1]=d1;
        qli->remcnt=1;
        for(i=sizeof(qli->rem)-1; i>=0 && qli->pos<qli->size; i--,qli->remcnt++)
        {
          memmove(&qli->rem[0],&qli->rem[1],QLI_REMBUFSIZ-1);
          qli->rem[sizeof(qli->rem)-1]=QLI_GETNEXTBYTE(qli);
          fprintf(stderr," 0x%02x ",qli->rem[sizeof(qli->rem)-1]);
          DBG_POS[__LINE__]++;
        }
        fprintf(stderr,"\n--HURKA remcnt=%d MAX=%d\n",qli->remcnt,1);
        qli->run=0;
*/
      }
      else
      {
        int d2 = QLI_GETNEXTBYTE(qli);
if(qli->pos<=1) fprintf(stderr,"==HURKA %d data 0x%02x\n",__LINE__,d2);
        DBG_POS[__LINE__]++;
        int vg = (d1 & 0x3f) - 32;
        r += QLI_R_FACTOR * (vg - 8 + ((d2 >> 4) & 0x0f));
        g += QLI_G_FACTOR * (vg);
        b += QLI_B_FACTOR * (vg - 8 +  (d2       & 0x0f));
        qli_pixel_t rgb = QLI_RGB_PACK(r,g,b);
        for(int i=0;i<QLI_BPP;i++) qli->px[i] = (rgb>>(8*(QLI_BPP-i-1)))&0xff;
        QLI_UPDATE_INDEX(qli, qli->px);
        qli->run=1;
      }
    }
    else if(QLI_OP_RUN == cm)
    {
      qli->run=1+(d1&0x3f);
    }
  }
  qli->pixels_left-=ret;
  if(qli->pos>0 && qli->pos>=qli->size && new_chunk!=NULL && qli->pixels_left>0)  *new_chunk=1;

  return(ret);
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
int QLI_FUNC_NAME(qli_encode, QLI_POSTFIX) (uint32_t *rgb, int width, int height, int stride, uint8_t *buf, size_t bufsize)
{
  int i,res;
  struct qli_image img;
  int pos,opos;
  int out_cnt=0;
  uint32_t pix=0;
  uint8_t px[QLI_BPP]={0};
  qli_pixel_t ppx,ppx_prev=0;
  int run=0,end;

  if(rgb==NULL||width<=0||height<=0) return(-1);
  if(0!=qli_init(&img, width, height, 0, buf, bufsize)) return(-1);
  if(stride==0) stride = width*sizeof(uint32_t);
  end=width*height;
  if(NULL==buf) bufsize=width*height*(QLI_BPP+1);
  for(pos=opos=0;pos<end;pos++,ppx_prev=ppx)
  {
    pix=rgb[ (pos / width) * (stride/sizeof(uint32_t)) + (pos % width) ];
    ppx=QLI_RGB_PACK(QLI_RGB32_RED(pix), QLI_RGB32_GREEN(pix), QLI_RGB32_BLUE(pix));
    for(i=0;i<QLI_BPP;i++) px[i] = (ppx>>(8*(QLI_BPP-i-1)))&0xff;
    if(ppx==ppx_prev && pos<end-1)
    {
      run++;
      continue;
    }
    int idx=QLI_GET_INDEX(px);
    if(run>0)
    {
      while(run>0)
      {
        if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_RUN|( ((run-QLI_MAX_RUN_VALUE)>0 ? QLI_MAX_RUN_VALUE : run) - 1 );
        run-=QLI_MAX_RUN_VALUE;
        out_cnt++;
      }
      run=0;
    }
    for(i=res=0;i<QLI_BPP;i++) res|=(px[i]^img.index[i][idx]);
    if(res==0)
    {
      if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_INDEX|(idx);
      out_cnt++;
      continue;
    }
    QLI_UPDATE_INDEX(&img,px);
    signed char vr = ( QLI_PACK_GET_RED(ppx)   - QLI_PACK_GET_RED(ppx_prev)   ) / QLI_R_FACTOR;
    signed char vg = ( QLI_PACK_GET_GREEN(ppx) - QLI_PACK_GET_GREEN(ppx_prev) ) / QLI_G_FACTOR;
    signed char vb = ( QLI_PACK_GET_BLUE(ppx)  - QLI_PACK_GET_BLUE(ppx_prev)  ) / QLI_B_FACTOR;
    if( vr>-3 && vr<2 && vg>-3 && vg<2 && vb>-3 && vb<2 )
    {
      if(NULL!=buf&&opos<bufsize) buf[opos++]=QLI_OP_DIFF|( (vr + 2) << 4 | (vg + 2) << 2 | (vb + 2) );
      out_cnt++;
      continue;
    }
    signed char vg_r = vr-vg;
    signed char vg_b = vb-vg;
    if( vg_r>-9 && vg_r<8 && vg>-33 && vg<32 && vg_b>-9 && vg_b<8 )
    {
      if(NULL!=buf&&opos<bufsize-1)
      {
        buf[opos++]=QLI_OP_LUMA | (vg+32);
        buf[opos++]=(vg_r+8)<<4 | (vg_b+8);
      }
      out_cnt+=2;
      continue;
    }
    if(NULL!=buf&&opos<bufsize-QLI_BPP-1)
    {
      buf[opos++]=QLI_OP_RGB;
      for(i=0;i<QLI_BPP;i++) buf[opos++] = px[i];
    }
    out_cnt+=1+QLI_BPP;
  }
  return(out_cnt);
}

#if QLI_NOSTDIO == 0
int QLI_FUNC_NAME(qli_save, QLI_POSTFIX) (uint32_t *rgb, int width, int height, char *file)
{
  uint8_t *buf;
  int ret=0;
  
  if(rgb==NULL||width<=0||height<=0||NULL==file) return(-1);
  buf=calloc(1,width*height*QLI_BPP);
  ret= QLI_FUNC_NAME(qli_encode, QLI_POSTFIX) (rgb, width, height, 0, buf, width*height*QLI_BPP);
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
      flags|=qli_index_code[QLI_INDEX_SIZE];
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

#endif
