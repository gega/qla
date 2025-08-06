//gcc -fsanitize=address -fsanitize=leak -fno-omit-frame-pointer -Wall -g -o q q.c qla qla.c
//gcc -O0 -g --coverage -fcondition-coverage -fsanitize=address -fsanitize=leak -fno-omit-frame-pointer -Wall -Wdangling-pointer -g -o qla qla.c
#include <stdio.h>
#include <stdlib.h>


extern char DBG_BUFFER[];


#define QLI_PIXEL_FORMAT 0
#define QLI_IMPLEMENTATION
#include "qli.h"
#define QLA_IMPLEMENTATION
#include "qla.h"
#define RPPM_IMPLEMENTATION
#include "rppm.h"

#define PIX_BUF (88)

#define RGB_TO_RGB565LE(r, g, b) (uint16_t)(((((uint16_t)g))<<13) | ((((uint16_t)b)&0xf8)<<5) | (((uint16_t)r)&0xf8) | (((uint16_t)g)>>5))
#define RGB_TO_RGB565BE(r, g, b) (((((uint16_t)r) & 0xf8) << 8) | ((((uint16_t)g) & 0xfc) << 3) | (((uint16_t)b) >> 3))

#define BE_GET_RED(rgb565)   (uint8_t)(((rgb565)>>8L)&~7L)
#define BE_GET_GREEN(rgb565) (uint8_t)(((rgb565)>>3L)&~3L)
#define BE_GET_BLUE(rgb565)  (uint8_t)(((rgb565)<<3L)&~7L)

#define LE_GET_RED(rgb565)   (uint8_t)BE_GET_RED(((rgb565>>8L)|(rgb565<<8L)))
#define LE_GET_GREEN(rgb565) (uint8_t)BE_GET_GREEN(((rgb565>>8L)|(rgb565<<8L)))
#define LE_GET_BLUE(rgb565)  (uint8_t)BE_GET_BLUE(((rgb565>>8L)|(rgb565<<8L)))

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif


static struct rppm img[3]={0};
static int imgp=3;

void rel_cb(uint32_t *b)
{
  for(int i=0;i<3;i++)
  {
    if(img[i].pixels==b)
    {
      rppm_free(&img[i]);
      memset(&img[i],0,sizeof(struct rppm));
    }
  }
}

static FILE *wppm_newframe(char *name, int frameno)
{
  char fnam[44];
  snprintf(fnam, sizeof(fnam), "%s-%04d.ppm",name,frameno);
  FILE *f=fopen(fnam, "wb");
  return(f);
}

static void wppm_header(FILE *f, int width, int height)
{
  /*
    - A "magic number" for identifying the file type. A ppm image's magic number is the two characters "P6".
    - Whitespace (blanks, TABs, CRs, LFs).
    - A width, formatted as ASCII characters in decimal.
    - Whitespace.
    - A height, again in ASCII decimal.
    - Whitespace.
    - The maximum color value (Maxval), again in ASCII decimal. Must be less than 65536 and more than zero.
    - A single whitespace character (usually a newline).
    - A raster of Height rows, in order from top to bottom. Each row consists of Width pixels, in order from left to right.
      Each pixel is a triplet of red, green, and blue samples, in that order. Each sample is represented in pure binary by
      either 1 or 2 bytes. If the Maxval is less than 256, it is 1 byte. Otherwise, it is 2 bytes. The most significant
      byte is first. 
   */
  fprintf(f, "P6\n%d %d\n255\n",width,height);
}

static void wppm_close(FILE *f)
{
  fclose(f);
}

static unsigned long crc32(unsigned long crc, const void *_p, size_t len)
{
	unsigned long crc2, b, i;
	const unsigned char *p = _p;
	static volatile int crc_tbl_inited = 0;
	static unsigned long crc_tbl[256];

	if (_p == NULL)
		return (0);

	if (!crc_tbl_inited) {
		for (b = 0; b < 256; ++b) {
			crc2 = b;
			for (i = 8; i > 0; --i) {
				if (crc2 & 1)
					crc2 = (crc2 >> 1) ^ 0xedb88320UL;
				else    
					crc2 = (crc2 >> 1);
			}
			crc_tbl[b] = crc2;
		}
		crc_tbl_inited = 1;
	}

	crc = crc ^ 0xffffffffUL;
	/* A use of this loop is about 20% - 30% faster than
	 * no use version in any optimization option of gcc.  */
	for (;len >= 8; len -= 8) {
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
	}
	while (len--)
		crc = crc_tbl[(crc ^ *p++) & 0xff] ^ (crc >> 8);
	return (crc ^ 0xffffffffUL);
}

// calling ./qla delay filename-stem output-dir
// filename template: "filename-stem%03d.ppm" , counter
int main(int argc, char **argv)
{
  char fnam[PATH_MAX];
  int counter=0;
  struct qla_encode qla;
  int inited=0;
  uint8_t hdr[QLA_HEADER_LEN];
  uint8_t *buf=NULL;
  int buflen;
  int delay;
  int frame_len;

  if(argc<2) { fprintf(stderr,"err, not enough arguments\n"); exit(0); }
  if((argv[1][0]=='e'&&argc<5) || (argv[1][0]=='d'&&argc<3) )
  {
    fprintf(stderr,"Usage: %s [e|d] delay filename-stem out.qla\n",argv[0]);
    exit(0);
  }
  if(argv[1][0]=='e')
  {
    // encode
    FILE *f;
    delay=atoi(argv[2]);
    if(NULL==(f=fopen(argv[4],"wb"))) { fprintf(stderr,"Cannot create output file '%s'\n",argv[4]); exit(1); }
    for(counter=0; 1 ;counter++)
    {
      snprintf(fnam,sizeof(fnam),"%s%03d.ppm",argv[3],counter);
      imgp = (imgp+1) % 3;
      if(0!=rppm_load(&img[imgp], fnam)) break;
      fprintf(stderr,"ppm opened '%s'\n",fnam);
      if(!inited)
      {
        // write header
        if(0!=(qla_init_encode(&qla, img[imgp].width, img[imgp].height, rel_cb))) { fprintf(stderr,"Unable to initialize qla struct\n"); exit(1); }
        if(0!=(qla_generate_header(&qla, hdr))) { fprintf(stderr,"Error generating header data\n"); exit(1); }
        fwrite(hdr, sizeof(hdr), 1, f);
        if(ferror(f)) { fprintf(stderr,"Write error\n"); exit(1); }
        buflen=QLA_HEADER_LEN+img[imgp].width*img[imgp].height*QLI_BPP;
        buf=malloc(buflen);
        if(NULL==buf) { fprintf(stderr,"Out of memory\n"); exit(1); }
        fprintf(stderr,"header written\n");
        inited=1;
      }
      frame_len = qla_encode_frame(&qla, img[imgp].pixels, delay, buf, buflen);
      fprintf(stderr,"frame %d encoded (%d bytes)\n",counter, frame_len);
      fwrite(buf, frame_len, 1, f);
      if(ferror(f)) { fprintf(stderr,"Write error\n"); exit(1); }
    }
    if(NULL!=buf) free(buf);
    fclose(f);
    if(inited) qla_destroy_encode(&qla);
    else { fprintf(stderr,"No input files\n"); exit(1); }
  }
  else if(argv[1][0]=='d')
  {
    // decode (pseudo)
#define READBUFLEN (HARKALY1) /* even! 261 error 2610 ok */
#define OUTBUFLEN (HARKALY2) /* size ? */
    uint8_t outbuf[OUTBUFLEN];
    uint8_t readbuf[READBUFLEN];
    uint32_t readbuf_len=0;
    uint8_t header[QLA_HEADER_LEN];
    int xx,yy,rcol;
    int size;
    int loop=0;
    int new_chunk=0;
    int frameno=0;
    struct qla_anim q;
    FILE *fp = fopen(argv[2],"rb");
    fseek(fp, 0, SEEK_END);
    int insize=ftell(fp)-QLA_HEADER_LEN;
    fseek(fp, 0, SEEK_SET);
    fread(header,1,QLA_HEADER_LEN,fp);
    int st=qla_init_header(&q, header, sizeof(header), NULL, 0);
    if(st!=0) 
    {
      fprintf(stderr,"ERROR init header\n");
      exit(0);
    }
    uint8_t *framebuffer=calloc(q.width*q.height,QLI_BPP);
 //   int framebuffer_pos=0;
    uint8_t *fb888=calloc(q.width*q.height,3);
    size_t first_frame=ftell(fp);
//    time1_ms = time_now();
    while( 1 )
    {
      size = qla_decode_frame(&q, outbuf, sizeof(outbuf), (feof(fp) ? QLA_FLUSH : &new_chunk) );
      if(new_chunk)
      {
        new_chunk=0;
        readbuf_len=fread(readbuf, 1, MIN(sizeof(readbuf),insize), fp);
        insize-=readbuf_len;
        qla_new_chunk(&q, readbuf, readbuf_len);
        if(readbuf_len==0)
        {
          // loop!
          fprintf(stderr," LOOP!\n");
          fseek(fp, first_frame, SEEK_SET);
          loop=1;
          break;
        }
        if(feof(fp))
        {
          fprintf(stderr," LOOP DELAY!\n");
          fseek(fp, first_frame, SEEK_SET);
          loop=1;
        }
      }
      if(size == QLA_NEWFRAME)
      {
//        framebuffer_pos=0;
        if(frameno!=0)
        {
          FILE *fo=wppm_newframe("out",frameno);
          wppm_header(fo, q.width, q.height);
          for(int i=0;i<(q.width*q.height);i++)
          {
            int rgb565=((int)framebuffer[i*2+1])<<8|framebuffer[i*2];
            fb888[i*3+0]=QLI_PACK_GET_RED(rgb565);
            fb888[i*3+1]=QLI_PACK_GET_GREEN(rgb565);
            fb888[i*3+2]=QLI_PACK_GET_BLUE(rgb565);
          }
          fwrite(fb888,q.width*q.height,3,fo);
          wppm_close(fo);
          //exit(0);
        }
        if(!loop) frameno++;
        else
        {
          frameno=1;
          break;
        }
        loop=0;
        //time2_ms = time_now();
        //if( (time2_ms-time1_ms) < q.delay ) sleep( q.delay-(time2_ms-time1_ms) );
        //time1_ms = time2_ms;
        fprintf(stderr,"FRANE NO #%d\n",frameno);
      }
      else if(size == QLA_NEWRECT)
      {
        fprintf(stderr," SET_WINDOW %d,%d %dx%d\n",q.rect.x, q.rect.y, q.rect.w, q.rect.h);
        xx=0;
        yy=0;
        rcol=rand()%0xffff;
      }
      else if(size>0)
      {
        unsigned long crc=crc32(0L, outbuf, size);
        fprintf(stderr," SEND SPI %d bytes [%08lx]\n", size, crc);
        for(int i=0;i<size/QLI_BPP;i++)
        {
          for(int j=0;j<QLI_BPP;j++)
          {
            if(q.rect.w<0)
            {
            framebuffer[ j + ((q.rect.y+yy)*q.width*QLI_BPP) + ((q.rect.x+xx)*QLI_BPP) ]=rcol;
            }
            else
            {
            framebuffer[ j + ((q.rect.y+yy)*q.width*QLI_BPP) + ((q.rect.x+xx)*QLI_BPP) ]=outbuf[i*QLI_BPP+j];
            }
          }
          xx++;
          if(xx>=q.rect.w)
          {
            xx=0;
            yy++;
          }
        }
        //memcpy(&framebuffer[framebuffer_pos], outbuf, size);
        //framebuffer_pos+=size;
      }
    }
    free(framebuffer);
    free(fb888);
  }
  else { fprintf(stderr,"Unknown command '%c'\n",argv[1][0]); exit(1); }
  for(int i=0;DBG_BUFFER[i]>=0;i++) if(DBG_BUFFER[i]>0) fprintf(stderr,"QDS %d: %d\n",i,DBG_BUFFER[i]);

  return(0);
}
