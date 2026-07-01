/*------------------------------------------------------------------------------
* imu.cc : decode imu raw measurement data
*
* version : $Revision:$ $Date:$
* history : 2017/11/06  1.0  new
*-----------------------------------------------------------------------------*/
#include <carvig.h>

/* constants/macros ----------------------------------------------------------*/
#define NUMBYTES_GI310  43                      /* numbers of bytes of gi310 imu raw data */
#define YSASM330_HEADLEN 5                       /* YS-ASM330 header bytes */
#define YSASM330_CKLEN   2                       /* YS-ASM330 checksum bytes */
#define MAXDIFFTIME     10.0                    /* max time difference to reset  */
const unsigned char gi310_head[2]={0x55,0xAA};  /* imu message header */
const unsigned char ysasm330_head[2]={0x59,0x53}; /* imu message header */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))

/* get fields (little-endian) ------------------------------------------------*/
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static short          I2(unsigned char *p) {short          i; memcpy(&i,p,2); return i;}
static int            I4(unsigned char *p) {int            i; memcpy(&i,p,4); return i;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}

/* for odometry option--------------------------------------------------------*/
static double res=2048; /* odometry resolution */
static double d=0.73;   /* odometry wheel diameter (m) */

/* set odometry resolution----------------------------------------------------*/
extern void odores(double rese) {res=rese;}
/* set odometry wheel diameter------------------------------------------------*/
extern void odod(double de) {d=de;}

/* check header --------------------------------------------------------------*/
static int chkhead(const unsigned char *buff, const unsigned char* head)
{
    return (buff[0]==head[0])&&(buff[1]==head[1]);
}
/* checksum ------------------------------------------------------------------*/
static unsigned char chksum(const unsigned char *buff, int len)
{
    int i;
    unsigned char sum=0;
    for (i=0;i<len;i++) sum+=buff[i]; return sum;
}
/* YS-ASM330 checksum --------------------------------------------------------*/
static void chksum_ysasm330(const unsigned char *buff, int len,
                            unsigned char *ck1, unsigned char *ck2)
{
    int i;
    *ck1=*ck2=0;
    for (i=0;i<len;i++) {
        *ck1=(unsigned char)(*ck1+buff[i]);
        *ck2=(unsigned char)(*ck2+*ck1);
    }
}
/* decode imu time------------------------------------------------------------*/
static void decode_sow_time(raw_t *raw,double *sow,int *start)
{
    static unsigned int pps=0;

    if (pps==0) {
        pps=U4(raw->buff+6); *sow=U4(raw->buff+2);
        return;
    }
    if (pps!=U4(raw->buff+6)) {(*sow)++; *start=1;}
    pps=U4(raw->buff+6);
}
/* adjust gps seconds of week and imu time------------------------------------*/
static void adjtime(raw_t* raw,const double sowi,double *sowo,double *timu,
                    int *week,unsigned int *dcc)
{
    static unsigned int imuc=0,dc=0;
    int d=0;
    
    if (imuc==0) {
        imuc=U4(raw->buff+10); *sowo=sowi;
        return;
    }
    *dcc=dc=U4(raw->buff+10)-imuc<0?UINT_MAX+U4(raw->buff+10)-imuc:U4(raw->buff+10)-imuc;
    imuc=U4(raw->buff+10);

    /* increase week */
    if ((*sowo=(int)(1.0/FREQOCXO*dc+sowi))>=604800.0) {
        *sowo-=604800.0; (*week)++;
    }
    d=U4(raw->buff+10)-U4(raw->buff+06);
    *timu=*sowo+1.0/FREQOCXO*d;
}
/* decode odometry and convert to velocity in vehicle frame------------------*/
static int decode_odo_data(raw_t *raw,double dt)
{
    static int dc;

    if (raw->imu.time.time==0||dt==0.0) {
        raw->imu.odoc=I2(raw->buff+38); return 0;
    }
    dc=I2(raw->buff+38)-raw->imu.odoc<=SHRT_MIN?
       I2(raw->buff+38)-raw->imu.odoc+USHRT_MAX:
       I2(raw->buff+38)-raw->imu.odoc>=SHRT_MAX?
       I2(raw->buff+38)-raw->imu.odoc-USHRT_MAX:
       I2(raw->buff+38)-raw->imu.odoc;

    raw->imu.odoc=I2(raw->buff+38);
    raw->imu.odo.time=raw->imu.time;
    raw->imu.odo.dt=dt;
    raw->imu.odo.dr=dc/res*PI*d;
    return 1;
}
/* decode imu data ----------------------------------------------------------*/
static int decode_imu_data(raw_t *raw)
{
    int i,week=0;
    unsigned int dc=0;
    static int start=0;
    static double sow=0.0,timu=0.0;
    gtime_t t0;

    raw->imut.n=0;

    /* decode GPS sow (s) */
    decode_sow_time(raw,&sow,&start);

    /* start decode imu time */
    if (start) {
        adjtime(raw,sow,&sow,&timu,&week,&dc);
    }
    else return 0;

    /* current and precious time difference is too large */
    if (dc*1.0/FREQOCXO>MAXDIFFTIME) return 0;

    t0=gpst2time(week,timu);
    raw->imu.pps =U4(raw->buff+06);
    raw->imu.imuc=U4(raw->buff+10);

    for (i=0;i<3;i++) {
        raw->imu.gyro[i]=R4(raw->buff+14+i*4);
        raw->imu.accl[i]=R4(raw->buff+26+i*4);
    }
    decode_odo_data(raw,dc*1.0/FREQOCXO);
    raw->imu.time=t0;

    /* add imu measurement data */
    addimudata(&raw->imut,&raw->imu);
    return timu>0.0?4:0;
}
/* decode M39 IMU GI310 data ------------------------------------------------*/
static int decode_imu_m39(raw_t *raw, unsigned char data)
{
    raw->buff[raw->nbyte++]=data;

    if (raw->nbyte<2) return 0; /* synchronize frame */

    if (!chkhead(raw->buff,gi310_head)) {raw->nbyte=0; return 0;}
    if (raw->nbyte<NUMBYTES_GI310) return 0;

    if (chksum(raw->buff+2,40)==data) { /* checksum */
        raw->nbyte=0;
        return decode_imu_data(raw);
    }
    else {
        raw->nbyte=0; return -1; /* checksum fail */
    }
}
/* decode YS-ASM330 IMU data ------------------------------------------------*/
static int decode_imu_ysasm330(raw_t *raw)
{
    imud_t imu={0};
    double ep[6]={0};
    unsigned int usec=0;
    int i,offset=YSASM330_HEADLEN,end=YSASM330_HEADLEN+U1(raw->buff+4);
    int id,len,has_utc=0,has_gyro=0,has_accl=0;

    raw->imut.n=0;
    imu.imuc=U2(raw->buff+2);

    while (offset+2<=end) {
        id =U1(raw->buff+offset);
        len=U1(raw->buff+offset+1);
        offset+=2;

        if (offset+len>end) return 0;

        switch (id) {
            case 0x01: /* IMU temperature (0.01 C) */
                if (len==2) imu.temp=I2(raw->buff+offset)*0.01;
                break;
            case 0x10: /* accelerometer (micro-g) */
                if (len==12) {
                    for (i=0;i<3;i++) imu.accl[i]=I4(raw->buff+offset+i*4)*Mg2M;
                    has_accl=1;
                }
                break;
            case 0x20: /* gyroscope (micro-deg/s) */
                if (len==12) {
                    for (i=0;i<3;i++) imu.gyro[i]=I4(raw->buff+offset+i*4)*1E-6;
                    has_gyro=1;
                }
                break;
            case 0x50: /* UTC time */
                if (len==11) {
                    usec=U4(raw->buff+offset)*1000;
                    if (usec>999999) usec=999999;
                    ep[0]=U2(raw->buff+offset+4)+2000;
                    ep[1]=U1(raw->buff+offset+6);
                    ep[2]=U1(raw->buff+offset+7);
                    ep[3]=U1(raw->buff+offset+8);
                    ep[4]=U1(raw->buff+offset+9);
                    ep[5]=U1(raw->buff+offset+10);
                    has_utc=1;
                }
                break;
            case 0x51: /* UTC microsecond extension */
                if (len==4) {
                    usec=U4(raw->buff+offset);
                    if (usec>999999) usec=999999;
                }
                break;
        }
        offset+=len;
    }
    if (!has_utc||!has_gyro||!has_accl) return 0;

    ep[5]+=usec*1E-6;
    imu.time=utc2gpst(epoch2time(ep));
    raw->imu=imu;

    addimudata(&raw->imut,&raw->imu);
    return 4;
}
/* decode YS-ASM330 IMU frame -----------------------------------------------*/
static int decode_ysasm330(raw_t *raw)
{
    unsigned char ck1,ck2;
    int len=U1(raw->buff+4);

    chksum_ysasm330(raw->buff+2,3+len,&ck1,&ck2);
    if (ck1!=raw->buff[YSASM330_HEADLEN+len]||
        ck2!=raw->buff[YSASM330_HEADLEN+len+1]) {
        return -1;
    }
    return decode_imu_ysasm330(raw);
}
/* input imu raw data in backward--------------------------------------------*/
static int nextimub(const imu_t *imu,imu_t *data,int *index)
{
    int i; data->n=0;
    for (i=0;i<100;i++) addimudata(data,&imu->data[(*index)--]);
    if (data->n) return 4; else return 0;
}
/* input imu raw data for backward solution----------------------------------*/
static int decode_imu_m39b(raw_t *raw, unsigned char data)
{
    if (raw->imub.n==0) {
        prcopt_t *opt=(prcopt_t *)raw->optp;
        stream_t *str=(stream_t *)raw->strp;
        readimub(str->path,&raw->imub,opt->insopt.imudecfmt,opt->insopt.imuformat,
                 opt->insopt.imucoors,opt->insopt.imuvalfmt);

        raw->curb=raw->imub.n-1;
    }
    raw->imut.n=0;
    return nextimub(&raw->imub,&raw->imut,&raw->curb);
}
/* input imu raw message -----------------------------------------------------
 * args   : raw_t *raw         IO     receiver raw data control struct
 *          unsigned char data I stream data (1 byte)
 * return >1:ok ,0:fail
 * --------------------------------------------------------------------------*/
extern int input_m39(raw_t *raw, unsigned char data)
{
    trace(3,"input_m39: data=%02X\n",data);
    raw->len=NUMBYTES_GI310;
    if (raw->dire) return decode_imu_m39b(raw,data);
    else           return decode_imu_m39 (raw,data);
}
/* input YS-ASM330 raw message -----------------------------------------------
 * args   : raw_t *raw         IO     receiver raw data control struct
 *          unsigned char data I stream data (1 byte)
 * return 4:imu data, 0:no message, -1:checksum fail
 * --------------------------------------------------------------------------*/
extern int input_ysasm330(raw_t *raw, unsigned char data)
{
    int ret;

    trace(3,"input_ysasm330: data=%02X\n",data);

    if (raw->nbyte>=MAXRAWLEN) raw->nbyte=0;
    raw->buff[raw->nbyte++]=data;

    if (raw->nbyte==1) {
        if (raw->buff[0]!=ysasm330_head[0]) raw->nbyte=0;
        return 0;
    }
    if (raw->nbyte==2) {
        if (!chkhead(raw->buff,ysasm330_head)) {
            raw->nbyte=raw->buff[1]==ysasm330_head[0]?1:0;
            if (raw->nbyte) raw->buff[0]=ysasm330_head[0];
        }
        return 0;
    }
    if (raw->nbyte<YSASM330_HEADLEN) return 0;

    raw->len=YSASM330_HEADLEN+U1(raw->buff+4)+YSASM330_CKLEN;
    if (raw->len>MAXRAWLEN) {
        raw->nbyte=raw->len=0;
        return -1;
    }
    if (raw->nbyte<raw->len) return 0;

    ret=decode_ysasm330(raw);
    raw->nbyte=raw->len=0;
    return ret;
}
/* input imu raw message from file --------------------------------------------
* input next imu raw message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, 1: ok 0: fail)
*----------------------------------------------------------------------------*/
extern int input_m39f(raw_t *raw,FILE *fp)
{
    int i,data,ret;

    trace(3,"input_imuf:\n");

    for (i=0;i<4096;i++) {
        if ((data=fgetc(fp))==EOF) return -2;
        if ((ret=input_m39(raw,(unsigned char)data))) return ret;
    }
    return 0; /* return at every 4k bytes */
}
/* input YS-ASM330 raw message from file -------------------------------------*/
extern int input_ysasm330f(raw_t *raw,FILE *fp)
{
    int i,data,ret;

    trace(3,"input_ysasm330f:\n");

    for (i=0;i<4096;i++) {
        if ((data=fgetc(fp))==EOF) return -2;
        if ((ret=input_ysasm330(raw,(unsigned char)data))) return ret;
    }
    return 0;
}
/* read imu measurement data from file-----------------------------------------
 * args   :  char *file     I  input imu measurement data file
 *           imu_t *imu     O  output imu measurement data
 *           int    decfmt  I  imu measurement data decode method
 *           int    imufmt  I  imu measurement data format
 *           int    coor    I  imu body coordinate frame
 *           int    valfmt  I  imu gyro measurement data format
 * return : number of imu measurement data
 * --------------------------------------------------------------------------*/
extern int readimub(const char *file,imu_t* imu,int decfmt,int imufmt,int coor,
                    int valfmt)
{
    FILE *fp;
    raw_t raw={0};
    int data,siz;

    trace(3,"readimub:\n");

    raw.imufmt=imufmt;
    imu->n=imu->nmax=0; imu->data=NULL;
    imu->format=decfmt; imu->coor=coor; imu->valfmt=valfmt;

    if (!(fp=fopen(file,"r"))) {
        trace(2,"imu measurement data file open error \n");
        return 0;
    }
    /* read imu message from file */
    while (1) {
        if ((data=fgetc(fp))==EOF) break;
        if ((input_m39(&raw,(unsigned char)data))) {

            if (imu->n>=imu->nmax) {
                trace(5,"readimub: imu->n=%d nmax=%d\n",imu->n,imu->nmax);

                siz=sizeof(imud_t)*(imu->nmax+=4096);
                if (!(imu->data=(imud_t*)realloc(imu->data,siz))) {

                    fprintf(stderr,"readimub :memory allocation error\n");
                    free(imu->data); imu->n=imu->nmax=0;
                    break;
                }
            }
            imu->data[imu->n++]=raw.imu;
        }
    }
    fclose(fp);
    return imu->n;
}
/* read YS-ASM330 imu measurement data from file------------------------------
 * args   :  char *file     I  input imu measurement data file
 *           imu_t *imu     O  output imu measurement data
 * return : number of imu measurement data
 * --------------------------------------------------------------------------*/
extern int readysasm330(const char *file,imu_t* imu)
{
    FILE *fp;
    raw_t raw={0};
    int data,ret;

    trace(3,"readysasm330:\n");

    imu->n=imu->nmax=0; imu->data=NULL;
    imu->decfmt=IMUDECFMT_RATE;
    imu->format=STRFMT_YSASM330;
    imu->coor=IMUCOOR_FRD;
    imu->valfmt=IMUVALFMT_DEG;

    if (!(fp=fopen(file,"rb"))) {
        trace(2,"imu measurement data file open error \n");
        return 0;
    }
    while ((data=fgetc(fp))!=EOF) {
        ret=input_ysasm330(&raw,(unsigned char)data);
        if (ret==4) addimudata(imu,&raw.imu);
    }
    fclose(fp);
    freeimudata(&raw.imut);
    return imu->n;
}
/* add imu measurement data -------------------------------------------------*/
extern int addimudata(imu_t *imu, const imud_t *data)
{
    imud_t *obs_data;

    if (imu->nmax<=imu->n) {
        if (imu->nmax<=0) imu->nmax=64; else imu->nmax*=2;
        if (!(obs_data=(imud_t *)realloc(imu->data,sizeof(imud_t)*imu->nmax))) {
            trace(1,"addimudata: memalloc error n=%dx%d\n",sizeof(imud_t),imu->nmax);
            free(imu->data); imu->data=NULL; imu->n=imu->nmax=0;
            return -1;
        }
        imu->data=obs_data;
    }
    imu->data[imu->n++]=*data;
    return 1;
}
/* free imu measurement data--------------------------------------------------*/
extern void freeimudata(imu_t *imu)
{
    trace(3,"freeimudata:\n");
    if (imu->data) {
        free(imu->data); imu->data=NULL; imu->n=imu->nmax=0;
    }
}
