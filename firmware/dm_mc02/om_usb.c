#include "om_usb.h"
#include <string.h>
#include <math.h>

_Static_assert(sizeof(float) == 4, "IEEE binary32 required");
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static void wr32(uint8_t *p, uint32_t n) {
    for (unsigned i=0; i<4; ++i) p[i]=(uint8_t)(n>>(8*i));
}
static uint32_t crc(const uint8_t *p, size_t n) {
    uint32_t c=0xffffffffu;
    while (n--) { c^=*p++; for(unsigned i=0;i<8;++i) c=(c>>1)^((c&1u)?0xedb88320u:0u); }
    return ~c;
}
static int valid(const om_joint *j) {
    return j->bus<3 && j->motor_id>=1 && j->motor_id<=127 &&
        (j->direction==1 || j->direction==-1) &&
        isfinite(j->lower) && isfinite(j->upper) && j->lower<j->upper &&
        isfinite(j->offset) && isfinite(j->ratio) && j->ratio>0 && j->ratio<=100 &&
        isfinite(j->kp) && j->kp>0 && j->kp<=5000 &&
        isfinite(j->kd) && j->kd>0 && j->kd<=100 &&
        fabsf(j->offset+j->direction*j->ratio*j->lower)<=12.57f &&
        fabsf(j->offset+j->direction*j->ratio*j->upper)<=12.57f;
}
static uint16_t quant(double value,double low,double high) {
    return (uint16_t)((value-low)*65535.0/(high-low));
}
int om_encode_targets(const om_joint *j,const float *q,om_can *out) {
    om_can tmp[12];
    for(unsigned i=0;i<12;++i) {
        if (!valid(j+i) || !isfinite(q[i]) || q[i]<j[i].lower || q[i]>j[i].upper) return -1;
        double p=j[i].offset+(double)j[i].direction*j[i].ratio*q[i];
        if (!isfinite(p) || fabs(p)>12.57) return -1;
        uint16_t v[4]={quant(p,-12.57,12.57),32767,quant(j[i].kp,0,5000),quant(j[i].kd,0,100)};
        tmp[i].bus=j[i].bus;
        tmp[i].id=0x01000000u | (32767u<<8) | j[i].motor_id; /* zero feed-forward torque */
        for(unsigned k=0;k<4;++k) { tmp[i].data[2*k]=(uint8_t)(v[k]>>8); tmp[i].data[2*k+1]=(uint8_t)v[k]; }
    }
    memcpy(out,tmp,sizeof(tmp)); /* whole batch validated before any output */
    return 0;
}
void om_fault(om_usb *s,uint8_t reason) {
    if(!s->fault) s->fault=reason?reason:1;
    s->ticket_pending=s->lease=0;
}
int om_init(om_usb *s,const uint8_t nonce[16],const uint8_t digest[32],const om_joint *j) {
    memset(s,0,sizeof(*s));
    uint8_t any=0; for(unsigned i=0;i<16;++i) any|=nonce[i];
    if(!any || !j) { om_fault(s,1); return -1; }
    unsigned buses[3]={0,0,0};
    for(unsigned i=0;i<12;++i) {
        if(!valid(j+i)) { om_fault(s,1); return -1; }
        ++buses[j[i].bus];
        for(unsigned k=0;k<i;++k)
            if(j[k].bus==j[i].bus && j[k].motor_id==j[i].motor_id) { om_fault(s,1); return -1; }
    }
    if(buses[0]!=4 || buses[1]!=4 || buses[2]!=4) { om_fault(s,1); return -1; }
    memcpy(s->nonce,nonce,16); memcpy(s->digest,digest,32); memcpy(s->joints,j,sizeof(s->joints));
    s->configured=1; return 0;
}
/* Modular subtraction works across tick wrap for the 50 ms bounded lease. */
static int expired(uint32_t now,uint32_t deadline) { return (uint32_t)(now-deadline)<0x80000000u; }
void om_tick(om_usb *s,uint32_t now) {
    if(s->lease && expired(now,s->lease_deadline)) om_fault(s,5);
    if(s->ticket_pending && expired(now,s->deadline)) s->ticket_pending=0;
}
static void reply(om_usb *s,uint8_t kind,const uint8_t *data,size_t n,uint8_t *out,size_t *len) {
    out[0]='O';out[1]='M';out[2]=1;out[3]=kind;out[4]=(uint8_t)n;out[5]=0;
    memcpy(out+6,s->nonce,16);wr32(out+22,s->sequence);wr32(out+26,s->deadline);
    if(n) memcpy(out+30,data,n);
    wr32(out+30+n,crc(out,30+n)); *len=34+n;
}
static int error(om_usb *s,uint8_t reason,uint8_t *out,size_t *len) {
    om_fault(s,reason); reply(s,255,&s->fault,1,out,len); return -1;
}
int om_feed(om_usb *s,uint8_t byte,uint32_t now,uint8_t *out,size_t *len) {
    *len=0;om_tick(s,now);
    if(s->used>=OM_MAX_FRAME) { s->used=0;return error(s,2,out,len); }
    s->buffer[s->used++]=byte;
    if(s->used<30) return 0;
    uint8_t *p=s->buffer;size_t n=p[4]|((size_t)p[5]<<8);
    if(p[0]!='O'||p[1]!='M'||p[2]!=1||n>64) {s->used=0;return error(s,2,out,len);}
    if(s->used<34+n) return 0;
    s->used=0;
    if(rd32(p+30+n)!=crc(p,30+n)) return error(s,2,out,len);
    uint8_t kind=p[3];
    /* Diagnostics remain readable after a CAN observation fault. They cannot
     * clear a fault, renew a ticket, or actuate. Other commands stay locked. */
    if(kind==5 && s->configured && n==1 && !rd32(p+26) &&
       !memcmp(p+6,s->nonce,16) && s->status) {
        uint8_t payload[64];
        size_t count=s->status(s->status_context,p[30],now,payload);
        if(!count || count>64) return error(s,2,out,len);
        reply(s,133,payload,count,out,len);
        wr32(out+22,rd32(p+22)); /* query correlation, not a command ticket */
        wr32(out+26,0);
        wr32(out+30+count,crc(out,30+count));
        return 1;
    }
    if(!s->configured || (s->fault && !(s->fault==9 && kind==1)))
        return error(s,s->fault?s->fault:1,out,len);
    if(kind==1) {
        uint8_t zero[16]={0}, info[33]={1};
        if(n || memcmp(p+6,zero,16) || rd32(p+22) || rd32(p+26)) return error(s,2,out,len);
        memcpy(info+1,s->digest,32);reply(s,129,info,33,out,len);return 1;
    }
    if(memcmp(p+6,s->nonce,16)) return error(s,3,out,len);
    if(kind==4 && !n) return error(s,7,out,len); /* explicit STOP latches bench session */
    if(kind==2 && !n && !rd32(p+22) && !rd32(p+26)) {
        if(s->sequence==0xffffffffu) return error(s,4,out,len);
        ++s->sequence;s->deadline=now+50u;s->ticket_pending=1;
        /* A new ticket may not renew an existing dry-run lease. */
        reply(s,130,NULL,0,out,len);return 1;
    }
    if(kind==3 && n==48) {
        if(!s->ticket_pending || rd32(p+22)!=s->sequence || rd32(p+26)!=s->deadline || expired(now,s->deadline))
            return error(s,4,out,len);
        float q[12];om_can frames[12];
        for(unsigned i=0;i<12;++i) {uint32_t u=rd32(p+30+4*i);memcpy(q+i,&u,4);}
        if(om_encode_targets(s->joints,q,frames)) return error(s,6,out,len);
        s->ticket_pending=0;s->lease=1;s->lease_deadline=s->deadline;
        uint8_t locked=1;reply(s,131,&locked,1,out,len);return 1;
    }
    return error(s,2,out,len);
}
