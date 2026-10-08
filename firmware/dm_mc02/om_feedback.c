#include "om_feedback.h"
#include <string.h>
#include <limits.h>
static void inc(uint32_t *value) { if(*value!=UINT32_MAX) ++*value; }
static uint16_t be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]<<8|p[1]); }
static uint32_t le32(const uint8_t *p) {
    return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;
}
static void put32(uint8_t *p,uint32_t value) {
    for(unsigned k=0;k<4;++k) p[k]=(uint8_t)(value>>(8*k));
}
static void putfloat(uint8_t *p,float value) {
    uint32_t raw;memcpy(&raw,&value,4);put32(p,raw);
}
int om_feedback_init(om_feedback *s,const om_joint *j,uint32_t now) {
    memset(s,0,sizeof(*s));s->started=now;
    if(!j) return -1;
    unsigned buses[3]={0};
    for(unsigned i=0;i<OM_JOINTS;++i) {
        if(j[i].bus>=3 || !j[i].motor_id || j[i].motor_id>127) return -1;
        for(unsigned k=0;k<i;++k)
            if(j[i].bus==j[k].bus && j[i].motor_id==j[k].motor_id) return -1;
        ++buses[j[i].bus];
        s->joint[i].bus=j[i].bus;s->joint[i].motor=j[i].motor_id;
    }
    if(buses[0]!=4 || buses[1]!=4 || buses[2]!=4) return -1;
    s->ready=1;return 0;
}
void om_feedback_fault(om_feedback *s,uint8_t bus,uint8_t reason) {
    if(bus>=3) return;
    s->bus[bus].fault|=reason;
    if(reason&OM_FB_MALFORMED) inc(&s->bus[bus].malformed);
    if(reason&OM_FB_OVERFLOW) inc(&s->bus[bus].overflow);
}
void om_feedback_receive(om_feedback *s,uint8_t bus,uint32_t id,
                         const uint8_t data[8],uint32_t received,uint32_t now) {
    if(!s->ready || bus>=3) return;
    if(!data || id>=0x20000000u) {om_feedback_fault(s,bus,OM_FB_MALFORMED);return;}
    if((uint32_t)(now-received)>=OM_FB_TIMEOUT_MS) {
        om_feedback_fault(s,bus,OM_FB_TIMEOUT);return;
    }
    unsigned kind=id>>24,motor=(id>>8)&255u;
    om_feedback_joint *j=NULL;
    for(unsigned i=0;i<OM_JOINTS;++i)
        if(s->joint[i].bus==bus && s->joint[i].motor==motor) j=&s->joint[i];
    if((id&255u)!=OM_FB_HOST || !j || (kind!=2 && kind!=17 && kind!=21)) {
        inc(&s->bus[bus].ignored);return;
    }
    /* Do not let an older queued state replace a newer state (tick wraps OK). */
    if(kind==2) {
        if((j->seen&1u) && (uint32_t)(received-j->received)>=0x80000000u) {
            om_feedback_fault(s,bus,OM_FB_TIMEOUT);return;
        }
        j->position=be16(data)*25.14f/65535.0f-12.57f;
        j->velocity=be16(data+2)*30.0f/65535.0f-15.0f;
        j->torque=be16(data+4)*240.0f/65535.0f-120.0f;
        j->temperature=be16(data+6)/10.0f;
        j->mode=(uint8_t)((id>>22)&3u);j->motor_fault=(uint8_t)((id>>16)&63u);
        j->received=received;j->seen|=1u;inc(&j->count);
        /* Running/calibration mode is unexpected on this unenabled bench. */
        if(j->motor_fault || j->mode || be16(data)==0 || be16(data)==65535 ||
           be16(data+2)==0 || be16(data+2)==65535)
            om_feedback_fault(s,bus,OM_FB_MOTOR_ERROR);
    } else if(kind==17) {
        if(data[2] || data[3]) {om_feedback_fault(s,bus,OM_FB_MALFORMED);return;}
        j->parameter_index=(uint16_t)(data[0]|(uint16_t)data[1]<<8);
        j->parameter_value=le32(data+4);j->parameter_time=received;
        j->parameter_error=(uint8_t)(id>>16);j->seen|=2u;
        if(j->parameter_error) om_feedback_fault(s,bus,OM_FB_MOTOR_ERROR);
    } else {
        j->fault_bits=le32(data);j->warning_bits=le32(data+4);j->seen|=4u;
        if(j->fault_bits || j->warning_bits) om_feedback_fault(s,bus,OM_FB_MOTOR_ERROR);
    }
    inc(&s->bus[bus].accepted);
}
void om_feedback_tick(om_feedback *s,uint32_t now) {
    if(!s->ready) return;
    for(unsigned i=0;i<OM_JOINTS;++i) {
        const om_feedback_joint *j=&s->joint[i];
        uint32_t since=(j->seen&1u)?j->received:s->started;
        if((uint32_t)(now-since)>=OM_FB_TIMEOUT_MS)
            om_feedback_fault(s,j->bus,OM_FB_TIMEOUT);
    }
}
int om_feedback_faulted(const om_feedback *s) {
    return !s->ready || s->bus[0].fault || s->bus[1].fault || s->bus[2].fault;
}
size_t om_feedback_status(void *context,uint8_t selector,uint32_t now,uint8_t out[64]) {
    om_feedback *s=context;
    if(!s || (selector>=OM_JOINTS && selector!=255)) return 0;
    om_feedback_tick(s,now);
    memset(out,0,64);out[0]=1;out[1]=1;out[2]=s->ready;out[3]=selector;
    if(selector==255) {
        put32(out+4,now);put32(out+8,OM_FB_TIMEOUT_MS);
        for(unsigned b=0;b<3;++b) {
            unsigned at=16+16*b;out[12+b]=s->bus[b].fault;
            put32(out+at,s->bus[b].accepted);put32(out+at+4,s->bus[b].ignored);
            put32(out+at+8,s->bus[b].malformed);put32(out+at+12,s->bus[b].overflow);
        }
    } else {
        const om_feedback_joint *j=&s->joint[selector];
        out[4]=j->bus;out[5]=j->motor;out[6]=j->seen;
        if((j->seen&1u) && (uint32_t)(now-j->received)<OM_FB_TIMEOUT_MS) out[6]|=8u;
        if((j->seen&2u) && (uint32_t)(now-j->parameter_time)<OM_FB_TIMEOUT_MS) out[6]|=16u;
        out[7]=s->bus[j->bus].fault;out[8]=(uint8_t)om_feedback_faulted(s);
        out[9]=j->mode;out[10]=j->motor_fault;out[11]=j->parameter_error;
        put32(out+12,now);put32(out+16,j->received);put32(out+20,j->count);
        putfloat(out+24,j->position);putfloat(out+28,j->velocity);
        putfloat(out+32,j->torque);putfloat(out+36,j->temperature);
        put32(out+40,j->fault_bits);put32(out+44,j->warning_bits);
        out[48]=(uint8_t)j->parameter_index;out[49]=(uint8_t)(j->parameter_index>>8);
        put32(out+52,j->parameter_value);put32(out+56,j->parameter_time);
        put32(out+60,s->bus[j->bus].malformed);
    }
    return 64;
}
