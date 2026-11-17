#include "SCH16T_Pico.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstring>
#include <cstdio>

SCH16T::SCH16T(spi_inst_t *spi, uint cs_pin, int reset_pin, uint8_t ta9_8, uint32_t baudrate)
    : spi_(spi), cs_(cs_pin), reset_(reset_pin < 0 ? 0xffffffffu : static_cast<uint>(reset_pin)),
      ta9_8_(ta9_8 & 0x03), baudrate_(baudrate) {}

uint8_t SCH16T::crc8(uint64_t frame) const {
    uint64_t data = frame & 0xFFFFFFFFFF00ULL;
    uint8_t crc = 0xFF;
    for (int i=47; i>=0; --i) {
        uint8_t bit = (data >> i) & 1;
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x2F) ^ bit
                           : static_cast<uint8_t>((crc << 1) | bit);
    }
    return crc;
}

uint64_t SCH16T::addressed(uint64_t request) const {
    uint64_t out = (request | (static_cast<uint64_t>(ta9_8_) << 46)) & 0xFFFFFFFFFF00ULL;
    return out | crc8(out);
}
uint64_t SCH16T::addressedNoCrc(uint64_t request) const {
    return (request | (static_cast<uint64_t>(ta9_8_) << 38)) & 0xFFFFFFFFFFULL;
}

uint64_t SCH16T::transfer(uint64_t frame) {
    uint8_t tx[6], rx[6]{};
    for (unsigned i=0; i<6; ++i) tx[i] = static_cast<uint8_t>(frame >> ((5-i)*8));
    gpio_put(cs_, 0);
    spi_write_read_blocking(spi_, tx, rx, 6);
    gpio_put(cs_, 1);
    uint64_t result=0;
    for (uint8_t b : rx) result = (result << 8) | b;
    return result;
}

uint64_t SCH16T::writeRegister(uint64_t command40, uint32_t data20) {
    uint64_t frame = addressedNoCrc(command40) | (data20 & 0xFFFFF);
    frame <<= 8;
    return transfer(frame | crc8(frame));
}

void SCH16T::reset() {
    if (reset_ != 0xffffffffu) {
        gpio_put(reset_, 0); sleep_ms(2); gpio_put(reset_, 1);
    }
}
void SCH16T::sendSPIreset() { transfer(addressed(SOFTRESET)); }

bool SCH16T::validDecimation(uint32_t v) { return v==2 || v==4 || v==8 || v==16 || v==32; }
uint32_t SCH16T::decimationBits(uint32_t v) {
    switch(v) { case 2:return 0; case 4:return 1; case 8:return 2; case 16:return 3; case 32:return 4; default:return 0; }
}
uint32_t SCH16T::accSensBits(uint32_t v) {
    switch(v) { case 3200:return 1; case 6400:return 2; case 12800:return 3; case 25600:return 4; default:return 0; }
}
bool SCH16T::isValidRateSens(uint32_t v) const { return v==1600 || v==3200 || v==6400; }
uint32_t SCH16T::rateSensToBits(uint32_t v) const {
    switch(v) { case 1600:return 2; case 3200:return 3; case 6400:return 4; default:return 1; }
}
uint32_t SCH16T::bitsToRateSens(uint32_t v) const {
    switch(v) { case 2:return 1600; case 3:return 3200; case 4:return 6400; default:return 0; }
}
bool SCH16T_K10::isValidRateSens(uint32_t v) const { return v==100 || v==200 || v==400; }
uint32_t SCH16T_K10::rateSensToBits(uint32_t v) const {
    switch(v) { case 100:return 2; case 200:return 3; case 400:return 4; default:return 1; }
}
uint32_t SCH16T_K10::bitsToRateSens(uint32_t v) const {
    switch(v) { case 2:return 100; case 3:return 200; case 4:return 400; default:return 0; }
}
uint32_t SCH16T::rateFilterBits(uint32_t v) {
    switch(v) { case 13:return 0x092; case 30:return 0x049; case 68:return 0x000; case 235:return 0x16D; case 280:return 0x0DB; case 370:return 0x124; case 0:return 0x1FF; default:return 0xFFFFFFFF; }
}
uint32_t SCH16T::accFilterBits(uint32_t v) {
    switch(v) { case 13:return 0x092; case 30:return 0x049; case 68:return 0x000; case 210:return 0x16D; case 240:return 0x0DB; case 290:return 0x124; case 0:return 0x1FF; default:return 0xFFFFFFFF; }
}

int SCH16T::setFilters(uint32_t r, uint32_t a12, uint32_t a3) {
    uint32_t rb=rateFilterBits(r), ab=accFilterBits(a12), a3b=accFilterBits(a3);
    if(rb==0xFFFFFFFF || ab==0xFFFFFFFF || a3b==0xFFFFFFFF) return SCH16T_ERR_INVALID_PARAM;
    writeRegister(SET_FILT_RATE,rb); writeRegister(SET_FILT_ACC12,ab); writeRegister(SET_FILT_ACC3,a3b);
    return SCH16T_OK;
}
int SCH16T::setRateSensDec(uint16_t r1,uint16_t r2,uint16_t dec) {
    if(!isValidRateSens(r1)||!isValidRateSens(r2)||!validDecimation(dec)) return SCH16T_ERR_INVALID_PARAM;
    sens_rate1_=r1; sens_rate2_=r2;
    uint32_t d=decimationBits(dec);
    uint32_t data=rateSensToBits(r1); data=(data<<3)|rateSensToBits(r2);
    data=(data<<3)|d; data=(data<<3)|d; data=(data<<3)|d;
    writeRegister(SET_RATE_CTRL,data); return SCH16T_OK;
}
int SCH16T::setAccSensDec(uint16_t a1,uint16_t a2,uint16_t a3,uint16_t dec) {
    if(!accSensBits(a1)||!accSensBits(a2)||!accSensBits(a3)||!validDecimation(dec)) return SCH16T_ERR_INVALID_PARAM;
    sens_acc1_=a1; sens_acc2_=a2; sens_acc3_=a3;
    uint32_t d=decimationBits(dec);
    uint32_t data=accSensBits(a1); data=(data<<3)|accSensBits(a2);
    data=(data<<3)|d; data=(data<<3)|d; data=(data<<3)|d;
    writeRegister(SET_ACC12_CTRL,data);
    writeRegister(SET_ACC3_CTRL,accSensBits(a3));
    return SCH16T_OK;
}
int SCH16T::setDRY(int8_t polarity,bool enable) {
    if(polarity < -1 || polarity > 1) return SCH16T_ERR_INVALID_PARAM;
    transfer(addressed(READ_USER_IF_CTRL));
    uint64_t response=transfer(addressed(READ_USER_IF_CTRL));
    uint32_t data=static_cast<uint32_t>((response >> 8) & 0xFFFFF);
    if(polarity==0) data &= ~0x40u; else if(polarity==1) data |= 0x40u;
    if(enable) data |= 0x20u; else data &= ~0x20u;
    writeRegister(SET_USER_IF_CTRL,data); return SCH16T_OK;
}
int SCH16T::enableMeas(bool enable,bool eoi) {
    uint32_t data=(enable?1u:0u)|(eoi?2u:0u);
    writeRegister(SET_MODE_CTRL,data); return SCH16T_OK;
}

int SCH16T::begin(const SCH16T_filter &f,const SCH16T_sensitivity &s,const SCH16T_decimation &d,bool dry) {
    if(!spi_ || ta9_8_>3 || baudrate_==0) return SCH16T_ERR_INVALID_PARAM;
    gpio_init(cs_); gpio_set_dir(cs_,GPIO_OUT); gpio_put(cs_,1);
    if(reset_!=0xffffffffu) { gpio_init(reset_); gpio_set_dir(reset_,GPIO_OUT); gpio_put(reset_,1); }
    spi_init(spi_,baudrate_);
    // SPI0/SPI1 pin mux is selected by the application with gpio_set_function().
    spi_set_format(spi_,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
    reset(); sleep_ms(10); sendSPIreset(); sleep_ms(32);
    if(setFilters(f.Rate12,f.Acc12,f.Acc3)!=SCH16T_OK) return SCH16T_ERR_INVALID_PARAM;
    if(setRateSensDec(s.Rate1,s.Rate2,d.Rate2)!=SCH16T_OK) return SCH16T_ERR_INVALID_PARAM;
    if(setAccSensDec(s.Acc1,s.Acc2,s.Acc3,d.Acc2)!=SCH16T_OK) return SCH16T_ERR_INVALID_PARAM;
    if(setDRY(0,dry)!=SCH16T_OK) return SCH16T_ERR_OTHER;
    enableMeas(true,false); sleep_ms(215);
    SCH16T_status st{}; getStatus(&st); enableMeas(true,true); sleep_ms(3);
    getStatus(&st); getStatus(&st);
    initialized_=verifyStatus(&st);
    return initialized_ ? SCH16T_OK : SCH16T_ERR_SENSOR_INIT;
}

uint16_t SCH16T::data16(uint64_t f) { return static_cast<uint16_t>((f>>8)&0xFFFF); }
int32_t SCH16T::signed20(uint64_t f) {
    int32_t value = static_cast<int32_t>((f >> 8) & 0xFFFFF);
    if (value & 0x80000) value |= ~0xFFFFF;
    return value;
}
bool SCH16T::frameError(uint64_t f) const { return (f & 0x001E00000000ULL)!=0; }
void SCH16T::readVector(uint64_t first,uint64_t second,uint64_t third,int32_t v[3],bool &err) {
    transfer(addressed(first));
    uint64_t a=transfer(addressed(second));
    uint64_t b=transfer(addressed(third));
    v[0]=signed20(a); v[1]=signed20(b);
    (void)err;
}

void SCH16T::getData(SCH16T_raw_data *d) {
    if(!d) return;
    uint64_t r[7];
    transfer(addressed(READ_RATE_X1));
    r[0]=transfer(addressed(READ_RATE_Y1)); r[1]=transfer(addressed(READ_RATE_Z1));
    r[2]=transfer(addressed(READ_ACC_X1)); r[3]=transfer(addressed(READ_ACC_Y1));
    r[4]=transfer(addressed(READ_ACC_Z1)); r[5]=transfer(addressed(READ_TEMP)); r[6]=transfer(addressed(READ_TEMP));
    d->frame_error=false; for(auto x:r) d->frame_error |= frameError(x);
    for(int i=0;i<3;i++) { d->Rate1_raw[i]=signed20(r[i]); d->Acc1_raw[i]=signed20(r[i+3]); }
    d->Temp_raw=signed20(r[6])>>4;
}
void SCH16T::getDataDecimated(SCH16T_raw_data *d) {
    if (!d) return;
    uint64_t r[7];
    transfer(addressed(READ_RATE_X2));
    r[0]=transfer(addressed(READ_RATE_Y2)); r[1]=transfer(addressed(READ_RATE_Z2));
    r[2]=transfer(addressed(READ_ACC_X2)); r[3]=transfer(addressed(READ_ACC_Y2));
    r[4]=transfer(addressed(READ_ACC_Z2)); r[5]=transfer(addressed(READ_TEMP)); r[6]=transfer(addressed(READ_TEMP));
    d->frame_error=false; for(auto x:r) d->frame_error |= frameError(x);
    for(int i=0;i<3;i++) { d->Rate2_raw[i]=signed20(r[i]); d->Acc2_raw[i]=signed20(r[i+3]); }
    d->Temp_raw=signed20(r[6])>>4;
}
void SCH16T::getDataAux(SCH16T_raw_data *d) {
    if (!d) return;
    uint64_t r[4];
    transfer(addressed(READ_ACC_X3));
    r[0]=transfer(addressed(READ_ACC_Y3)); r[1]=transfer(addressed(READ_ACC_Z3));
    r[2]=transfer(addressed(READ_TEMP)); r[3]=transfer(addressed(READ_TEMP));
    d->frame_error=false; for(auto x:r) d->frame_error |= frameError(x);
    for (int i=0; i<3; i++) d->Acc3_raw[i] = signed20(r[i]);
    d->Temp_raw = signed20(r[3]) >> 4;
}
void SCH16T::convertData(const SCH16T_raw_data *i, SCH16T_result *o) const {
    if (!i || !o) return;
    for (int a=0; a<3; a++) {
        o->Rate1[a] = float(i->Rate1_raw[a]) / sens_rate1_;
        o->Acc1[a] = float(i->Acc1_raw[a]) / sens_acc1_;
    }
    o->Temp = float(i->Temp_raw) / 100.0f;
}
void SCH16T::convertDataDecimated(const SCH16T_raw_data *i, SCH16T_result *o) const {
    if (!i || !o) return;
    for (int a=0; a<3; a++) {
        o->Rate2[a] = float(i->Rate2_raw[a]) / sens_rate2_;
        o->Acc2[a] = float(i->Acc2_raw[a]) / sens_acc2_;
    }
    o->Temp = float(i->Temp_raw) / 100.0f;
}
void SCH16T::convertDataAux(const SCH16T_raw_data *i, SCH16T_result *o) const {
    if (!i || !o) return;
    for (int a=0; a<3; a++) o->Acc3[a] = float(i->Acc3_raw[a]) / sens_acc3_;
    o->Temp = float(i->Temp_raw) / 100.0f;
}

int SCH16T::getStatus(SCH16T_status *s) {
    if(!s) return SCH16T_ERR_NULL_POINTER;
    transfer(addressed(READ_STAT_SUM));
    s->Summary=data16(transfer(addressed(READ_STAT_SUM_SAT)));
    s->Summary_Sat=data16(transfer(addressed(READ_STAT_COM)));
    s->Common=data16(transfer(addressed(READ_STAT_RATE_COM)));
    s->Rate_Common=data16(transfer(addressed(READ_STAT_RATE_X)));
    s->Rate_X=data16(transfer(addressed(READ_STAT_RATE_Y)));
    s->Rate_Y=data16(transfer(addressed(READ_STAT_RATE_Z)));
    s->Rate_Z=data16(transfer(addressed(READ_STAT_ACC_X)));
    s->Acc_X=data16(transfer(addressed(READ_STAT_ACC_Y)));
    s->Acc_Y=data16(transfer(addressed(READ_STAT_ACC_Z)));
    s->Acc_Z=data16(transfer(addressed(READ_STAT_ACC_Z)));
    return SCH16T_OK;
}
bool SCH16T::verifyStatus(const SCH16T_status *s) const {
    if(!s) return false;
    return s->Summary==0xFFFF && s->Summary_Sat==0xFFFF && s->Common==0xFFFF && s->Rate_Common==0xFFFF &&
        s->Rate_X==0xFFFF && s->Rate_Y==0xFFFF && s->Rate_Z==0xFFFF && s->Acc_X==0xFFFF && s->Acc_Y==0xFFFF && s->Acc_Z==0xFFFF;
}
const char *SCH16T::getSnbr() {
    static char out[15]; transfer(addressed(READ_SN_ID1));
    uint16_t id1=data16(transfer(addressed(READ_SN_ID2)));
    uint16_t id2=data16(transfer(addressed(READ_SN_ID3)));
    uint16_t id3=data16(transfer(addressed(READ_SN_ID3)));
    std::snprintf(out,sizeof(out),"%05u%01X%04X",id2,id1&0xF,id3); return out;
}
