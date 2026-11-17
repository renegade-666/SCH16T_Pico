/*
 * SCH16T_Pico - Raspberry Pi Pico SDK driver for Murata SCH16T-K01/K10.
 *
 * Based on the public smplboards-SCH16T Arduino library by Heng Teng Yi
 * and Murata Electronics Oy source material. BSD-3-Clause terms apply;
 * see LICENSE for notices and disclaimer.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include "hardware/spi.h"

#define SCH16T_OK                 0
#define SCH16T_ERR_NULL_POINTER  -1
#define SCH16T_ERR_INVALID_PARAM -2
#define SCH16T_ERR_SENSOR_INIT   -3
#define SCH16T_ERR_OTHER         -4

struct SCH16T_raw_data {
    int32_t Rate1_raw[3]{};
    int32_t Rate2_raw[3]{};
    int32_t Acc1_raw[3]{};
    int32_t Acc2_raw[3]{};
    int32_t Acc3_raw[3]{};
    int32_t Temp_raw{};
    bool frame_error{};
};
struct SCH16T_result {
    float Rate1[3]{}; float Rate2[3]{};
    float Acc1[3]{};  float Acc2[3]{}; float Acc3[3]{};
    float Temp{};
};
struct SCH16T_status {
    uint16_t Summary{}, Summary_Sat{}, Common{}, Rate_Common{};
    uint16_t Rate_X{}, Rate_Y{}, Rate_Z{}, Acc_X{}, Acc_Y{}, Acc_Z{};
};
struct SCH16T_filter { uint16_t Rate12{68}, Acc12{68}, Acc3{68}; };
struct SCH16T_sensitivity {
    uint16_t Rate1{3200}, Rate2{3200};
    uint16_t Acc1{6400}, Acc2{6400}, Acc3{6400};
};
struct SCH16T_decimation { uint16_t Rate2{4}, Acc2{4}; };
enum SCH16T_axis { SCH16T_AXIS_X=0, SCH16T_AXIS_Y=1, SCH16T_AXIS_Z=2 };

class SCH16T {
public:
    SCH16T(spi_inst_t *spi, uint cs_pin, int reset_pin=-1, uint8_t ta9_8=0,
           uint32_t baudrate=10'000'000);
    virtual ~SCH16T() = default;

    int begin(const SCH16T_filter &filter={}, const SCH16T_sensitivity &sensitivity={},
              const SCH16T_decimation &decimation={}, bool enableDRY=false);
    void reset();
    void sendSPIreset();
    void getData(SCH16T_raw_data *data);
    void getDataDecimated(SCH16T_raw_data *data);
    void getDataAux(SCH16T_raw_data *data);
    void convertData(const SCH16T_raw_data *in, SCH16T_result *out) const;
    void convertDataDecimated(const SCH16T_raw_data *in, SCH16T_result *out) const;
    void convertDataAux(const SCH16T_raw_data *in, SCH16T_result *out) const;
    int setFilters(uint32_t rate_hz, uint32_t acc12_hz, uint32_t acc3_hz);
    int setRateSensDec(uint16_t rate1, uint16_t rate2, uint16_t decimation);
    int setAccSensDec(uint16_t acc1, uint16_t acc2, uint16_t acc3, uint16_t decimation);
    int setDRY(int8_t polarity, bool enable);
    int enableMeas(bool enableSensor, bool setEOI);
    int getStatus(SCH16T_status *status);
    bool verifyStatus(const SCH16T_status *status) const;
    const char *getSnbr();
    bool initialized() const { return initialized_; }

protected:
    virtual bool isValidRateSens(uint32_t value) const;
    virtual uint32_t rateSensToBits(uint32_t value) const;
    virtual uint32_t bitsToRateSens(uint32_t value) const;

private:
    enum : uint64_t {
        READ_RATE_X1=0x0048000000ACULL, READ_RATE_Y1=0x00880000009AULL, READ_RATE_Z1=0x00C80000006DULL,
        READ_ACC_X1=0x0108000000F6ULL, READ_ACC_Y1=0x014800000001ULL, READ_ACC_Z1=0x018800000037ULL,
        READ_ACC_X3=0x01C8000000C0ULL, READ_ACC_Y3=0x02080000002EULL, READ_ACC_Z3=0x0248000000D9ULL,
        READ_RATE_X2=0x0288000000EFULL, READ_RATE_Y2=0x02C800000018ULL, READ_RATE_Z2=0x030800000083ULL,
        READ_ACC_X2=0x034800000074ULL, READ_ACC_Y2=0x038800000042ULL, READ_ACC_Z2=0x03C8000000B5ULL,
        READ_TEMP=0x0408000000B1ULL,
        READ_STAT_SUM=0x05080000001CULL, READ_STAT_SUM_SAT=0x0548000000EBULL,
        READ_STAT_COM=0x0588000000DDULL, READ_STAT_RATE_COM=0x05C80000002AULL,
        READ_STAT_RATE_X=0x0608000000C4ULL, READ_STAT_RATE_Y=0x064800000033ULL,
        READ_STAT_RATE_Z=0x068800000005ULL, READ_STAT_ACC_X=0x06C8000000F2ULL,
        READ_STAT_ACC_Y=0x070800000069ULL, READ_STAT_ACC_Z=0x07480000009EULL,
        READ_FILT_RATE=0x0948000000FAULL, READ_FILT_ACC12=0x0988000000CCULL,
        READ_FILT_ACC3=0x09C80000003BULL, READ_RATE_CTRL=0x0A08000000D5ULL,
        READ_ACC12_CTRL=0x0A4800000022ULL, READ_ACC3_CTRL=0x0A8800000014ULL,
        READ_MODE_CTRL=0x0D4800000010ULL, READ_USER_IF_CTRL=0x0CC80000007CULL,
        READ_SN_ID1=0x0F4800000065ULL, READ_SN_ID2=0x0F8800000053ULL, READ_SN_ID3=0x0FC8000000A4ULL,
        SET_FILT_RATE=0x0968000000ULL, SET_FILT_ACC12=0x09A8000000ULL, SET_FILT_ACC3=0x09E8000000ULL,
        SET_RATE_CTRL=0x0A28000000ULL, SET_ACC12_CTRL=0x0A68000000ULL,
        SET_ACC3_CTRL=0x0AA8000000ULL, SET_MODE_CTRL=0x0D68000000ULL,
        SET_USER_IF_CTRL=0x0CE8000000ULL, SOFTRESET=0x0DA800000AC3ULL
    };

    uint64_t transfer(uint64_t frame);
    uint64_t addressed(uint64_t request) const;
    uint64_t addressedNoCrc(uint64_t request) const;
    uint64_t writeRegister(uint64_t command40, uint32_t data20);
    uint8_t crc8(uint64_t frame) const;
    bool frameError(uint64_t frame) const;
    static int32_t signed20(uint64_t frame);
    static uint16_t data16(uint64_t frame);
    static bool validDecimation(uint32_t value);
    static uint32_t decimationBits(uint32_t value);
    static uint32_t accSensBits(uint32_t value);
    static uint32_t rateFilterBits(uint32_t value);
    static uint32_t accFilterBits(uint32_t value);
    void readVector(uint64_t first, uint64_t second, uint64_t third,
                    int32_t values[3], bool &error);

    spi_inst_t *spi_;
    uint cs_, reset_;
    uint8_t ta9_8_;
    uint32_t baudrate_;
    uint16_t sens_rate1_{3200}, sens_rate2_{3200};
    uint16_t sens_acc1_{6400}, sens_acc2_{6400}, sens_acc3_{6400};
    bool initialized_{false};
};

class SCH16T_K01 : public SCH16T {
public:
    using SCH16T::SCH16T;
};
class SCH16T_K10 : public SCH16T {
public:
    using SCH16T::SCH16T;
protected:
    bool isValidRateSens(uint32_t value) const override;
    uint32_t rateSensToBits(uint32_t value) const override;
    uint32_t bitsToRateSens(uint32_t value) const override;
};
