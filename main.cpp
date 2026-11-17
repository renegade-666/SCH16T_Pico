#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "SCH16T_Pico.h"
#include <cstdio>

// Example wiring (Pico W): GP16=MISO, GP19=MOSI, GP18=SCK, GP17=CS.
// Optional EXTRESN -> GP20. Sensor board must be powered from 3V3.
static constexpr uint PIN_MISO=16, PIN_CS=17, PIN_SCK=18, PIN_MOSI=19, PIN_RESET=20;

int main() {
    stdio_init_all();
    spi_init(spi0, 10'000'000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    // Set to 1 for SCH16T-K10, or leave 0 for SCH16T-K01.
#define USE_SCH16T_K10 0
#if USE_SCH16T_K10
    SCH16T_K10 imu(spi0, PIN_CS, PIN_RESET);
    SCH16T_sensitivity sensitivity{200,200,6400,6400,6400};
#else
    SCH16T_K01 imu(spi0, PIN_CS, PIN_RESET);
    SCH16T_sensitivity sensitivity{3200,3200,6400,6400,6400};
#endif

    SCH16T_filter filter{68,68,68};
    SCH16T_decimation decimation{4,4};
    int rc=imu.begin(filter,sensitivity,decimation,false);
    printf("SCH16T init: %d\n",rc);
    if(rc!=SCH16T_OK) { while(true) tight_loop_contents(); }

    printf("Serial: %s\n",imu.getSnbr());
    SCH16T_raw_data raw{}; SCH16T_result value{};
    while(true) {
        imu.getData(&raw);
        imu.convertData(&raw,&value);
        printf("gyro [deg/s] % .4f % .4f % .4f | accel [m/s^2] % .4f % .4f % .4f | T %.2f C | frame_error=%d\n",
               value.Rate1[0],value.Rate1[1],value.Rate1[2],
               value.Acc1[0],value.Acc1[1],value.Acc1[2],value.Temp,raw.frame_error?1:0);
        sleep_ms(20);
    }
}
