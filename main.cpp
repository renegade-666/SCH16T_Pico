#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "SCH16T_Pico.h"
#include <cstdio>

// Example wiring (Pico W): GP16=MISO, GP19=MOSI, GP18=SCK, GP17=CS.
// Optional EXTRESN -> GP20. Sensor board must be powered from 3V3.
static const uint PIN_MISO = 16, PIN_CS = 17, PIN_SCK = 18, PIN_MOSI = 19, PIN_RESET = 20;
static const uint SPI_BAUDRATE = 10'000'000;
static const uint16_t GYROSCOPE_SENSITIVITY = 3200;
static const uint16_t ACCELEROMETER_SENSITIVITY = 6400;
static const uint16_t LPF_MEDIUM_BANDWIDTH_HZ = 68;
static const uint16_t DECIMATION_NO_DECIMATION = 2;
static const uint32_t DATA_FRAME_DURATION_MS = 20;

int main() {
    stdio_init_all();
    spi_init(spi0, SPI_BAUDRATE);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);

    SCH16T_K01 imu(spi0, PIN_CS, PIN_RESET);
    SCH16T_sensitivity sensitivity{GYROSCOPE_SENSITIVITY, GYROSCOPE_SENSITIVITY, ACCELEROMETER_SENSITIVITY, ACCELEROMETER_SENSITIVITY, ACCELEROMETER_SENSITIVITY};

    SCH16T_filter filter{LPF_MEDIUM_BANDWIDTH_HZ, LPF_MEDIUM_BANDWIDTH_HZ, LPF_MEDIUM_BANDWIDTH_HZ};
    SCH16T_decimation decimation{DECIMATION_NO_DECIMATION, DECIMATION_NO_DECIMATION};
    const bool use_data_ready_interrupt = false;
    int rc = imu.begin(filter, sensitivity, decimation, use_data_ready_interrupt);
    printf("SCH16T init: %d\n", rc);
    if (rc!=SCH16T_OK) { while(true) tight_loop_contents(); }

    printf("Serial: %s\n", imu.getSnbr());
    SCH16T_raw_data raw{};
    SCH16T_result value{};
    while(true) {
        imu.getData(&raw);
        imu.convertData(&raw, &value);
        // gyro [deg/s] + accel [m/s^2]
        printf("gx % .4f gy % .4f gz % .4f ax % .4f ay % .4f az % .4f\n", value.Rate1[0], value.Rate1[1], value.Rate1[2], value.Acc1[0], value.Acc1[1], value.Acc1[2]);
        sleep_ms(DATA_FRAME_DURATION_MS);
    }
}
