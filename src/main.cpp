#include <mbed.h>
#include <math.h>
#include <arm_math.h>
#include <queue.h>
#include <time.h>
// =================================================
// * Recitation 5: SPI and Gyroscope *
// =================================================

// TODOs:
// [1] Get started with an SPI object instance and connect to the Gyroscope!
// [2] Read the XYZ axis from the Gyroscope and Visualize on the Teleplot. 
// [3] Fetching Data from the sensor via Polling vs Interrupt ?

// Define control register addresses and their configurations
// Define control register addresses and their configurations
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000
#define OUT_X_L 0x28
#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)
#define FILTER_COEFFICIENT 0.1f // Adjust this value as needed

EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

float gx_1s [10];
float gy_1s [10];
float gz_1s [10];
int time_counter = 0;

float_t convertAnguarToFrequency(float gx, float gy, float gz){
    float_t omega = sqrt(pow(gx,2)+ pow(gy,2)+ pow(gz,2));
    // float_t trans = fft(omega);
    float_t frequency = omega/(2* M_PI);
    return frequency ;
}

void storeAngularVelocity(float gx, float gy, float gz, int time){
  gx_1s[time]= gx;
  gy_1s[time]= gy;
  gz_1s[time]= gz;
}

// // SPI callback function
// void spi_cb(int event) {
//     flags.set(SPI_FLAG);
// }

// Data ready callback function
// void data_cb() {
//     flags.set(DATA_READY_FLAG);
// }

int main()
{
    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI.
    uint8_t write_buf[32], read_buf[32];

    // Interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    // Configure SPI format and frequency.
    spi.format(8, 3);
    spi.frequency(1000000);

    // Configure CTRL_REG1 register.
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register.
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);
    while(1){
        if (time_counter==10){
          printf("\nTremor detect");
          time_counter=0;
        }
        uint16_t raw_gx, raw_gy, raw_gz;
        float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;
        float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f;
        float gx, gy, gz, fq;

        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);
        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);
        gx = raw_gx * SCALING_FACTOR;
        gy = raw_gy * SCALING_FACTOR;
        gz = raw_gz * SCALING_FACTOR;
        //Apply Simple low-pass filter        
        filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
        filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
        filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;
        high_pass_gx = gx - filtered_gx;
        high_pass_gy = gy - filtered_gy;
        high_pass_gz = gz - filtered_gz;
        // Print the raw values for debugging 
        printf("RAW -> \t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

            printf(">x_axis: %d|g \n", raw_gx);
            printf(">y_axis: %d|g \n", raw_gy);
            printf(">z_axis: %d|g \n", raw_gz);

        // Convert raw data to actual values using a scaling factor
        storeAngularVelocity(gx,gy,gz,time_counter);
        float frequency = convertAnguarToFrequency(high_pass_gx,high_pass_gy,high_pass_gz);
        // Print the actual values
        printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t fq%4.5f \n", 
        high_pass_gx, high_pass_gy,high_pass_gz, frequency);
        time_counter +=1;
        thread_sleep_for(100);
    }

    }























