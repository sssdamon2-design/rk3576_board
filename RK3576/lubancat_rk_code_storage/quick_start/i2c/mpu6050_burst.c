#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define MPU_ADDR 0x68
#define SAMPLE_COUNT 20

// 向 MPU6050 指定寄存器写入一个字节
static int write_reg(int fd, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};

    struct i2c_msg msg = {
        .addr  = MPU_ADDR,
        .flags = 0,
        .len   = 2,
        .buf   = tx
    };

    struct i2c_rdwr_ioctl_data req = {
        .msgs  = &msg,
        .nmsgs = 1
    };

    return ioctl(fd, I2C_RDWR, &req) == 1 ? 0 : -1;
}

// 从 0x3B 开始连续读取 14 字节
static int read_burst(int fd, uint8_t data[14])
{
    uint8_t reg = 0x3B;

    struct i2c_msg msgs[2] = {
        {
            .addr  = MPU_ADDR,
            .flags = 0,
            .len   = 1,
            .buf   = &reg
        },
        {
            .addr  = MPU_ADDR,
            .flags = I2C_M_RD,
            .len   = 14,
            .buf   = data
        }
    };

    struct i2c_rdwr_ioctl_data req = {
        .msgs  = msgs,
        .nmsgs = 2
    };

    return ioctl(fd, I2C_RDWR, &req) == 2 ? 0 : -1;
}

// 两个字节组合为有符号 16 位数据
static int16_t to_s16(const uint8_t *p)
{
    int32_t value = ((int32_t)p[0] << 8) | p[1];

    if (value >= 32768)
        value -= 65536;

    return (int16_t)value;
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s /dev/i2c-8\n", argv[0]);
        return 1;
    }

    int fd = open(argv[1], O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    // 初始化 MPU6050
    if (write_reg(fd, 0x6B, 0x00) != 0 ||
        write_reg(fd, 0x19, 0x07) != 0 ||
        write_reg(fd, 0x1A, 0x06) != 0 ||
        write_reg(fd, 0x1B, 0x00) != 0 ||
        write_reg(fd, 0x1C, 0x00) != 0) {

        fprintf(stderr, "MPU6050 initialization failed\n");
        close(fd);
        return 1;
    }

    sleep(1);

    long sum_x = 0;
    long sum_y = 0;
    long sum_z = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        uint8_t data[14];

        if (read_burst(fd, data) != 0) {
            fprintf(stderr, "MPU6050 burst read failed\n");
            close(fd);
            return 1;
        }

        int16_t ax = to_s16(&data[0]);
        int16_t ay = to_s16(&data[2]);
        int16_t az = to_s16(&data[4]);

        int16_t gx = to_s16(&data[8]);
        int16_t gy = to_s16(&data[10]);
        int16_t gz = to_s16(&data[12]);

        sum_x += ax;
        sum_y += ay;
        sum_z += az;

        printf("%02d | ACC: %6d %6d %6d | GYRO: %6d %6d %6d\n",
               i + 1, ax, ay, az, gx, gy, gz);

        struct timespec delay = {
            .tv_sec = 0,
            .tv_nsec = 100000000L
        };

        nanosleep(&delay, NULL);
    }

    printf("\nAverage raw:\n");
    printf("X = %.1f\n", sum_x / (double)SAMPLE_COUNT);
    printf("Y = %.1f\n", sum_y / (double)SAMPLE_COUNT);
    printf("Z = %.1f\n", sum_z / (double)SAMPLE_COUNT);

    close(fd);
    return 0;
}
