#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "mpu6050.h"
#include "Fusion.h"

// parâmetros I²C e AHRS
#define SAMPLE_PERIOD    (0.01f)     // 10 ms
#define I2C_SDA_GPIO     4
#define I2C_SCL_GPIO     5
#define MPU_ADDRESS      0x68

// protocolo binário
#define SYNC_BYTE        0xFF
#define AXIS_X           0   // roll → X do mouse
#define AXIS_Y           1   // pitch → Y do mouse
#define AXIS_CLICK       2   // evento “click”

// escala (graus → unidades de mouse)
#define SCALE            (2.0f)

// limiar de jerk para clique (variação de aceleração no eixo Z)
#define JERK_THRESHOLD   (8.5f)            // ajuste pra calibrar “brusquidão”
// tempo mínimo entre dois cliques
#define CLICK_DEBOUNCE   pdMS_TO_TICKS(500) // 500 ms

typedef struct {
    float roll, pitch, yaw;
    bool  click;
} PointerMsg;

static QueueHandle_t xQueuePos;

// reset da MPU6050
static void mpu6050_reset() {
    uint8_t buf[] = { 0x6B, 0x00 };
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

// leitura raw de accel/gyro/temp
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6], reg;
    // acelerômetro
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg,1,true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer,6,false);
    for(int i=0;i<3;i++) accel[i] = (buffer[2*i]<<8)|buffer[2*i+1];
    // giroscópio
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg,1,true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer,6,false);
    for(int i=0;i<3;i++) gyro[i] = (buffer[2*i]<<8)|buffer[2*i+1];
    // temperatura
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg,1,true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer,2,false);
    *temp = (buffer[0]<<8)|buffer[1];
}

// tarefa principal: leitura, fusão e detecção de click via jerk no eixo Z
static void mpu6050_task(void *p) {
    // inicializa I²C
    i2c_init(i2c_default, 400000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    mpu6050_reset();

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    float last_accel_z   = 0.0f;
    bool  last_above_z   = false;
    TickType_t last_click_tick = 0;

    int16_t accel[3], gyro[3], temp;
    while (1) {
        mpu6050_read_raw(accel, gyro, &temp);

        FusionVector fg = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f
        };
        FusionVector fa = {
            .axis.x = accel[0] / 16384.0f,
            .axis.y = accel[1] / 16384.0f,
            .axis.z = accel[2] / 16384.0f
        };

        // calcula jerk no eixo Z
        float jerk_z = (fa.axis.z - last_accel_z) / SAMPLE_PERIOD;
        bool  above_z = (jerk_z > JERK_THRESHOLD);
        last_accel_z = fa.axis.z;

        // fusão AHRS e Euler
        FusionAhrsUpdateNoMagnetometer(&ahrs, fg, fa, SAMPLE_PERIOD);
        FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // rising‐edge de jerk_z + debounce
        TickType_t now = xTaskGetTickCount();
        bool click_event = false;
        if (above_z && !last_above_z && (now - last_click_tick) > CLICK_DEBOUNCE) {
            click_event     = true;
            last_click_tick = now;
        }
        last_above_z = above_z;

        // empacota e envia para a fila
        PointerMsg msg = {
            .roll  = e.angle.roll,
            .pitch = e.angle.pitch,
            .yaw   = e.angle.yaw,
            .click = click_event
        };
        xQueueSend(xQueuePos, &msg, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// tarefa de envio UART (stdout)
static void uart_task(void *p) {
    PointerMsg msg;
    while (1) {
        if (xQueueReceive(xQueuePos, &msg, portMAX_DELAY)) {
            uint8_t buf[4];
            int16_t v;
            // X ← roll
            v = (int16_t)(msg.pitch * SCALE);
            buf[0]=SYNC_BYTE; buf[1]=AXIS_X;
            buf[2]=(v>>8)&0xFF; buf[3]=v&0xFF;
            fwrite(buf,1,4,stdout);
            // Y ← pitch
            v = (int16_t)(msg.roll * SCALE);
            buf[0]=SYNC_BYTE; buf[1]=AXIS_Y;
            buf[2]=(v>>8)&0xFF; buf[3]=v&0xFF;
            fwrite(buf,1,4,stdout);
            // CLICK
            if (msg.click) {
                buf[0]=SYNC_BYTE; buf[1]=AXIS_CLICK;
                buf[2]=0; buf[3]=1;
                fwrite(buf,1,4,stdout);
            }
            fflush(stdout);
        }
    }
}

int main() {
    stdio_init_all();
    xQueuePos = xQueueCreate(16, sizeof(PointerMsg));
    xTaskCreate(mpu6050_task, "MPU", 4096, NULL, 2, NULL);
    xTaskCreate(uart_task,    "UART",4096, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1);
    return 0;
}