// Tasks
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

/*
 * Sources:
 *  Mathematical Model of an IMU - nitinjsanket:
 *      https://nitinjsanket.github.io/tutorials/attitudeest/madgwick#madgwickfilt
 *
 *  Madgwick_Filter - bjohnsonfl:
 *      https://github.com/bjohnsonfl/Madgwick_Filter/blob/master/madgwickFilter.c
 *      - Using for fast-computation of the marg ahrs update
 *      - see my TM4C-Modules/9dof repo to see my SLOW easier to understand version:
 *      https://github.com/Kyojin-743/TM4C-Modules/blob/main/9dof/main.c
 *
 *  Notes:
 *      With the clock at 80Mhz, trying to run the ahrs and pid both at 1kHz, that leaves 80k total clocks in 1kHz
 *
 */

//C-Std Lib.
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

//TM4C-Core
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "asm.h"

//TM4C-Peripheral drivers
#include "nvic.h"
#include "uart0.h"
#include "gpio.h"
#include "i2c1.h"
#include "spi1.h"

//Device Drivers
#include "shell.h"
#include "kernel.h"
#include "tasks.h"
#include "nrf24l01.h"
#include "mpu6050.h"
#include "quaternion.h"
#include "qmc5883p.h"
#include "bme280.h"

#define MPU6050_INT     PORTE,4
//#define QMC5883P_INT    PORTB,5
#define NRF24L01_INT    PORTB,5
#define NRF24L01_CSN    PORTD,1
#define NRF24L01_CE     PORTE,3

#define PIN_0           0x01
#define PIN_1           0x02
#define PIN_2           0x04
#define PIN_3           0x08
#define PIN_4           0x10
#define PIN_5           0x20
#define PIN_6           0x40
#define PIN_7           0x80

//Drone Specific Types
typedef struct { float pitch, roll, yaw; } Attitude;
typedef struct { float x, y, z; } MagData;
typedef struct { float x, y, z; } BaroData;

Attitude attitude;
MagData  mag_data;
BaroData baro_data;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void gpioBIsr(void) {
    if(isPinInterrupt(NRF24L01_INT)) {
        clearPinInterrupt(NRF24L01_INT);
    }
}

void gpioEIsr(void) {
    if(isPinInterrupt(MPU6050_INT)) {
        _postSemaphore(semaphore_mpu_data_ready);

//        readI2c1Register(MPU6050_ADDR, MPU6050_INT_STATUS_R); //clear int for mpu
        clearPinInterrupt(MPU6050_INT);
    }
}

void initTaskHw(void) {

    //COMM PROTOCOLS (uart0, sp1, i2c1)
    initUart0();
    setUart0BaudRate(115200, 80e6);
    initI2c1();
    initSpi1(USE_SSI_RX);
    setSpi1BaudRate(4e6, 80e6);
    setSpi1Mode(0, 0);

    //GPIO (mpu, qmc, nrf interrupts)
    enablePort(PORTB);
    enablePort(PORTD);
    enablePort(PORTE);

//    selectPinDigitalInput(QMC5883P_INT);
    selectPinDigitalInput(MPU6050_INT); //cleared when read INT_STATUS
    selectPinDigitalInput(NRF24L01_INT);
    selectPinPushPullOutput(NRF24L01_CSN);
    selectPinPushPullOutput(NRF24L01_CE);

    selectPinInterruptRisingEdge(MPU6050_INT);
    selectPinInterruptFallingEdge(NRF24L01_INT);

    enablePinInterrupt(MPU6050_INT);
    enablePinInterrupt(NRF24L01_INT);

    //Stopwatch
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R3;
    _delay_cycles(3);

    WTIMER3_CTL_R  &= ~(TIMER_CTL_TAEN);
    WTIMER3_CFG_R   = 0x4;
    WTIMER3_TAMR_R  = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR;
    WTIMER3_TAV_R   = 0;
    WTIMER3_CTL_R  |= TIMER_CTL_TAEN;

    WTIMER3_CTL_R  &= ~(TIMER_CTL_TBEN);
    WTIMER3_CFG_R   = 0x4;
    WTIMER3_TBMR_R  = TIMER_TBMR_TBMR_1_SHOT | TIMER_TBMR_TBCDIR;
    WTIMER3_TBV_R   = 0;
    WTIMER3_CTL_R  |= TIMER_CTL_TBEN;

    //MODULES
    qmc_init();
    mpu_init();
    initBme280();
    nrfInit();
    nrfSetTxMode(1, (uint8_t[]){0x11,0x22,0x33,0x44,0x55});
}

#define COUNT2SEC .0000000125 //80MHz
float deltaSeconds(void) {
    uint32_t counts = WTIMER3_TAV_R;
    WTIMER3_TAV_R = 0;
    return ((float)counts) * COUNT2SEC;
}

Vec3f gyroOffsets(void) {
    uint32_t i;
    MpuData m;
    Vec3f s = {0,0,0};
    for(i = 0; i < 1000; ++i) {
        m = mpu_read();
        s.x += m.g.x;
        s.y += m.g.y;
        s.z += m.g.z;
        waitMicrosecond(2e3);
    }
    s.x /= 1000;
    s.y /= 1000;
    s.z /= 1000;
    return s;
}

Vec3f correct(Vec3f raw, float A[3][3], float b[3]) {
    return (Vec3f) {
        .x=A[0][0] * raw.x + A[0][1] * raw.y + A[0][2] * raw.z + b[0],
        .y=A[1][0] * raw.x + A[1][1] * raw.y + A[1][2] * raw.z + b[1],
        .z=A[2][0] * raw.x + A[2][1] * raw.y + A[2][2] * raw.z + b[2]
    };
}

Vec3f sliding_window(Vec3f window[], Vec3f new_data, uint8_t size, uint8_t *window_idx) {
    window[((*window_idx)++) & (size-1)] = new_data;
    uint8_t i;
    Vec3f v = {};
    for(i = 0; i < size; ++i) {
        v.x += window[i].x;
        v.y += window[i].y;
        v.z += window[i].z;
    }
    v.x /= (float)size;
    v.y /= (float)size;
    v.z /= (float)size;
    return v;
}

#define M_BETA   .02f
#define RAD2DEG  57.2957795131f
#define DEG2RAD  0.0174532925f
//============================================================
//9-DOF MADGWICK
//============================================================
void MadgwickQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

Vec3f MARG_AHRS_update(Quaternion *q_est, const Vec3f *a, const Vec3f *g, const Vec3f *m, float dt_s) {
    float q[4];
    q[0] = q_est->w;
    q[1] = q_est->x;
    q[2] = q_est->y;
    q[3] = q_est->z;

    Quaternion q_prev = *q_est;

    float ax = a->x, ay = a->y, az = a->z,
          gx = g->x * DEG2RAD, gy = g->y * DEG2RAD, gz = g->z * DEG2RAD,
          mx = m->x, my = m->y, mz = m->z;
    MadgwickQuaternionUpdate(q, ax, ay, az, gx, gy, gz, mx, my, mz, dt_s);
    q_est->w = q[0];
    q_est->x = q[1];
    q_est->y = q[2];
    q_est->z = q[3];

    float dot = q_est->w * q_prev.w + q_est->x * q_prev.x + q_est->y * q_prev.y + q_est->z * q_prev.z;
    if (dot < 0.0f) {
        q_est->w = -q_est->w;
        q_est->x = -q_est->x;
        q_est->y = -q_est->y;
        q_est->z = -q_est->z;
    }

    float r11 = 1 - 2*(q_est->y*q_est->y + q_est->z*q_est->z);
    float r21 = 2*(q_est->x*q_est->y - q_est->w*q_est->z);
    float r31 = 2*(q_est->x*q_est->z + q_est->w*q_est->y);
    float r32 = 2*(q_est->y*q_est->z - q_est->w*q_est->x);
    float r33 = 1 - 2*(q_est->x*q_est->x + q_est->y*q_est->y);

    return (Vec3f) {
        RAD2DEG * atan2f(r32, r33),
        RAD2DEG * asinf(-r31),
        RAD2DEG * atan2f(r21, r11)
    };
}
#undef M_BETA
#undef RAD2DEG
#undef DEG2RAD

//============================================================
//6-DOF MADGWICK
//============================================================

#define M_BETA       0.01f
#define RAD2DEG     57.2957795131
#define DEG2RAD      0.0174532925
Vec3f IMU_AHRS_update(Quaternion *q_est, const Vec3f *a, const Vec3f *g, float dt_s) {
    Quaternion q_prev = *q_est;

    //Estimate
    Quaternion q_accel = {0, a->x, a->y, a->z};
    q_accel = q_norm(q_accel);
    Quaternion q_gyro = {0, g->x*.5, g->y*.5, g->z*.5};
    q_gyro = q_scale(q_gyro, DEG2RAD);
    q_gyro = q_mul(q_prev, q_gyro);

    //Objective Function
    float f[3] = {
          2*(q_prev.x*q_prev.z - q_prev.w*q_prev.y) - q_accel.x,
          2*(q_prev.w*q_prev.x + q_prev.y*q_prev.z) - q_accel.y,
          2*(0.5 - q_prev.x*q_prev.x - q_prev.y*q_prev.y) - q_accel.z,
    };

    float j[3][4] =  {
          {-2 * q_prev.y, 2 * q_prev.z, -2 * q_prev.w, 2 * q_prev.x},
          {2 * q_prev.x, 2 * q_prev.w, 2 * q_prev.z, 2 * q_prev.y},
          {0, -4 * q_prev.x, -4 * q_prev.y, 0},
    };

    //Gradiant (J' * F)
    Quaternion q_grad = {
         j[0][0]*f[0] + j[1][0]*f[1] + j[2][0]*f[2],
         j[0][1]*f[0] + j[1][1]*f[1] + j[2][1]*f[2],
         j[0][2]*f[0] + j[1][2]*f[1] + j[2][2]*f[2],
         j[0][3]*f[0] + j[1][3]*f[1] + j[2][3]*f[2]
    };
    q_grad = q_norm(q_grad);
    q_grad = q_scale(q_grad, M_BETA);
    *q_est = q_add(q_prev, q_scale(q_sub(q_gyro, q_grad), dt_s));

    float dot = q_est->w * q_prev.w + q_est->x * q_prev.x + q_est->y * q_prev.y + q_est->z * q_prev.z;
    if (dot < 0.0f) {
        q_est->w = -q_est->w;
        q_est->x = -q_est->x;
        q_est->y = -q_est->y;
        q_est->z = -q_est->z;
    }

    // Rotation matrix (body to world, assuming ZYX order)
    float r11 = 1 - 2*(q_est->y*q_est->y + q_est->z*q_est->z);
    float r21 = 2*(q_est->x*q_est->y - q_est->w*q_est->z);
    float r31 = 2*(q_est->x*q_est->z + q_est->w*q_est->y);
    float r32 = 2*(q_est->y*q_est->z - q_est->w*q_est->x);
    float r33 = 1 - 2*(q_est->x*q_est->x + q_est->y*q_est->y);

    return (Vec3f) {
        RAD2DEG * atan2f(r32, r33),
        RAD2DEG * asinf(-r31),
        RAD2DEG * atan2f(r21, r11)
    };
}
#undef M_BETA
#undef RAD2DEG
#undef DEG2RAD

/* ============================================================================
 *  ESTIMATE ATTITUDE   (1kHz)          (Priority: 0)
 * ============================================================================
 *  Description: This process will read the sensors, update attitude, and run the pid loop for the motors
 *
 *  Control Flow:
 *      Wait on Mpu data ready semaphore (yields to other tasks while waiting), that is posted by interrupt
 *      reads mpu data
 *      polls qmc data ready (no interrupt)
 *      if qmc -> read qmc & marg ahrs
 *      else -> imu_ahrs
 *      pid update (with updated attitude and current setpoint)
 *      send motor control
 *
 *  Notes:
 *      no sleeping as the wait on data will provide the other tasks with time to run
 *      the mpu thus governs the speed in which this task runs (mpu6050 set at 1khz datarate)
 * */

#define GYRO_MSE    0.1
#define PI          3.141592
#define RAD2DEG     180.0/PI
#define WINDOW_SIZE 4
void task_ahrs_pid(void) {
    float mag_A[3][3]   =  {{0.00861610, 0.00195565, 0.00083649},
                           {0.00195565, 10.99483483, -0.22468925},
                           {0.00083649, -0.22468925, 10.5611354}};
    float mag_b [3]     =  {178.46403898, -0.03880254, -0.03374748};
    float accel_A[3][3] =  {{1.00550042, -0.00211106, -0.00102880},
                           {-0.00211106, 1.00552839, 0.00102497},
                           {-0.00102880, 0.00102497, 0.98906820}};
    float accel_b[3]    =  {0.02526037, 0.00277766, 0.01647459};
    Vec3f g_ofs = gyroOffsets();
    Vec3f window_a[WINDOW_SIZE] = {};
    Vec3f window_g[WINDOW_SIZE] = {};
    Vec3f window_m[WINDOW_SIZE] = {};
    uint8_t idx_a = 0, idx_g = 0, idx_m = 0;
    float dt_marg_s = 0.0f;
    Quaternion q_est = {1.0f, 0.0f, 0.0f, 0.0f};

    for(;;) {
        wait(semaphore_mpu_data_ready);
        uint8_t data = readI2c1Register(QMC5883P_ADDR, QMC5883P_STAT_R);

        //Read MPU
        MpuData mpu = mpu_read();
        Vec3f accel = (Vec3f){mpu.a.x, mpu.a.y, mpu.a.z};
        Vec3f gyro  = (Vec3f){mpu.g.x - g_ofs.x, mpu.g.y - g_ofs.y, -(mpu.g.z - g_ofs.z)};
        Vec3f mag;
        accel = correct(accel, accel_A, accel_b);
        accel = sliding_window(window_a, accel, WINDOW_SIZE, &idx_a);
        gyro  = sliding_window(window_g, gyro, WINDOW_SIZE, &idx_g);

        //if MAG ready -> read it
        if(data & QMC5883P_STAT_DRDY) {
            mag = qmc_read();
            float t = mag.x;
            mag.x = mag.y;
            mag.y = -t;
            mag = correct(mag, mag_A, mag_b);
            mag = sliding_window(window_m, mag, WINDOW_SIZE, &idx_m);
        }

        Vec3f euler;
        float dt_s = deltaSeconds();
        //Run the Full 9DOF MARG AHRS Update
        if(data & QMC5883P_STAT_DRDY && !(data & QMC5883P_STAT_DRDY)) {
            euler = MARG_AHRS_update(&q_est, &accel, &gyro, &mag, dt_marg_s);
            dt_marg_s = 0;
        }
        //Run the Partial 6DOF IMU AHRS Update
        else {
            euler = IMU_AHRS_update(&q_est, &accel, &gyro, dt_s);
            //dt_s 'auto' zeros after function scope ends
        }

        //HERE RUN PID

        // pid elevation
        // pid pitch
        // pid roll
        // pid yaw

        //SEND PWM


        char buffer[100];
        usprintf(buffer, "%f,%f,%f\n", euler.x, euler.y, euler.z);
        putsUart0(buffer);

    }//END FOR LOOP
}//END TASK


/* ============================================================================
 * RC INPUT             (50hz)          (Priority: 2)
 * ============================================================================
 *  Description: Receive Controller Input update PID setpoints
 *
 *  Control Flow:
 *      Sleep 20ms (50hz)
 *      read from nrf
 *
 * */
void task_receive_input(void) {
    for(;;) {

    }
}


/* ============================================================================
 * TELEMETRY            (1hz)           (Priority: 2)
 * ============================================================================
 *  Description: Transmit battery health telemetry
 *
 * */
void task_send_telem(void) {
    for(;;) {

    }
}


/* ============================================================================
 * IDLE                 (??hz)          (Priority: 7)
 * ============================================================================
*  Description: Do nothing task
*
*/
void idle(void) {
    for(;;) yield();
}


//============================================================================
// Using for speed (MARG AHRS UPDATE)
//============================================================================

#define M_BETA  0.2f
void MadgwickQuaternionUpdate(float q[4], float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    float beta = M_BETA;
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}
