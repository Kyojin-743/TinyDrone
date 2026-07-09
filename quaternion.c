/*
 * quaternion.c
 *
 *  Created on: Mar 24, 2026
 *      Author: kyojin
 */
#include "quaternion.h"
#include <math.h>

float inv_sqrt(float number )
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;                       // evil floating point bit level hacking
    i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
    return y;
}

inline Quaternion q_mul(Quaternion q1, Quaternion q2) {
    return (Quaternion){
        .w= q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        .x= q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        .y= q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        .z= q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w};
}

inline Quaternion q_scale(Quaternion q1, float scalar) {
    return (Quaternion){
        .w= q1.w * scalar,
        .x= q1.x * scalar,
        .y= q1.y * scalar,
        .z= q1.z * scalar};
}

inline Quaternion q_add(Quaternion q1, Quaternion q2) {
    return (Quaternion){
        .w= q1.w + q2.w,
        .x= q1.x + q2.x,
        .y= q1.y + q2.y,
        .z= q1.z + q2.z};
}

inline Quaternion q_sub(Quaternion q1, Quaternion q2) {
    return (Quaternion){
        .w= q1.w - q2.w,
        .x= q1.x - q2.x,
        .y= q1.y - q2.y,
        .z= q1.z - q2.z};
}


inline float q_mag(Quaternion q1) {
    return sqrtf(q1.w*q1.w + q1.x*q1.x + q1.y*q1.y + q1.z*q1.z);
}

inline Quaternion q_norm(Quaternion q1) {
    float norm = q_mag(q1);
    if(norm == 0.0) return q1;
    return (Quaternion) {
        .w= q1.w/norm,
        .x= q1.x/norm,
        .y= q1.y/norm,
        .z= q1.z/norm,
    };
}

inline Quaternion q_star(Quaternion q1) {
    return (Quaternion) {
        .w=q1.w,
        .x=-q1.w,
        .y=-q1.y,
        .z=-q1.z,
    };
}
