#ifndef INVERSE_H
#define INVERSE_H
#include <iostream>
#include <unistd.h>
#include <cmath>
#include "../../src/DynamixelWorkbench.h"

// inverse kinematics quadruped(4 leged)
class inverse
{
private:
    const float c = 21.70; // coxa dalam mm
    const float f = 65.00; // femur dalam mm
    const float t = 76.95; // tibia dalam mm

protected:
#define leg(str) (str == "L1" ? 0 : str == "L2" ? 1 \
                                : str == "R1"   ? 2 \
                                : str == "R2"   ? 3 \
                                                : -1)

#define toRad(theta) (theta * M_PI / 180)
#define toDeg(theta) (theta * 180 / M_PI)

#define L1 0
#define L2 1
#define R1 2
#define R2 3
#define xx -1

#define memori_data 25 // jumlah memori per koordinat per kaki

    DynamixelWorkbench dxl_wb;

    // variabel data titik trjektori
    int jlh_data;

    int32_t posisi_servo[15];
    const uint8_t handler_index = 0;

    // Simpan Data IK
    float array_p[4][3][memori_data]; //[l1,l2,r1,r2][x,y,z][panjang data]

    // variabel koordinat inverse kinematics (current inverse position)
    float c_inv_p[4][3]; //[l1,l2,r1,r2][x,y,z]
    // new inverse position
    float n_inv_p[4][3]; //[l1,l2,r1,r2][x,y,z]

private:
    // pola trjektori parabola
    void polinomial_trj(uint8_t dxl_id, float pol[4][3]);

public:
    // inverse kinematics
    void inverse_s(uint8_t dxl_id, float x, float y, float z);

protected:
    // trjektori langkah
    void trj_langkah(uint8_t dxl_id, float x0, float y0, float z0,
                     float x1, float y1, float zp);
    // trjektori gesek
    void trj_lurus(uint8_t dxl_id, float x0, float y0, float z0,
                   float x1, float y1, float z1);
    // eksekusi trjektori
    void trj_start(uint8_t id_kakiL1, uint8_t id_kakiL2,
                   uint8_t id_kakiR1, uint8_t id_kakiR2, int dly_trj);

    // eksekusi trjektori
    void start_creep(int dly_trj);
};
#endif