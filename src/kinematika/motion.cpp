#include "../../include/kinematika/motion.h"

// menurunkan lengan gripper
void motion::capit()
{
    siap(820, 0, 512);
}

// standby mode kaki membentuk X terhadap body
void motion::siap(uint a, uint b, uint c)
{
    // posisi_servo[12] = a; // 820; // 204
    // posisi_servo[13] = b;
    // posisi_servo[14] = c;        // 820; // 204
    inverse_s(L1, -75, 45, -65); // x1,y1,zp
    inverse_s(L2, -75, -45, -65);
    inverse_s(R1, 75, 45, -65);
    inverse_s(R2, 75, -45, -65);
}

// pola langkah segitiga
void motion::tripod_gait(int sekuen, int dly_trj)
{
    // ganjil
    if (sekuen % 2 != 0)
    {
        // step kaki kanan
        // langkah kaki kanan
        trj_langkah(R1, c_inv_p[R1][0], c_inv_p[R1][1], c_inv_p[R1][2], n_inv_p[R1][0], n_inv_p[R1][1], titik_puncak);
        trj_langkah(L2, c_inv_p[L2][0], c_inv_p[L2][1], c_inv_p[L2][2], n_inv_p[L2][0], n_inv_p[L2][1], titik_puncak);

        // geser kaki kiri
        trj_lurus(L1, c_inv_p[L1][0], c_inv_p[L1][1], c_inv_p[L1][2], n_inv_p[L1][0], n_inv_p[L1][1], n_inv_p[L1][2]);
        trj_lurus(R2, c_inv_p[R2][0], c_inv_p[R2][1], c_inv_p[R2][2], n_inv_p[R2][0], n_inv_p[R2][1], n_inv_p[R2][2]);
        trj_start(L1, L2, R1, R2, dly_trj);
    }

    // genap
    else if (sekuen % 2 == 0)
    {
        // step kaki kiri
        // langkah kaki kiri
        trj_langkah(L1, c_inv_p[L1][0], c_inv_p[L1][1], c_inv_p[L1][2], n_inv_p[L1][0], n_inv_p[L1][1], titik_puncak);
        trj_langkah(R2, c_inv_p[R2][0], c_inv_p[R2][1], c_inv_p[R2][2], n_inv_p[R2][0], n_inv_p[R2][1], titik_puncak);

        // geser kaki kanan
        trj_lurus(R1, c_inv_p[R1][0], c_inv_p[R1][1], c_inv_p[R1][2], n_inv_p[R1][0], n_inv_p[R1][1], n_inv_p[R1][2]);
        trj_lurus(L2, c_inv_p[L2][0], c_inv_p[L2][1], c_inv_p[L2][2], n_inv_p[L2][0], n_inv_p[L2][1], n_inv_p[L2][2]);
        trj_start(L1, L2, R1, R2, dly_trj);
    }
}

void motion::creep_gait(int sekuen, int dly_trj)
{
    int delay2 = dly_trj/2;
    // ganjil
    if (sekuen % 2 != 0)
    {
        // step kaki kanan
        // langkah kaki kanan
        trj_langkah(R1, c_inv_p[R1][0], c_inv_p[R1][1], c_inv_p[R1][2], n_inv_p[R1][0], n_inv_p[R1][1], titik_puncak);
        trj_langkah(R2, c_inv_p[R2][0], c_inv_p[R2][1], c_inv_p[R2][2], n_inv_p[R2][0], n_inv_p[R2][1], titik_puncak);

        // geser kaki kiri
        trj_lurus(L1, c_inv_p[L1][0], c_inv_p[L1][1], c_inv_p[L1][2], n_inv_p[L1][0], n_inv_p[L1][1], n_inv_p[L1][2]);
        trj_lurus(L2, c_inv_p[L2][0], c_inv_p[L2][1], c_inv_p[L2][2], n_inv_p[L2][0], n_inv_p[L2][1], n_inv_p[L2][2]);
        trj_start(xx, xx, R1, xx, delay2);
        trj_start(xx, xx, xx, R2, delay2);
        trj_start(L1, L2, xx, xx, delay2);
        // start_creep(dly_trj);
    }

    // genap
    else if (sekuen % 2 == 0)
    {
        // step kaki kiri
        // langkah kaki kiri
        trj_langkah(L1, c_inv_p[L1][0], c_inv_p[L1][1], c_inv_p[L1][2], n_inv_p[L1][0], n_inv_p[L1][1], titik_puncak);
        trj_langkah(L2, c_inv_p[L2][0], c_inv_p[L2][1], c_inv_p[L2][2], n_inv_p[L2][0], n_inv_p[L2][1], titik_puncak);
        // geser kaki kanan
        trj_lurus(R1, c_inv_p[R1][0], c_inv_p[R1][1], c_inv_p[R1][2], n_inv_p[R1][0], n_inv_p[R1][1], n_inv_p[R1][2]);
        trj_lurus(R2, c_inv_p[R2][0], c_inv_p[R2][1], c_inv_p[R2][2], n_inv_p[R2][0], n_inv_p[R2][1], n_inv_p[R2][2]);
        trj_start(L1, xx, xx, xx, delay2);
        trj_start(xx, L2, xx, xx, delay2);
        trj_start(xx, xx, R1, R2, delay2);
        // start_creep(dly_trj);
    }
}