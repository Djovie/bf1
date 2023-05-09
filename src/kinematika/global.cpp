#pragma once

#include <iostream>
using namespace std;

namespace bodyKin
{
    // offset bodi (mm)
    // kaki L1
    float offset_L1_x = -55.00;
    float offset_L1_y = 55.00 ;
    float offset_L1_z = 0;
    // kaki L2
    float offset_L2_x = -55.00; 
    float offset_L2_y = -55.00;
    float offset_L2_z = 0;
    // kaki R1
    float offset_R1_x = 55.00;
    float offset_R1_y = 55.00;
    float offset_R1_z = 0;
    // kaki R2
    float offset_R2_x = 55.00;
    float offset_R2_y = -55.00;
    float offset_R2_z = 0;

    // variabel koordinat kaki thdp bodi (current position)
    float body_L1_x, body_L1_y, body_L1_z; // kaki L1
    float body_L2_x, body_L2_y, body_L2_z; // kaki L2
    float body_R1_x, body_R1_y, body_R1_z; // kaki R1
    float body_R2_x, body_R2_y, body_R2_z; // kaki R2

    // variabel koordinat baru, kaki thdp bodi (new)
    float n_body_L1_x, n_body_L1_y, n_body_L1_z; // kaki L1
    float n_body_L2_x, n_body_L2_y, n_body_L2_z; // kaki L2
    float n_body_R1_x, n_body_R1_y, n_body_R1_z; // kaki R1
    float n_body_R2_x, n_body_R2_y, n_body_R2_z; // kaki R2

    float step_cal = 1.5; // (1.5) perhitungan step gait

    // KONFIGURASI GERAK
    // gerak langkah
    float max_step = 50; // 30 mm

    float step_kanan, step_kiri, step_kanan_x, step_kanan_y, step_kiri_x, step_kiri_y; // langkah kaki
    // float body_step, body_step_y, body_step_x, jumlah_step, jumlah_step_y, jumlah_step_x, step_gait, step_gait_y, step_gait_x;

    // koordinat IK sebelum (old)
    // float o_L1_x, o_L1_y, o_L1_z; // kaki L1
    // float o_L2_x, o_L2_y, o_L2_z; // kaki L2
    // float o_R1_x, o_R1_y, o_R1_z; // kaki R1
    // float o_R2_x, o_R2_y, o_R2_z; // kaki R2

}
namespace inverseKin
{
#define L1 0
#define L2 1
#define R1 2
#define R2 3
#define xx -1

#define memori_data 25 // jumlah memori per koordinat per kaki

    // variabel data titik trajectory
    int jlh_data;

    int32_t posisi_servo[12];
    const uint8_t handler_index = 0;

    // Simpan Data IK
    // L1
    float array_px_L1[memori_data];
    float array_py_L1[memori_data];
    float array_pz_L1[memori_data];
    // L2
    float array_px_L2[memori_data];
    float array_py_L2[memori_data];
    float array_pz_L2[memori_data];
    // R1
    float array_px_R1[memori_data];
    float array_py_R1[memori_data];
    float array_pz_R1[memori_data];
    // R2
    float array_px_R2[memori_data];
    float array_py_R2[memori_data];
    float array_pz_R2[memori_data];

    // variabel koordinat inverse kinematics (current position)
    float L1_x, L1_y, L1_z; // kaki L1
    float L2_x, L2_y, L2_z; // kaki L2
    float R1_x, R1_y, R1_z; // kaki R1
    float R2_x, R2_y, R2_z; // kaki R2

    // koordinat IK baru (new)
    float n_L1_x, n_L1_y, n_L1_z; // kaki L1
    float n_L2_x, n_L2_y, n_L2_z; // kaki L2
    float n_R1_x, n_R1_y, n_R1_z; // kaki R1
    float n_R2_x, n_R2_y, n_R2_z; // kaki R2
}
