#include "../../include/kinematika/inverse.h"

// inverse kinematics
void inverse::inverse_s(uint8_t dxl_id, float x, float y, float z)
{
    float theta_c, theta_f, theta_t, kalkulasi;     // sudut dari analisa IK
    float theta_f1, theta_f2, a, x0, x1;            // digunakan pd perhitungan
    float theta_c_real, theta_f_real, theta_t_real; // sudut hasil normalisasi servo
    const char *log;                                // syncwrite

    // a. HITUNG THETA Coxa
    // hitung theta c

    theta_c = atan2(y, x);
    theta_c = theta_c * 180 / M_PI; // rad to deg
    x0 = sqrt(pow(y, 2) + pow(x, 2));

    // b. HITUNG THETA Femur
    // hitung theta f1
    x1 = x0 - c; // pengurangan panjang x0 dan coxa
    theta_f1 = atan2(z, x1);
    theta_f1 = theta_f1 * 180 / M_PI;
    // hitung panjang a
    a = sqrt(pow(z, 2) + pow(x1, 2));
    // hitung f2
    theta_f2 = acos((pow(f, 2) + pow(a, 2) - pow(t, 2)) / (2 * a * f));
    theta_f2 = theta_f2 * 180 / M_PI;
    // hitung f
    theta_f = theta_f1 + theta_f2;
    // c. HITUNG THETA Tibia
    // hitung theta t
    theta_t = acos((pow(f, 2) + pow(t, 2) - pow(a, 2)) / (2 * f * t));
    theta_t = (theta_t * (180 / M_PI)) - 90;

    // d. Normalisasi 0 derajat servo
    if (dxl_id == R1 || dxl_id == R2)
    {
        // coba hitung manual
        theta_c_real = 150 - theta_c;
        theta_f_real = 150 - theta_f;
        theta_t_real = 150 + theta_t;
    }
    if (dxl_id == L1 || dxl_id == L2)
    {
        theta_c = theta_c < 0 ? 360 + theta_c : theta_c;
        theta_c_real = 330 - theta_c;
        theta_f_real = 150 - theta_f;
        theta_t_real = 150 + theta_t;
    }

    switch (dxl_id)
    {
    case R1:
        // putar
        posisi_servo[0] = theta_t_real * 3.41;
        posisi_servo[1] = theta_f_real * 3.41;
        posisi_servo[2] = theta_c_real * 3.41;

        dxl_wb.syncWrite(handler_index, &posisi_servo[0], &log); // eksekusi aktuator

        c_inv_p[2][0] = x;
        c_inv_p[2][1] = y;
        c_inv_p[2][2] = z;
        break;
    case R2:
        // putar
        posisi_servo[3] = theta_t_real * 3.41;
        posisi_servo[4] = theta_f_real * 3.41;
        posisi_servo[5] = theta_c_real * 3.41;

        dxl_wb.syncWrite(handler_index, &posisi_servo[0], &log); // eksekusi aktuator

        c_inv_p[3][0] = x;
        c_inv_p[3][1] = y;
        c_inv_p[3][2] = z;
        break;
    case L1:
        // putar
        posisi_servo[6] = theta_t_real * 3.41;
        posisi_servo[7] = theta_f_real * 3.41;
        posisi_servo[8] = theta_c_real * 3.41;

        dxl_wb.syncWrite(handler_index, &posisi_servo[0], &log); // eksekusi aktuator

        c_inv_p[0][0] = x;
        c_inv_p[0][1] = y;
        c_inv_p[0][2] = z;
        break;
    case L2:
        // putar
        posisi_servo[9] = theta_t_real * 3.41;
        posisi_servo[10] = theta_f_real * 3.41;
        posisi_servo[11] = theta_c_real * 3.41;

        dxl_wb.syncWrite(handler_index, &posisi_servo[0], &log); // eksekusi aktuator

        c_inv_p[1][0] = x;
        c_inv_p[1][1] = y;
        c_inv_p[1][2] = z;
        break;
    }
}

// pola trjektori parabola
void inverse::polinomial_trj(uint8_t dxl_id, float pol[4][3])
{
    // KONFIGURASI TRAJECTORY
    float iterasi = 0.1; // banyak step 1/0.05=20
    float cal[4];        // untuk perhitungan polinomial berdasarkan 4 kaki
    float hp[3];         // hasil polinomial [x,y,z]
    int nmr_data = 0;    // no data array

    // hitung end point dengan polinomial
    for (float t = 0.0; t <= 1.009; t = t + iterasi)
    {
        // hitung polinomial
        cal[0] = pow((1 - t), 3);
        cal[1] = 3 * t * pow((1 - t), 2);
        cal[2] = 3 * pow(t, 2) * (1 - t);
        cal[3] = pow(t, 3);

        for (ushort i = 0; i < 3; i++)
        {
            hp[i] = 0;

            for (ushort j = 0; j < 4; j++)
            {
                hp[i] += cal[j] * pol[j][i];
            }

            // simpan hasil perhitungan
            array_p[dxl_id][i][nmr_data] = hp[i];
        }

        nmr_data++;
    }

    jlh_data = nmr_data;
}

// trayektori gesek
void inverse::trj_lurus(uint8_t dxl_id, float x0, float y0, float z0, float x1, float y1, float z1)
{
    // titik vektor (vector point)
    float vector_p[4][3]; //[p1,p2,p3,p4][x,y,z]

    // tentukan titik vektor polinomial
    vector_p[0][0] = x0;
    vector_p[0][1] = y0;
    vector_p[0][2] = z0; // P1
    vector_p[1][0] = x0;
    vector_p[1][1] = y0;
    vector_p[1][2] = z0; // P2

    vector_p[2][0] = x1;
    vector_p[2][1] = y1;
    vector_p[2][2] = z1; // P3
    vector_p[3][0] = x1;
    vector_p[3][1] = y1;
    vector_p[3][2] = z1; // P4

    polinomial_trj(dxl_id, vector_p);
}

// trayektori langkah
void inverse::trj_langkah(uint8_t dxl_id, float x0, float y0, float z0, float x1, float y1, float zp)
{
    float vector_p[4][3]; //[p1,p2,p3,p4][x,y,z]

    float z1;
    z1 = (zp - (0.25 * z0)) / 0.75;
    // tentukan titik vektor polinomial
    vector_p[0][0] = x0;
    vector_p[0][1] = y0;
    vector_p[0][2] = z0; // P1
    vector_p[1][0] = x0;
    vector_p[1][1] = y0;
    vector_p[1][2] = z1; // P2
    vector_p[2][0] = x1;
    vector_p[2][1] = y1;
    vector_p[2][2] = z1; // P3
    vector_p[3][0] = x1;
    vector_p[3][1] = y1;
    vector_p[3][2] = z0; // P4

    polinomial_trj(dxl_id, vector_p);
}

// eksekusi trayektori
void inverse::trj_start(uint8_t id_kakiL1, uint8_t id_kakiL2, uint8_t id_kakiR1, uint8_t id_kakiR2, int dly_trj)
{
    // Hitung hasil perhitungan tsb menggunakan IK

    for (uint8_t i = 0; i < jlh_data; i++)
    {
        if (id_kakiL1 == L1)
        {
            inverse_s(id_kakiL1, array_p[L1][0][i], array_p[L1][1][i], array_p[L1][2][i]);
        }

        if (id_kakiL2 == L2)
        {
            inverse_s(id_kakiL2, array_p[L2][0][i], array_p[L2][1][i], array_p[L2][2][i]);
        }

        if (id_kakiR1 == R1)
        {
            inverse_s(id_kakiR1, array_p[R1][0][i], array_p[R1][1][i], array_p[R1][2][i]);
        }

        if (id_kakiR2 == R2)
        {
            inverse_s(id_kakiR2, array_p[R2][0][i], array_p[R2][1][i], array_p[R2][2][i]);
        }

        usleep(dly_trj);
    }
}

//////////////////////////////////////////////////////////////////
// eksekusi trjektori baru
void inverse::start_creep(int dly_trj)
{
    bool l1 = 1, l2 = 0, r1 = 0, r2 = 0;
    int il1 = 0, il2 = 0, ir1 = 0, ir2 = 0;
    int data = ((jlh_data / 2) + 0) * 4;

    // Hitung hasil perhitungan tsb menggunakan IK
    while (data)
    {
        if (l1)
        {
            inverse_s(0, array_p[L1][0][il1], array_p[L1][1][il1], array_p[L1][2][il1]);
            il1++;
            l1 = (il1 < jlh_data);
            if (l1 == false)
            {
                r1 = true; //(il1 > (jlh_data / 2) - 1);
            }
        }
        if (r1)
        {
            inverse_s(2, array_p[R1][0][ir1], array_p[R1][1][ir1], array_p[R1][2][ir1]);
            ir1++;
            r1 = (ir1 < jlh_data);
            if (r1 == false)
            {
                r2 = true; //(ir1 >= (jlh_data / 2) - 1);
            }
        }
        if (r2)
        {
            inverse_s(3, array_p[R2][0][ir2], array_p[R2][1][ir2], array_p[R2][2][ir2]);
            ir2++;
            r2 = (ir2 < jlh_data);
            if (r2 == false)
            {
                l2 = true; //(ir2 >= (jlh_data / 2) - 1);
            }
        }
        if (l2)
        {
            inverse_s(1, array_p[L2][0][il2], array_p[L2][1][il2], array_p[L2][2][il2]);
            il2++;
            l2 = (il2 < jlh_data);
            // l1 = (il2 >= (dta / 2) - 1);
        }
        usleep(dly_trj);
        data--;
    }
}