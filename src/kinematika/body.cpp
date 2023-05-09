#include "../../include/kinematika/body.h"

// inisialisasi servo
body::body(uint8_t id[12], const char *port_name, uint32_t baud_rate)
{
    const char *log;
    uint16_t model_number = 0;

    // size_t idLength;
    ushort idLength = 15;

    dxl_wb.init(port_name, baud_rate, &log);

    for (int i = 0; i < idLength; i++)
    {
        dxl_wb.ping(id[i], &model_number, &log);
        dxl_wb.addSyncWriteHandler(id[i], "Goal_Position", &log);
    }
}

// hitung koordinat kaki terhadap pusat body
void body::kaki_to_body()
{
    for (int i = 0; i < 4; i++)
    {
        c_body_p[i][0] = c_inv_p[i][0] + offset[i][0];
        c_body_p[i][1] = c_inv_p[i][1] + offset[i][1];
        c_body_p[i][2] = c_inv_p[i][2] + offset[i][2];
    }
}

// hitung koordinat kaki terhadap pusat coxa (Inverse kinematics)
void body::kaki_to_coxa()
{
    // koordinat kaki baru
    for (int i = 0; i < 4; i++)
    {
        n_inv_p[i][0] = n_body_p[i][0] - offset[i][0];
        n_inv_p[i][1] = n_body_p[i][1] - offset[i][1];
        n_inv_p[i][2] = n_body_p[i][2] - offset[i][2];
    }
}

// gerak lurus body sumbu y (maju / mundur)
void body::gerak_body_y(float pos_y, int dly_trj)
{
    float body_step, jumlah_step, step_gait;
    // float step_kanan, step_kiri;
    // float step_cal = 1.5; // (1.5) perhitungan step gait

    // float max_step = 50; // 30 mm

    // hitung step, banyak step
    for (int n = 1; n <= 1000; n++)
    {
        body_step = pos_y / n;

        if (abs(body_step) <= max_step)
        {
            jumlah_step = n;
            break;
        }
    }

    // hitung total step gait
    step_gait = jumlah_step + 1;

    // hitung body kinematic, gait
    for (int i = 1; i <= step_gait; i++)
    {

        // tentukan step langkah
        // step awal
        if (i == 1)
        {
            step_kanan = step_cal * body_step;
            step_kiri = -step_cal * body_step;
        }
        // step genap
        else if ((i % 2 == 0) && (i != 1) && (i != step_gait)) //(i != step_gait)
        {
            step_kanan = -(step_cal * 2) * body_step;
            step_kiri = (step_cal * 2) * body_step;
        }
        // step ganjil
        else if ((i % 2 != 0) && (i != 1) && (i != step_gait)) //(i != step_gait)
        {
            step_kanan = (step_cal * 2) * body_step;
            step_kiri = -(step_cal * 2) * body_step;
        }
        // step akhir
        else if (i == step_gait)
        {
            // ganjil
            if (i % 2 != 0)
            {
                step_kanan = step_cal * body_step;
                step_kiri = -step_cal * body_step;
            }
            // genap
            else if (i % 2 == 0)
            {
                step_kanan = -step_cal * body_step;
                step_kiri = step_cal * body_step;
            }
        }

        // hitung posisi relatif thdp bodi
        kaki_to_body();

        // hitung koordinat selanjutnya
        // langkah kiri
        // kaki L1
        n_body_p[0][0] = c_body_p[0][0];
        n_body_p[0][1] = c_body_p[0][1] + step_kiri;
        n_body_p[0][2] = c_body_p[0][2];
        // kaki R2
        n_body_p[3][0] = c_body_p[3][0];
        n_body_p[3][1] = c_body_p[3][1] + step_kiri;
        n_body_p[3][2] = c_body_p[3][2];

        // langkah kanan
        // kaki R1
        n_body_p[2][0] = c_body_p[2][0];
        n_body_p[2][1] = c_body_p[2][1] + step_kanan;
        n_body_p[2][2] = c_body_p[2][2];
        // kaki L2
        n_body_p[1][0] = c_body_p[1][0];
        n_body_p[1][1] = c_body_p[1][1] + step_kanan;
        n_body_p[1][2] = c_body_p[1][2];

        // kembalikan koordinat ke pusat coxa
        kaki_to_coxa();

        // @note buka komen setelah maintenance
        // // gerakkan kaki
        // for (int a = 0; a < 4; a++)
        // {
        //     std::cout << n_body_p[a][0] << "\t || \t"
        //               << n_body_p[a][1] << "\t || \t"
        //               << n_body_p[a][2]
        //               << std::endl;
        // }
        // std::cout << "======================================================" << std::endl;

        tripod_gait(i, dly_trj);
        // creep_gait(i, dly_trj);
    }
}

// gerak lurus body sumbu x (kanan / kiri)
void body::gerak_body_x(float pos_x, int dly_trj)
{
    float body_step, jumlah_step, step_gait;

    // hitung step, banyak step
    for (int n = 1; n <= 1000; n++)
    {
        body_step = pos_x / n;
        if (abs(body_step) <= max_step)
        {
            jumlah_step = n;
            break;
        }
    }

    // hitung total step gait
    step_gait = jumlah_step + 1;

    // hitung body kinematic, gait
    for (int i = 1; i <= step_gait; i++)
    { // kondisi awal i=1
        // tentukan step langkah
        // step awal
        if (i == 1)
        {
            step_kanan = step_cal * body_step;
            step_kiri = -step_cal * body_step;
        }
        // step genap
        else if ((i % 2 == 0) && (i != 1) && (i != step_gait))
        {
            step_kanan = -(step_cal * 2) * body_step;
            step_kiri = (step_cal * 2) * body_step;
        }
        // step ganjil
        else if ((i % 2 != 0) && (i != 1) && (i != step_gait))
        {
            step_kanan = (step_cal * 2) * body_step;
            step_kiri = -(step_cal * 2) * body_step;
        }
        // step akhir
        else if (i == step_gait)
        {
            // ganjil
            if (i % 2 != 0)
            {
                step_kanan = step_cal * body_step;
                step_kiri = -step_cal * body_step;
            }
            // genap
            else if (i % 2 == 0)
            {
                step_kanan = -step_cal * body_step;
                step_kiri = step_cal * body_step;
            }
        }

        // hitung posisi relatif thdp bodi
        kaki_to_body();

        // hitung koordinat selanjutnya
        // langkah kiri
        // kaki L1
        n_body_p[0][0] = c_body_p[0][0] + step_kiri;
        n_body_p[0][1] = c_body_p[0][1];
        n_body_p[0][2] = c_body_p[0][2];
        // kaki R2
        n_body_p[3][0] = c_body_p[3][0] + step_kiri;
        n_body_p[3][1] = c_body_p[3][1];
        n_body_p[3][2] = c_body_p[3][2];

        // langkah kanan
        // kaki R1
        n_body_p[2][0] = c_body_p[2][0] + step_kanan;
        n_body_p[2][1] = c_body_p[2][1];
        n_body_p[2][2] = c_body_p[2][2];
        // kaki L2
        n_body_p[1][0] = c_body_p[1][0] + step_kanan;
        n_body_p[1][1] = c_body_p[1][1];
        n_body_p[1][2] = c_body_p[1][2];

        // kembalikan koordinat ke pusat coxa
        kaki_to_coxa();

        // gerakkan kaki
        tripod_gait(i, dly_trj);
    }
}

// gerak putar
void body::gerak_putar(float arah, int dly_trj)
{
    float e_theta, e_alpha, e_beta; // sudut rotasi
    float step_kanan, step_kiri;    // langkah kaki
    float body_step, jumlah_step, step_gait;
    // BODY
    float max_putar = 10; // 10 derajat
    float step_cal = 0.5; // (1.5) perhitungan step gait
    float arah_2;

    // hitung step, banyak step
    for (int n = 1; n <= 1000; n++)
    {
        body_step = arah / n;
        if (abs(body_step) <= max_putar)
        {
            jumlah_step = n;
            break;
        }
    }

    // hitung total step gait
    step_gait = jumlah_step + 1;

    // hitung body kinematic, gait
    for (int i = 1; i <= step_gait; i++)
    {
        // tentukan step langkah
        // step awal
        if (i == 1)
        {
            step_kanan = step_cal * body_step;
            step_kiri = -step_cal * body_step;
        }
        // step genap
        else if ((i % 2 == 0) && (i != 1) && (i != step_gait))
        {
            step_kanan = -body_step;
            step_kiri = body_step;
        }
        // step ganjil
        else if ((i % 2 != 0) && (i != 1) && (i != step_gait))
        {
            step_kanan = body_step;
            step_kiri = -body_step;
        }
        // step akhir
        else if (i == step_gait)
        {
            // ganjil
            if (i % 2 != 0)
            {
                step_kanan = step_cal * body_step;
                step_kiri = -step_cal * body_step;
            }
            // genap
            else if (i % 2 == 0)
            {
                step_kanan = -step_cal * body_step;
                step_kiri = step_cal * body_step;
            }
        }

        // ubah ke rad
        step_kanan = toRad(step_kanan); // * M_PI / 180;
        step_kiri = toRad(step_kiri);   //* M_PI / 180;

        // rotasi Z (putar arah bodi)
        e_theta = 0;
        e_alpha = 0;
        e_beta = step_kanan;

        // hitung matriks rotasi kaki kanan
        double matriks_rotasi_kanan[3][3] =
            {
                {cos(e_alpha) * cos(e_beta),
                 -sin(e_beta) * cos(e_alpha),
                 sin(e_alpha)},

                {sin(e_theta) * sin(e_alpha) * cos(e_beta) + cos(e_theta) * sin(e_beta),
                 -sin(e_theta) * sin(e_beta) * sin(e_alpha) + cos(e_theta) * cos(e_beta),
                 -sin(e_theta) * cos(e_alpha)},

                {-(cos(e_theta) * sin(e_alpha) * cos(e_beta)) + sin(e_theta) * sin(e_beta),
                 cos(e_theta) * sin(e_alpha) * sin(e_beta) + sin(e_theta) * cos(e_beta),
                 cos(e_theta) * cos(e_alpha)}};

        // rotasi Z (putar arah bodi)
        e_theta = 0;
        e_alpha = 0;
        e_beta = step_kiri;

        // hitung matriks rotasi
        double matriks_rotasi_kiri[3][3] =
            {
                {cos(e_alpha) * cos(e_beta),
                 -sin(e_beta) * cos(e_alpha),
                 sin(e_alpha)},

                {sin(e_theta) * sin(e_alpha) * cos(e_beta) + cos(e_theta) * sin(e_beta),
                 -sin(e_theta) * sin(e_beta) * sin(e_alpha) + cos(e_theta) * cos(e_beta),
                 -sin(e_theta) * cos(e_alpha)},

                {-(cos(e_theta) * sin(e_alpha) * cos(e_beta)) + sin(e_theta) * sin(e_beta),
                 cos(e_theta) * sin(e_alpha) * sin(e_beta) + sin(e_theta) * cos(e_beta),
                 cos(e_theta) * cos(e_alpha)}};

        // hitung posisi relatif thdp bodi
        kaki_to_body();

        // hitung posisi setelah rotasi
        // langkah kiri
        // kaki L1
        n_body_p[0][0] = matriks_rotasi_kiri[0][0] * c_body_p[0][0] + matriks_rotasi_kiri[0][1] * c_body_p[0][1] + matriks_rotasi_kiri[0][2] * c_body_p[0][2];
        n_body_p[0][1] = matriks_rotasi_kiri[1][0] * c_body_p[0][0] + matriks_rotasi_kiri[1][1] * c_body_p[0][1] + matriks_rotasi_kiri[1][2] * c_body_p[0][2];
        n_body_p[0][2] = matriks_rotasi_kiri[2][0] * c_body_p[0][0] + matriks_rotasi_kiri[2][1] * c_body_p[0][1] + matriks_rotasi_kiri[2][2] * c_body_p[0][2];
        // kaki R2
        n_body_p[3][0] = matriks_rotasi_kiri[0][0] * c_body_p[3][0] + matriks_rotasi_kiri[0][1] * c_body_p[3][1] + matriks_rotasi_kiri[0][2] * c_body_p[3][2];
        n_body_p[3][1] = matriks_rotasi_kiri[1][0] * c_body_p[3][0] + matriks_rotasi_kiri[1][1] * c_body_p[3][1] + matriks_rotasi_kiri[1][2] * c_body_p[3][2];
        n_body_p[3][2] = matriks_rotasi_kiri[2][0] * c_body_p[3][0] + matriks_rotasi_kiri[2][1] * c_body_p[3][1] + matriks_rotasi_kiri[2][2] * c_body_p[3][2];

        // langkah kanan
        // kaki R1
        n_body_p[2][0] = matriks_rotasi_kanan[0][0] * c_body_p[2][0] + matriks_rotasi_kanan[0][1] * c_body_p[2][1] + matriks_rotasi_kanan[0][2] * c_body_p[2][2];
        n_body_p[2][1] = matriks_rotasi_kanan[1][0] * c_body_p[2][0] + matriks_rotasi_kanan[1][1] * c_body_p[2][1] + matriks_rotasi_kanan[1][2] * c_body_p[2][2];
        n_body_p[2][2] = matriks_rotasi_kanan[2][0] * c_body_p[2][0] + matriks_rotasi_kanan[2][1] * c_body_p[2][1] + matriks_rotasi_kanan[2][2] * c_body_p[2][2];
        // kaki L2
        n_body_p[1][0] = matriks_rotasi_kanan[0][0] * c_body_p[1][0] + matriks_rotasi_kanan[0][1] * c_body_p[1][1] + matriks_rotasi_kanan[0][2] * c_body_p[1][2];
        n_body_p[1][1] = matriks_rotasi_kanan[1][0] * c_body_p[1][0] + matriks_rotasi_kanan[1][1] * c_body_p[1][1] + matriks_rotasi_kanan[1][2] * c_body_p[1][2];
        n_body_p[1][2] = matriks_rotasi_kanan[2][0] * c_body_p[1][0] + matriks_rotasi_kanan[2][1] * c_body_p[1][1] + matriks_rotasi_kanan[2][2] * c_body_p[1][2];

        // kembalikan koordinat ke pusat coxa
        kaki_to_coxa();

        // gerakkan kaki
        tripod_gait(i, dly_trj);
    }
}

// rotasi terhadap pusat bodi
void body::rotasi_body(float roll, float pitch, float yaw, int dly_trj)
{
    float e_theta, e_alpha, e_beta; // selisih sudut (error)
    /*batasi utk mencegah error mekanik(constrain)->
    hitung perbedaan sudut bodi (SV-PV)->
    ubah ke rad.
    */
    e_theta = toRad((constrain(roll, -15, 15) - body_theta));  // sudut rotasi sb X
    e_alpha = toRad((constrain(pitch, -15, 15) - body_alpha)); // sudut rotasi sb Y
    e_beta = toRad((constrain(yaw, -15, 15) - body_beta));     // sudut rotasi sb Z
    // hitung matriks rotasi
    double matriks_rotasi[3][3] =
        {
            {cos(e_alpha) * cos(e_beta),
             -sin(e_beta) * cos(e_alpha),
             sin(e_alpha)},

            {sin(e_theta) * sin(e_alpha) * cos(e_beta) + cos(e_theta) * sin(e_beta),
             -sin(e_theta) * sin(e_beta) * sin(e_alpha) + cos(e_theta) * cos(e_beta),
             -sin(e_theta) * cos(e_alpha)},

            {-(cos(e_theta) * sin(e_alpha) * cos(e_beta)) + sin(e_theta) * sin(e_beta),
             cos(e_theta) * sin(e_alpha) * sin(e_beta) + sin(e_theta) * cos(e_beta),
             cos(e_theta) * cos(e_alpha)}};

    // hitung posisi relatif thdp bodi
    kaki_to_body();

    // hitung posisi setelah rotasi
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
            {
                n_body_p[i][j] += matriks_rotasi[j][k] * c_body_p[i][k];
            }

    // cetak(matriks_rotasi);

    // kembalikan koordinat ke pusat coxa, koordinat baru
    kaki_to_coxa();

    // gerakkan kaki (pos sekarang ke pos baru)
    for (int i = 0; i < 4; i++)
        trj_lurus(i, c_inv_p[i][0], c_inv_p[i][1], c_inv_p[i][2], n_inv_p[i][0], n_inv_p[i][1], n_inv_p[i][2]);
    trj_start(L1, L2, R1, R2, dly_trj);

    // simpan sudut sekarang
    body_theta = roll;
    body_alpha = pitch;
    body_beta = yaw;
}

// translasi pusat body
void body::translasi_body(float pos_x, float pos_y, float pos_z, int dly_trj)
{
    float pos[3] = {pos_x, pos_y, pos_z};

    // hitung posisi relatif thdp bodi
    kaki_to_body();

    // hitung koordinat selanjutnya

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 3; j++)
            n_body_p[i][j] = c_body_p[i][j] + pos[j];

    // kembalikan koordinat ke pusat coxa
    kaki_to_coxa();

    // gerakkan kaki (pos sekarang ke pos baru)
    trj_lurus(L1, c_inv_p[L1][0], c_inv_p[L1][1], c_inv_p[L1][2], n_inv_p[L1][0], n_inv_p[L1][1], n_inv_p[L1][2]);
    trj_lurus(L2, c_inv_p[L2][0], c_inv_p[L2][1], c_inv_p[L2][2], n_inv_p[L2][0], n_inv_p[L2][1], n_inv_p[L2][2]);
    trj_lurus(R1, c_inv_p[R1][0], c_inv_p[R1][1], c_inv_p[R1][2], n_inv_p[R1][0], n_inv_p[R1][1], n_inv_p[R1][2]);
    trj_lurus(R2, c_inv_p[R2][0], c_inv_p[R2][1], c_inv_p[R2][2], n_inv_p[R2][0], n_inv_p[R2][1], n_inv_p[R2][2]);
    trj_start(L1, L2, R1, R2, dly_trj);
}

void body::cetak(double matriks_rotasi[3][3])
{
    // for (int i = 0; i < 4; i++)
    // {
    //     for (int j = 0; j < 3; j++)
    //     {
    //         for (int k = 0; k < 3; k++)
    //         {
    //             n_body_p[i][j] += matriks_rotasi[j][k] * c_body_p[i][k];
    //         }
    //     }
    // }

    for (int a = 0; a < 3; a++)
    {
        for (int b = 0; b < 4; b++)
        {
            std::cout << round(n_body_p[b][a]) << "\t||\t";
        }
        std::cout << std::endl;
    }
    std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
}