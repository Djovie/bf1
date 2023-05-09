#ifndef BODY_H
#define BODY_H
#include "motion.h"

class body : public motion
{
private:
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

    // offset panjang bagian badan dari titik tengah robot terhadap kaki
    //[l1,l2,r1,r2][x,y,z]
    const float offset[4][3] = {{-55.00, +55.00, 0},
                                {-55.00, -55.00, 0},
                                {+55.00, +55.00, 0},
                                {+55.00, -55.00, 0}};

    // variabel koordinat kaki terhadap bodi (current body position)
    float c_body_p[4][3]; //[l1,l2,r1,r2][x,y,z]
    // new body position
    float n_body_p[4][3]; //[l1,l2,r1,r2][x,y,z]

    float step_cal = 2;//1.5; // (1.5) perhitungan step gait

    // KONFIGURASI GERAK
    float max_step = 10; // gerak langkah(50 mm)

    float step_kanan, step_kiri; // ,step_kanan_x, step_kanan_y, step_kiri_x, step_kiri_y;

    // sudut bodi (current position) (rotasi body)
    float body_theta = 0, body_alpha = 0, body_beta = 0;

public:
    // inisialisasi servo
    body(uint8_t dxl_id[12], const char *port_name = "/dev/ttyUSB0", uint32_t baud_rate = 1000000);

    // gerak lurus body sumbu y (+maju / -mundur)
    void gerak_body_y(float pos_y, int dly_trj);

    // gerak lurus body sumbu x (-kanan / +kiri)
    void gerak_body_x(float pos_x, int dly_trj);

    // gerak putar
    void gerak_putar(float arah, int dly_trj);

    // rotasi terhadap pusat bodi
    void rotasi_body(float roll, float pitch, float yaw, int dly_trj = 9000);

    // translasi (pergeseran) pusat body
    void translasi_body(float pos_x, float pos_y, float pos_z, int dly_trj = 9000);

private:
    // hitung koordinat kaki terhadap pusat body
    void kaki_to_body();

    // hitung koordinat kaki terhadap pusat coxa (Inverse kinematics)
    void kaki_to_coxa();

    void cetak(double matriks_rotasi[3][3]);
};
#endif