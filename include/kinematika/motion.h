#ifndef MOTION_H
#define MOTION_H
#include "inverse.h"

class motion : public inverse
{
private:
    // KONFIGURASI TRAJECTORY
    float titik_puncak = -50; // titik puncak langkah -50

public:
    // menurunkan lengan gripper
    void capit();

    // standby mode kaki membentuk X terhadap body
    void siap(uint a = 820, uint b = 380, uint c = 204); // parameter untuk body putar dan gripper

    // pola langkah segitiga
    void tripod_gait(int sekuen, int dly_trj);

    void creep_gait(int sekuen, int dly_trj);
};
#endif