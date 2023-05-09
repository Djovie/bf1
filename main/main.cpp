#include <kinematics.h>

uint8_t dxl_id[15] = {1, 2, 3,
                      4, 5, 6,
                      7, 8, 9,
                      10, 11, 12};
// 13, 14, 15};

body f1(dxl_id, "/dev/ttyUSB0");
void one();
void two();

int main()
{
    uint8_t cases = 0;
    bool flag = true;

    f1.siap();
                f1.rotasi_body(0, 0, 5, 9000); //
    f1.gerak_body_y(50, 40000);
                f1.rotasi_body(0, 0, -5, 9000); //

    // f1.siap();

    while (flag)
    {
        // f1.siap();
        // sleep(1);
        // f1.siap();
        // f1.capit();
        // switch (cases)
        // {
        // case 0:
        // {
        //     f1.siap();
        //     sleep(1);
        //     //     f1.rotasi_body(0, 0, 15, 9000); //

        //     f1.gerak_body_x(130, 10000);

        //     sleep(1);
        //     f1.gerak_body_y(40, 10000);
        //     cases = 1;
        //     sleep(1);
        //     break;
        // }
        // case 1:
        // {
        //     // f1.capit();
        // f1.gerak_body_y(10, 50000);
        // f1.gerak_putar(-20, 9000); //
        // f1.gerak_body_x(20, 10000);
        // sleep(2);
        // f1.gerak_body_y(-130, 10000);
        // f1.gerak_putar(-10, 9000); //
        // f1.siap();
        flag = false;
        // break;
        // }
        // }
    }
    return 0;
}
// sleep(1);
// f1.gerak_putar(-20, 10000);
void one()
{
}

void two()
{
}