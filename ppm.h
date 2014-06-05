#define T_MIN (1*F_CPU/1000) // 1ms = 12000 Ticks @ 12Mhz = Stick min position
#define T_MAX (2*F_CPU/1000) // 2ms = 24000 Ticks @ 12Mhz = Stick max position
#define T_OUT (3*F_CPU/1000) // 3ms = 36000 Ticks @ 12Mhz = Timeout

extern volatile char ppmNewData;
void ppmInit();
unsigned char ppmGet(int n);

