#include <stdio.h>
#include <stdint.h>


int main() {
  printf("Hello World\n");

  {
    float A1 = 0.440648973;
    float B2 = 0.999771237;
    float A2 = -0.999771237;
    float B1 = -0.440648973;
 
    float val = A1*B2 - A2*B1;
    printf("%f\n", val);
  }

  {
    uint32_t val0 = 0xBE29F05E;
    uint32_t val1 = 0x3EE19CBE;
    uint32_t val2 = 0xBF7FF102;
    float A1 = *((float*)&val0);
    float A2 = *((float*)&val1);
    float A3 = *((float*)&val2);
    printf("%f  %f  %f\n", A1, A2, A3);
    printf("%f  %f  %f\n", *((float*)&val0), *((float*)&val1), *((float*)&val2));
 
    uint32_t bal0 = 0x3E29F05E;
    uint32_t bal1 = 0xBEE19CBE;
    uint32_t bal2 = 0x3F7FF102;
    float B1 = *((float*)&bal0);
    float B2 = *((float*)&bal1);
    float B3 = *((float*)&bal2);
    printf("%f  %f  %f\n", B1, B2, B3);
    printf("%f  %f  %f\n", *((float*)&bal0), *((float*)&bal1), *((float*)&bal2));
 
    float tmp[3] = {0, 0, 0};
    tmp[0] = A2*B3 - A2*B3;
    tmp[1] = A3*B1 - A1*B3;
    tmp[2] = A1*B2 - A2*B1;
    printf("%.16f\n", tmp[0]);
    printf("%.16f\n", tmp[1]);
    printf("%.16f\n", tmp[2]);
  }

  return (0);
}
