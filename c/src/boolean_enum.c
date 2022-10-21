#include <stdio.h>

enum __attribute__((__packed__)) PackedFlags {
    PACKED = 0b00000001,
};

enum UnpackedFlags {
    UNPACKED = 0b00000001,
};

int main (int argc, char * argv[]) {
  printf("packed:\t\t%lu\n", sizeof(enum PackedFlags));
  printf("unpacked:\t%lu\n", sizeof(enum UnpackedFlags));
  return 0;
}
