#include <stdio.h>
#include <stdint.h>

union Uint8Flags {
  typedef enum {
    True = 0x1,
    False = 0x0
  } type;
  uint8_t _dummy; // set the size of the enumeration container.
};

union Uint64Flags {
  typedef enum {
    True = 0x123,
    False = 0x999
  } type;
  uint64_t _dummy; // set the size of the enumeration container.
};

template< typename T>
class Container
{
  public:
    void set(const T val_in) {
      val = val_in;
    }
 
    T get() const {
      return val;
    }
    
  private:
    T val;
};

int main (int argc, char * argv[]) {
  printf("uint8_t enum\npacked:\t\t%lu\n", sizeof(Uint8Flags));
  printf("value True:\t%lu\n", Uint8Flags::True);
  printf("value False:\t%lu\n\n", Uint8Flags::False);

  printf("uint64_t enum\npacked:\t\t%lu\n", sizeof(Uint64Flags));
  printf("value True:\t%lu\n", Uint64Flags::True);
  printf("value False:\t%lu\n\n", Uint64Flags::False);

  Container<Uint8Flags::type> u8;

  u8.set(Uint8Flags::True);
  printf("Matching: %s\n", (u8.get() == Uint8Flags::True) ? "matched" : "not matching");
  u8.set(Uint8Flags::False);
  printf("Matching: %s\n", (u8.get() == Uint8Flags::True) ? "matched" : "not matching");

  Container<Uint64Flags::type> u64;

  u64.set(Uint64Flags::True);
  printf("Matching: %s\n", (u64.get() == Uint64Flags::True) ? "matched" : "not matching");
  u64.set(Uint64Flags::False);
  printf("Matching: %s\n", (u64.get() == Uint64Flags::True) ? "matched" : "not matching");

  // This will cause a compilation time failure
  //u64.set(444);

  return 0;
}
