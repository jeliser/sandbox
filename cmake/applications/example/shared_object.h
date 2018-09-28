
#include "common/application/HelloWorldInterface.h"

#ifdef __cplusplus
extern "C" {
#endif

int hello_world();

class HelloWorld : public HelloWorldInterface {
 public:
  void print();
  std::list<std::string> get_list();
};

#ifdef __cplusplus
}
#endif
