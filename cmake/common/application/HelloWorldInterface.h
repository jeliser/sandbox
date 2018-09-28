
#include <list>
#include <memory>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

class HelloWorldInterface {
 public:
  virtual void print() = 0;
  virtual std::list<std::string> get_list() = 0;
};

std::unique_ptr<HelloWorldInterface> newInstance();

#ifdef __cplusplus
}
#endif
