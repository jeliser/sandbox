
#include <memory>
#include <list>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

int hello_world();

class HelloWorldInterface {
 public:
  virtual void print() = 0;
  virtual std::list<std::string> get_list() = 0;
};

class HelloWorld : public HelloWorldInterface {
 public:
  void print();
  std::list<std::string> get_list();
};

std::unique_ptr<HelloWorldInterface> newInstance();

#ifdef __cplusplus
}
#endif
