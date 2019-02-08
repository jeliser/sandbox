#include <stdio.h>
#include <string>
#include <memory>
#include <list>

class Interface {
public:
  virtual ~Interface() {};
  virtual std::string print() = 0;
  virtual std::unique_ptr<Interface> clone() = 0;
};

class ImplA : public Interface {
public:
  ImplA() = default;
 ~ImplA() {};

public:
  std::string print() { return "hello"; };
  static std::unique_ptr<Interface> newInstance() { return std::make_unique<ImplA>(); };
  std::unique_ptr<Interface> clone() { return std::make_unique<ImplA>(*this); }
};

class ImplB : public Interface {
public:
  ImplB() = default;
 ~ImplB() {};

public:
  std::string print() { return "world"; };
  static std::unique_ptr<Interface> newInstance() { return std::make_unique<ImplB>(); };
  std::unique_ptr<Interface> clone() { return std::make_unique<ImplB>(*this); }
};

int main(int argc, char * argv[]) {

  auto tmp = std::make_unique<int>();
  *tmp = 123;
  auto tmp2 = std::make_shared<int>(*tmp);
  printf("%d  %d\n", *tmp, *tmp2);


  // Create a bunch of unique pointers
  std::list<std::unique_ptr<Interface>> unique_objs;
  unique_objs.emplace_back(ImplA::newInstance());
  unique_objs.emplace_back(ImplB::newInstance());
  for(auto& obj : unique_objs) {
    printf("%s\n", obj->print().c_str());
  }

  // Create a bunch of shared pointers
  //
  // --- Notice that unique pointers from a factory and be cast to shared!! That's the real point of this example
  //
  std::list<std::shared_ptr<Interface>> shared_objs;
  shared_objs.emplace_back(unique_objs.front()->clone());
  shared_objs.emplace_back(unique_objs.back()->clone());
  for(auto& obj : shared_objs) {
    printf("%s\n", obj->print().c_str());
  }

  // Create a bunch of unique pointers
  std::list<std::unique_ptr<Interface>> new_unique_objs;
  new_unique_objs.emplace_back(unique_objs.front()->clone());
  new_unique_objs.emplace_back(unique_objs.back()->clone());
  for(auto& obj : new_unique_objs) {
    printf("%s\n", obj->print().c_str());
  }

  const char* char1 = "Hello";
  const char* char2 = "Hello";
  if(char1 == char2) {
    printf("char1 == char2\n");
  } else {
    printf("char1 != char2\n");
  }

  return 0;
}
