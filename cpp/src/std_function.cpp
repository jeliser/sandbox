// https://stackoverflow.com/questions/7713266/how-can-i-change-the-variable-to-which-a-c-reference-refers

#include <string>
#include <iostream>
#include <functional>
#include <map>

void test(bool val) {
  printf("%d\n", val);
}

typedef struct {
  std::function<void(bool)> callback;
} CallbackProperties;

int main() {
  auto func1 = std::function<void(bool)>();

  if(func1) {
    std::cout << "We're a function" << std::endl;
  } else {
    std::cout << "No function" << std::endl;
  }

  func1 = std::bind(test, std::placeholders::_1);
  if(func1) {
    std::cout << "We're a function" << std::endl;
    func1(false);
    func1(true);
  } else {
    std::cout << "No function" << std::endl;
  }

  func1 = nullptr;

  std::map<int, CallbackProperties> callbacks;
  callbacks.emplace(0, CallbackProperties{std::bind(test, std::placeholders::_1)});
  auto found = callbacks.find(0);
  found->second.callback(true);
  callbacks.clear();
}
