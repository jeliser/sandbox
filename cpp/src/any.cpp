#include <string>
#include <experimental/any>
#include <iostream>

using namespace std::string_literals;

/* Compile with:
$ g++ -std=c++17 any.cpp -o any
*/

class Message {
  int a;
  float b;
  double c;
  std::string d;

 public:
  bool set_any(const std::string& key, const std::experimental::any& val) {
    try {
      if(key == "a") {
        a = std::experimental::any_cast<int>(val);
      } else if(key == "b") {
        b = std::experimental::any_cast<float>(val);
      } else if(key == "c") {
        c = std::experimental::any_cast<double>(val);
      } else if(key == "d") {
        d = std::experimental::any_cast<std::string>(val);
      } else {
        std::cerr << "Invalid key " << key << std::endl;
        return false;
      }
    } catch(const std::experimental::bad_any_cast& e) {
      std::cout << "set_any() -> " << e.what() << '\n';
      return false;
    }
    return true;
  }

  std::experimental::any get_any(const std::string& key) {
    std::experimental::any ret;
    if(key == "a") {
      ret = a;
    } else if(key == "b") {
      ret = b;
    } else if(key == "c") {
      ret = c;
    } else if(key == "d") {
      ret = d;
    }
    return ret;
  }

  friend std::ostream& operator<<(std::ostream& os, const Message& m) {
    os << "a: " << m.a << " | b: " << m.b << " | c: " << m.c << " | d: " << m.d;
  }
};

int main(int argc, char const* argv[]) {
  Message msg;
  msg.set_any("a", 1);
  msg.set_any("b", 3.14f);
  msg.set_any("c", 9.81);
  msg.set_any("d", "Nelson"s);
  std::cout << "First message: " << msg << std::endl;

  Message second;
  second.set_any("a", msg.get_any("a"));
  second.set_any("b", msg.get_any("b"));
  second.set_any("c", msg.get_any("c"));
  second.set_any("d", msg.get_any("d"));
  std::cout << "Second message: " << second << std::endl;

  auto invalid_key = "aa"s;
  auto val = msg.get_any(invalid_key);

  if(!val.empty()) {
    std::cout << invalid_key << ": " << std::experimental::any_cast<int>(val) << std::endl;
  } else {
    std::cout << "Invalid key: " << invalid_key << std::endl;
  }

  for(std::string k : {"a", "b", "c", "d", "e"}) {
    auto v = msg.get_any(k);
    if(!v.empty()) {
      if(v.type() == typeid(int)) {
        std::cout << '"' << k << "\": " << std::experimental::any_cast<int>(v) << std::endl;
      } else if(v.type() == typeid(float)) {
        std::cout << '"' << k << "\": " << std::experimental::any_cast<float>(v) << std::endl;
      } else if(v.type() == typeid(double)) {
        std::cout << '"' << k << "\": " << std::experimental::any_cast<double>(v) << std::endl;
      } else if(v.type() == typeid(std::string)) {
        std::cout << '"' << k << "\": " << std::experimental::any_cast<std::string>(v) << std::endl;
      }
    } else {
      std::cout << "Invalid key: \"" << k << '"' << std::endl;
    }
  }
  return 0;
}
