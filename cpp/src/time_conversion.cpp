// duration::count
#include <iostream>     // std::cout
#include <chrono>       // std::chrono::seconds, std::chrono::milliseconds
                        // std::chrono::duration_cast

int main ()
{
  using namespace std::chrono;
  seconds foo (0.001235345); // 0.001235345 second
  std::cout << foo.count() << " seconds.\n";

  /// 0 seconds
  /// 0.001235 seconds

  return 0;
}
