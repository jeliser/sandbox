#include <cstdio>
#include <limits>
#include <memory>
#include <cstdlib>
#include <iostream>
#include <functional>

using namespace std;

struct node {
  int value;
  struct node* next;
};

typedef struct node Node;

auto createList() {
  auto del1 = [](Node* p) {
    while(p) {
      std::cout << "Deleting value is : " << p->value;
      struct node* n = p->next;
      delete p;
      p = n;
    }
    return;
  };
  std::unique_ptr<Node, decltype(del1)> head(new Node, del1);

  Node* temp = head.get();
  temp->value = 0;
  for(int i = 1; i < 8; i++) {
    if(temp->next == nullptr) {
      temp->next = new Node();
      temp = temp->next;
      temp->value = i;
      temp->next = nullptr;
    }
    // temp=temp->next;
  }
  return head;
}

class hello_t {
 public:
  std::unique_ptr<int, std::function<void(int*)>> world;

  hello_t() {
  }
};

int main() {
  auto del1 = [](Node* p) {
    while(p) {
      std::cout << "Deleting value is : " << p->value << std::endl;
      struct node* n = p->next;
      delete p;
      p = n;
    }
    return;
  };

  std::unique_ptr<Node, decltype(del1)> head(createList().release(), del1);
  auto tail = std::unique_ptr<Node, void (*)(Node*)>(createList().release(), del1);
  std::unique_ptr<Node, void (*)(Node*)> tails = std::unique_ptr<Node, void (*)(Node*)>(createList().release(), del1);

  hello_t hello;
  hello.world = std::unique_ptr<int, std::function<void(int*)>>(new int(), [](int* tmp) { delete tmp; });
}
