// c++ includes
#include <algorithm>
#include <list>
#include <memory>
#include <vector>
#include <iostream>

// related includes

template <typename T>
class ListIterator
{
public:
    using TYPE = typename T::value_type;

    /** The constructor of the ListIterator class
     */
    ListIterator(T& in_container) {
        //container = std::ref(in_container);
    };

    TYPE obj;

    void remove()
    {
    }
};

template <typename T, template<typename, typename> class C>
class ListIterator2
{
public:
    using ALLOCATOR = typename T::allocator_type;
    using TYPE      = typename T::value_type;

    /** The constructor of the ListIterator class
     */
    ListIterator2() {
        //container = std::ref(in_container);
    };

    C<TYPE, ALLOCATOR> obj;

    void remove()
    {
    }
};


template <template<typename, typename> class C, typename T>
class ListIterator3
{
public:

    /** The constructor of the ListIterator class
     */
    ListIterator3() {
        //container = std::ref(in_container);
    };

    C<T, std::allocator<T>> obj;

    void remove()
    {
    }
};


int main(int argc, char * argv[]) {
    auto my_list = std::list<uint32_t>();
    my_list.clear();
    auto iter = ListIterator<std::list<uint32_t>>(my_list);
    iter.obj = 100;
    std::cout << iter.obj << std::endl;

    auto iter2 = ListIterator2<std::list<uint32_t>, std::list>();
    iter2.remove();

    auto iter3 = ListIterator3<std::list, uint32_t>();
    iter3.remove();

}
