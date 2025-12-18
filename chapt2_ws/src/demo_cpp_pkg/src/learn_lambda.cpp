#include <iostream>
#include <algorithm>  // lambda需要

int main()
{
    auto add = [](int a , int b)->int {
        return a + b;
    };

    int sum = add(3, 5);
    auto print_sum = [sum]() -> void{
        std::cout << "3 + 5 = " << sum << "\n";
    };
    print_sum();
    return 0;
}