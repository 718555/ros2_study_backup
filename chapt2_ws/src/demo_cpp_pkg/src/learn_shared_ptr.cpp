#include <iostream>
#include <memory>

int main()
{
    auto p1 = std::make_shared<std::string>("This is a str");// std::make_shared<数据类型/类>(参数);返回值对应类的共享指针 
    std::cout << "p1 引用计数为 : " << p1.use_count() << ", 指向内存的地址为: " <<p1.get() << std::endl; // 1 ， 只有p1指向这个内存

    auto p2 = p1;
    std::cout << "p1 引用计数为 : " << p1.use_count() << ", 指向内存的地址为: " <<p1.get() << "\n"; // 2 ， p1和p2都指向这个内存
    std::cout << "p2 引用计数为 : " << p2.use_count() << ", 指向内存的地址为: " <<p2.get() << "\n"; // 2

    p1.reset(); // 释放引用，不指向"This is a str"所在内存
    std::cout << "p1 引用计数为 : " << p1.use_count() << ", 指向内存的地址为: " <<p1.get() << "\n"; // 0
    std::cout << "p2 引用计数为 : " << p2.use_count() << ", 指向内存的地址为: " <<p2.get() << "\n"; // 1

    std::cout << "p2 指向的内存地址数据: " << p2->c_str() << "\n"; //调用成员方法
    
    return 0;
}