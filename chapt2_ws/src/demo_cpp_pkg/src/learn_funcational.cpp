#include <iostream>
#include <functional> //函数包装器头文件

//自由函数
void save_with_free_fun(const std::string &file_name)
{
    std::cout << "调用了自由函数，保存:" << file_name << std::endl;
}

//成员函数
class FileSave
{
public:
    void save_with_member_fun(const std::string &file_name)
    {
        std::cout << "调用了成员方法，保存:" << file_name << std::endl;
    }
};

int main()
{
    FileSave fs;

    // lambda函数 
    auto save_with_lambda_fun = [](const std::string &file_name) ->void
    {
        std::cout << "调用了Lambda 函数，保存:" << file_name << std::endl;
    };

    // //三种调用方式
    // save_with_free_fun("file.txt");
    // fs.save_with_member_fun("file.txt");
    // save_with_lambda_fun("file.txt");


    // 将自由函数放进function对象中，void为返回类型
    std::function<void(const std::string &)> save1 = save_with_free_fun;
    // 将Lambda函数放入function对象中
    std::function<void(const std::string &)> save2 = save_with_lambda_fun;
    // 将成员方法放入包装器
    std::function<void(const std::string &)> save3 = std::bind(&FileSave::save_with_member_fun, &fs, std::placeholders::_1);
    
    // 无论哪种函数都可以使用统一的调用方式
    save1("file.txt");
    save2("file.txt");
    save3("file.txt");
    return 0;
}