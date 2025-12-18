#include <iostream>
#include <thread> //多线程
#include <chrono> //时间相关
#include <functional> // 函数包装器
#include <cpp-httplib/httplib.h> // 下载相关

class Download
{
public:
    void download(const std::string &host, const std::string &path, const std::function<void(const std::string &, const std::string &)> &callback)
    //host为主机地址；第二个为路径，其实是把完整的网址拆为前后两部分；第三个为回调函数，请求成功后调用，传递请求结果，回调函数两个参数，一个为地址一个为内容
    {
        std::cout << "线程ID: " << std::this_thread::get_id() << std::endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200)
        {
            callback(path, response->body);
        }
    }

    //启动下载函数，主要是创建线程
    void start_download(const std::string &host, const std::string &path, const std::function<void(const std::string &, const std::string &)> &callback)
    {
        auto download_fun = std::bind(&Download::download, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        std::thread download_thread(download_fun, host, path, callback);
        download_thread.detach();
    }
};

int main()
{
    Download download;
    auto download_finish_callback = [](const std::string &path, const std::string &result) -> void
    {
        std::cout << "下载完成：" << path << " 共：" << result.length() << "字，内容为：" << result.substr(0, 16) << std::endl;
    };

    download.start_download("http://localhost:8000", "/novel1.txt", download_finish_callback);//区分域名和地址
    download.start_download("http://localhost:8000", "/novel2.txt", download_finish_callback);
    download.start_download("http://localhost:8000", "/novel3.txt", download_finish_callback);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 10));// 休息1000ms * 10
    return 0;
}