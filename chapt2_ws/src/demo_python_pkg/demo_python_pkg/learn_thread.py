import threading # 使用多线程需要
import requests

class Download:
    def download(self,url,callback_world_count):
        print(f'线程编号:{threading.get_ident()}开始下载:{url}')
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback_world_count(url,response.text) #调用回调函数

    def start_download(self,url,callback_world_count):
        # self.download(url,callback_world_count)
        thread = threading.Thread(target=self.download,args=(url,callback_world_count))
        thread.start()


def world_count(url,result):
    '''
    普通函数 用于回调   
    '''
    print(f"{url}:{len(result)}->{result[:5]}")

def main():
    download  = Download()
    download.start_download('http://localhost:8001/novel1.txt',world_count)
    download.start_download('http://localhost:8001/novel2.txt',world_count)
    download.start_download('http://localhost:8001/novel3.txt',world_count)

