import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory # 获取功能包share目录绝对路径
import os

def main():
    # 获取图片真实路径，得到/root/study/chapt4/chapt4_ws/install/demo_python_service/share/demo-python_service
    # default_image_path = get_package_share_directory(
    #     'demo_python_service')+'/resource/default.jpg' # 第一个参数为功能包的名字
    # print(f"图片的真实路径:{default_image_path}")

    default_image_path = os.path.join(get_package_share_directory('demo_python_service'),'/resource/default.jpg') # 第一个参数为功能包的名字
    # os会自动判断需不需要加/，防止忘加

    print(f"图片的真实路径:{default_image_path}")

    # 使用 opencv 加载图像
    image = cv2.imread(default_image_path)

    # 查找图像中所有的人脸
    face_locations = face_recognition.face_locations(
        image, number_of_times_to_upsample=1, model='hog')
    # face_locations为人脸个数。第一个参数为图像，第二个参数为上采样的次数，越高的次数就能找到越小的点，默认值为1，第三个参数为识别人脸的模型默认为'hog'
    # 'hog'模型计算快但精度一般，除此之外还可以设置cnn模型

    # 绘制每个人脸的边框
    for top, right, bottom, left in face_locations:
        cv2.rectangle(image, (left, top), (right, bottom), (255, 0, 0), 4) 
        # 绘制矩形，第一个参数为图像；第二个参数为左上角的位置；第三个参数为矩形框右下角的位置；第四个参数为要绘制的边框的颜色；最后一个参数表示要设置的矩形框的宽度

    # 显示结果图像
    cv2.imshow('Face Detection', image) # 第一个参数为框的名字
    cv2.waitKey(0) # 表示一直等待我们按下这个按键他才会退出