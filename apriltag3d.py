
import cv2
import numpy as np
from pupil_apriltags import Detector

# 读取图像
img_left = cv2.imread("C:\\Users\\pengchen\\Desktop\\L\\CalibrationImg40_20230803-18-04-05.111_L.jpg")
gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)

# 定义左相机的内参矩阵和畸变系数
intrinsics_matrix_left = np.array([[3504.895, 0, 1260.197],
                                   [0, 3505.386, 967.865],
                                   [0, 0, 1]])
distortion_coeffs_left = np.array([-0.09625854, 0.2608055, 0.0003778949, 0.0007970959, 0])

# 创建AprilTag检测器
at_detector = Detector(families='tag36h11')

# 进行AprilTag检测
tags_left = at_detector.detect(gray_left)

# 定义AprilTag角点在其自身坐标系中的3D坐标（毫米）
object_points = np.array([
    [-5.76 / 2, -5.76 / 2, 0],
    [5.76 / 2, -5.76 / 2, 0],
    [5.76 / 2, 5.76 / 2, 0],
    [-5.76 / 2, 5.76 / 2, 0]
])

# 定义坐标轴点在自身坐标系中的3D坐标
axis_points = np.float32([[10,0,0], [0,10,0], [0,0,10]]).reshape(-1,3)

# 处理每个检测到的AprilTag
for tag_left in tags_left:
    # 使用OpenCV的solvePnP计算姿势
    ret, rvec, tvec = cv2.solvePnP(object_points, tag_left.corners.reshape(-1, 1, 2), intrinsics_matrix_left, distortion_coeffs_left)
    R_matrix, _ = cv2.Rodrigues(rvec)

    print("标签ID:", tag_left.tag_id)
    print("旋转矩阵:")
    print(R_matrix)
    print("平移向量:")
    print(tvec)  
    print("---------------------------------------")

    # 将3D坐标轴点投影到图像平面
    imgpts, jac = cv2.projectPoints(axis_points, rvec, tvec, intrinsics_matrix_left, distortion_coeffs_left)
    
    # 绘制坐标轴


    # 绘制AprilTag边界和中心
    for i in range(4):
        cv2.line(img_left, tuple(tag_left.corners[i].astype(int)), tuple(tag_left.corners[(i + 1) % 4].astype(int)), (0, 255, 0), 2)
        cv2.circle(img_left, tuple(tag_left.center.astype(int)), 4, (0, 0, 255), 4)
        cv2.putText(img_left, str(i), tuple(tag_left.corners[i].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)



# 调整窗口大小以适应整张图像
height, width = img_left.shape[:2]
max_height = 800
scaling_factor = max_height / float(height)
resized_image = cv2.resize(img_left, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)

# 显示标注后的图像
cv2.imshow('Detected AprilTags', resized_image)
cv2.waitKey(0)
cv2.destroyAllWindows()