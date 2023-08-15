import cv2
import numpy as np
from pupil_apriltags import Detector
from scipy.linalg import svd
import os
import xml.etree.ElementTree as ET

# 图像文件夹路径
left_folder_path = "C:/Users/pengchen/Desktop/L"
right_folder_path = "C:/Users/pengchen/Desktop/R"

# 获取左图和右图文件列表
left_image_files = sorted(os.listdir(left_folder_path))
right_image_files = sorted(os.listdir(right_folder_path))

# 创建AprilTag检测器
at_detector = Detector()

def calculate_transformation_matrix(left_image_path, right_image_path):
    # 读取图像
    img_left = cv2.imread(left_image_path)
    img_right = cv2.imread(right_image_path)
    gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

    # 定义左相机的内参矩阵和畸变系数
    intrinsics_matrix_left = np.array([[3504.895, 0, 1260.197],
                                    [0, 3505.386, 967.865],
                                    [0, 0, 1]])
    distortion_coeffs_left = np.array([-0.09625854, 0.2608055, 0.0003778949, 0.0007970959, 0])

    # 定义右相机的内参矩阵和畸变系数
    intrinsics_matrix_right = np.array([[3573.797, 0, 1241.063],
                                        [0, 3574.055, 966.152],
                                        [0, 0, 1]])
    distortion_coeffs_right = np.array([-0.05937385, 0.1189392, -0.000143728, 0.003604084, 0])

    # 矫正左侧图像
    img_left = cv2.undistort(img_left, intrinsics_matrix_left, distortion_coeffs_left)
    gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    img_right = cv2.undistort(img_right, intrinsics_matrix_right, distortion_coeffs_right)
    gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

    # 定义旋转和平移矩阵
    R = np.array([[0.9796001955760725, 0.002930991321463302, 0.2009349798248037],
                [-0.002537975543664841, 0.9999943294911776, -0.002213518834289528],
                [-0.200940328225721, 0.001658395418319203, 0.9796020795288136]])
    T = np.array([-62.72735790468046, 0, 0])

    # 进行双目矫正
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(intrinsics_matrix_left, distortion_coeffs_left,
                                                                    intrinsics_matrix_right, distortion_coeffs_right,
                                                                    gray_left.shape[::-1], R, T)

    left_map1, left_map2 = cv2.initUndistortRectifyMap(intrinsics_matrix_left, distortion_coeffs_left, R1, P1, gray_left.shape[::-1], cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(intrinsics_matrix_right, distortion_coeffs_right, R2, P2, gray_right.shape[::-1], cv2.CV_16SC2)

    img_left_rectified = cv2.remap(img_left, left_map1, left_map2, cv2.INTER_LINEAR)
    img_right_rectified = cv2.remap(img_right, right_map1, right_map2, cv2.INTER_LINEAR)
    gray_left_rectified = cv2.cvtColor(img_left_rectified, cv2.COLOR_BGR2GRAY)


    # 进行AprilTag检测
    tags_left = at_detector.detect(gray_left_rectified)

    # 定义AprilTag角点在其自身坐标系中的3D坐标（毫米）
    object_points = np.array([
        [-5.76 / 2, -5.76 / 2, 0],
        [5.76 / 2, -5.76 / 2, 0],
        [5.76 / 2, 5.76 / 2, 0],
        [-5.76 / 2, 5.76 / 2, 0]
    ])

    # 处理每个检测到的AprilTag
    all_object_points_3d = []
    points = np.array([])  # 初始化一个空数组作为默认值

    for tag_left in tags_left:
        
        # 使用OpenCV的solvePnP计算姿势
        ret, rvec, tvec = cv2.solvePnP(object_points, tag_left.corners.reshape(-1, 1, 2), intrinsics_matrix_left, distortion_coeffs_left)
        R_matrix, _ = cv2.Rodrigues(rvec)

        # 从每个标签中提取角点的3D坐标，并将它们堆叠在一起
        transformed_points = np.dot(R_matrix, object_points.T).T + tvec.T
        all_object_points_3d.extend(transformed_points)

    all_object_points_3d = np.array(all_object_points_3d)

    if len(all_object_points_3d) < 4:
            print("Not enough valid AprilTag detections.")
            return None  # 返回特殊值，表示循环应该跳过当前迭代
    else:
            points = all_object_points_3d[:4, :]
            return points

LITTLE_POINT = -1
L_CORRECT = 0

def PlaneEstimateByLS(points):
    if len(points) < 4:
        status = LITTLE_POINT
        return [], status
    
    xxyyzz = np.zeros((3, 3), dtype=np.float64)
    xyz = np.zeros((3, 1), dtype=np.float64)
    
    for i in range(len(points)):
        x, y, z = points[i]
        xxyyzz[0, 0] += x * x
        xxyyzz[0, 1] += x * y
        xxyyzz[0, 2] += x * z
        xxyyzz[1, 0] += x * y
        xxyyzz[1, 1] += y * y
        xxyyzz[1, 2] += y * z
        xxyyzz[2, 0] += x * z
        xxyyzz[2, 1] += y * z
        xxyyzz[2, 2] += z * z
        xyz[0, 0] += x
        xyz[1, 0] += y
        xyz[2, 0] += z
    
    result = np.linalg.inv(xxyyzz) @ xyz * -1
    ad = result[0, 0]
    bd = result[1, 0]
    cd = result[2, 0]
    D = -1 / np.sqrt(ad**2 + bd**2 + cd**2)
    A = D * ad
    B = D * bd
    C = D * cd
    status = L_CORRECT
    return [A, B, C, D], status


def compute_transformation_matrix(points):
    plane_coefficients, status = PlaneEstimateByLS(points)

    center_point = np.mean(points, axis=0)
    plane_normal = np.array(plane_coefficients[:3])
    plane_point = -plane_coefficients[3] * plane_normal

    projection_vector = center_point - plane_point
    projection_distance = np.dot(projection_vector, plane_normal)
    projected_point = center_point - projection_distance * plane_normal + plane_point

    point_3 = points[3]
    point_0 = points[0]

    projection_vector_3 = point_3 - plane_point
    projection_distance_3 = np.dot(projection_vector_3, plane_normal)
    projected_point_3 = point_3 - projection_distance_3 * plane_normal + plane_point

    projection_vector_0 = point_0 - plane_point
    projection_distance_0 = np.dot(projection_vector_0, plane_normal)
    projected_point_0 = point_0 - projection_distance_0 * plane_normal + plane_point

    x_axis = projected_point_3 - projected_point_0
    x_axis /= np.linalg.norm(x_axis)

    z_axis = plane_coefficients[:3]
    z_axis /= np.linalg.norm(z_axis)

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
    transformation_matrix[:3, 3] = center_point
    return transformation_matrix


# 创建一个txt文件并写入数据
with open("wzgj0.txt", "w") as txtfile:

    for left_file, right_file in zip(left_image_files, right_image_files):
            left_image_path = os.path.join(left_folder_path, left_file)
            right_image_path = os.path.join(right_folder_path, right_file)

            # 调用函数并获取变换矩阵
            transformation_matrix = calculate_transformation_matrix(left_image_path, right_image_path)
            if transformation_matrix is None:
                continue  # 跳过当前图像文件的处理，进入下一个图像文件的处理
            # 获取时间戳部分
            timestamp = left_file.split("_")[1].replace("-", " ")

            txtfile.write(timestamp + " ")
            for row in transformation_matrix:
                for value in row:
                    txtfile.write(f"{value:.10f} ")
            txtfile.write("\n")
        