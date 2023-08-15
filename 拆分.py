import os
import shutil

# 原始文件夹路径
source_folder = "C:/Users/pengchen/Desktop/puncture/Apriltag/patientMotion3"

# 目标文件夹路径（L 图片）
target_folder_l = "C:/Users/pengchen/Desktop/puncture/Apriltag/L"

# 目标文件夹路径（R 图片）
target_folder_r = "C:/Users/pengchen/Desktop/puncture/Apriltag/R"

# 确保目标文件夹存在，如果不存在则创建
os.makedirs(target_folder_l, exist_ok=True)
os.makedirs(target_folder_r, exist_ok=True)

# 遍历原始文件夹中的所有文件
for filename in os.listdir(source_folder):
    if filename.endswith("L.jpg"):
        source_file = os.path.join(source_folder, filename)
        target_file = os.path.join(target_folder_l, filename)
        shutil.copy(source_file, target_file)
        print(f"Copied {filename} to {target_folder_l}")
    elif filename.endswith("R.jpg"):
        source_file = os.path.join(source_folder, filename)
        target_file = os.path.join(target_folder_r, filename)
        shutil.copy(source_file, target_file)
        print(f"Copied {filename} to {target_folder_r}")

