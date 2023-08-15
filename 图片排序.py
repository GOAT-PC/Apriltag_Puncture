import os
import re

def get_number_from_filename(filename):
    match = re.search(r'CalibrationImg(\d+)_', filename)
    if match:
        return int(match.group(1))
    return None

def sort_images(folder_path):
    filenames = [filename for filename in os.listdir(folder_path) if filename.startswith('CalibrationImg')]
    sorted_filenames = sorted(filenames, key=get_number_from_filename)

    for idx, filename in enumerate(sorted_filenames):
        old_path = os.path.join(folder_path, filename)
        # 创建临时文件名，以避免潜在的文件名冲突
        temp_path = os.path.join(folder_path, f'temp_{idx}.temp')
        os.rename(old_path, temp_path)

    # 将临时文件重命名为原始文件名
    for idx, filename in enumerate(sorted_filenames):
        temp_path = os.path.join(folder_path, f'temp_{idx}.temp')
        new_path = os.path.join(folder_path, filename)
        os.rename(temp_path, new_path)

    print("Sorting completed successfully.")

folder_path = 'C:\\Users\\pengchen\\Desktop\\puncture\\Apriltag\\L'  # 替换为你的文件夹路径
sort_images(folder_path)
