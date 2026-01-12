import os
import subprocess
import threading
import argparse

parser = argparse.ArgumentParser(
    description="使用clang-format批量格式化plan项目代码"
)
parser.add_argument("--verbose", action="store_true")
args = parser.parse_args()

# 定义要格式化的文件夹列表
root_dirs = [
    "./src",
]
clang_format_path = "clang-format"
file_types = [".cc", ".cpp", ".hpp", ".h"]


def run_clang_format(file_list, verbose=False):
    for file_path in file_list:
        if verbose:
            print(f"正在格式化文件：{file_path}")
        subprocess.call([clang_format_path, "-i", file_path, "--style=file"])
        if verbose:
            print(f"文件格式化完成：{file_path}")


def traverse_and_collect(root_dir):
    file_list = []
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            file_path = os.path.join(root, file)
            if file_path.endswith(tuple(file_types)):
                file_list.append(file_path)
    if file_list:
        run_clang_format(file_list, args.verbose)


threads = []
for root_dir in root_dirs:
    thread = threading.Thread(target=traverse_and_collect, args=(root_dir,))
    thread.start()
    threads.append(thread)

# 等待所有线程完成
for thread in threads:
    thread.join()
