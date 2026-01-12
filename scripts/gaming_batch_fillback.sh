#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd -P)
PLANNING_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"
TOP_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd -P)"

source ${SCRIPT_DIR}/fillback_setup.bash
cd ${TOP_DIR}

# 检查参数数量
if [ $# -ne 2 ]; then
    echo "Usage: $0 <data_folder> <output_folder>"
    exit 1
fi

# 获取数据文件夹的路径
DATA_FOLDER_PATH=$1
FOLDER_B_PATH=$2

# 检查输入文件夹是否存在
if [ ! -d "$DATA_FOLDER_PATH" ]; then
    echo "Error: Folder $DATA_FOLDER_PATH does not exist."
    exit 1
fi

# 检查输出文件夹是否存在，如果不存在则创建
if [ ! -d "$FOLDER_B_PATH" ]; then
    echo "Folder $FOLDER_B_PATH does not exist. Creating it..."
    mkdir -p "$FOLDER_B_PATH"
    if [ $? -ne 0 ]; then
        echo "Failed to create folder $FOLDER_B_PATH."
        exit 1
    fi
fi

# 遍历数据文件夹中的子文件夹
for SUB_FOLDER in "$DATA_FOLDER_PATH"/*/; do
    # 提取子文件夹名称
    SUB_FOLDER_NAME=$(basename "$SUB_FOLDER")

    # 检查子文件夹名称是否符合 "-sXX" 的格式
    if [[ $SUB_FOLDER_NAME =~ -s([0-9]+)$ ]]; then
        # 提取数字后缀 XX
        SUFFIX=${BASH_REMATCH[1]}
        echo "Subfolder name: $SUB_FOLDER_NAME, suffix: $SUFFIX"
    else
        echo "Subfolder name $SUB_FOLDER_NAME does not match the expected pattern '-sXX'. xx=0"
        SUFFIX="0"
    fi

    # 检查子文件夹中是否存在名称包含"record"字符的数据包
    RECORD_DATA_PATH=$(find "$SUB_FOLDER" -type f -name "*record*" | head -n 1)
    if [ -z "$RECORD_DATA_PATH" ]; then
        echo "No record data file found in $SUB_FOLDER. Skipping..."
        continue
    fi

    # 创建输出文件夹中对应的子文件夹
    TARGET_FOLDER="$FOLDER_B_PATH/$SUB_FOLDER_NAME"
    if [ -d "$TARGET_FOLDER" ]; then
        rm -rf "$TARGET_FOLDER"
    fi
    mkdir -p "$TARGET_FOLDER"
    if [ $? -ne 0 ]; then
        echo "Failed to create folder $TARGET_FOLDER."
        continue
    fi

    # 播放数据包，并传递 -s 参数
    cyber_recorder play -f "$RECORD_DATA_PATH" -s "$SUFFIX" -k /st/pnc/pilot_planning_result /st/pnc/planning_debugframe >/dev/null 2>&1 &
    PLAY_PID=$!
    echo "Playing record data from $RECORD_DATA_PATH with suffix $SUFFIX (PID $PLAY_PID)..."

    # 启动planning程序
    mainboard -d ${PLANNING_DIR}/dag/planning_fillback.dag >/dev/null 2>&1 &
    PLANNING_PID=$!
    if [ $? -ne 0 ]; then
        echo "Failed to start planning program."
        exit 1
    fi
    echo "Planning program started with PID $PLANNING_PID."

    # 开始录包
    cyber_recorder record -a -o "$TARGET_FOLDER/fillback.record" >/dev/null 2>&1 &
    RECORD_PID=$!
    echo "Recording data with PID $RECORD_PID..."

    # 等待播放进程结束
    wait $PLAY_PID
    echo "Playback of record data from $RECORD_DATA_PATH finished."

    # 结束planning程序
    kill $PLANNING_PID
    wait $PLANNING_PID 2>/dev/null
    echo "Planning program terminated."

    # 找到新录制的数据包并移动到目标文件夹
    #mv ./*.record* "$TARGET_FOLDER"

    # 等待5秒
    echo "Waiting for 5 seconds before processing the next subfolder..."
    sleep 5
    # 杀掉录包进程
    kill -SIGINT $RECORD_PID
    wait $RECORD_PID 2>/dev/null
    echo "Recording process with PID $RECORD_PID terminated."
done

echo "All operations completed."
