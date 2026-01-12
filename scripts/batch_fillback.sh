#!/bin/bash
#kill all existing processes to prevent conflicts
pkill -f mainboard
pkill -f cyber_record
# Determine the script directory
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd -P)
PLANNING_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"     # Parent directory of SCRIPT_DIR
TOP_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd -P)" # Root directory of the project

# Source the setup script for environment variables
source "${SCRIPT_DIR}/fillback_setup.bash"
cd "${TOP_DIR}" # Change to the top directory

# Check the number of parameters
if [ $# -ne 4 ]; then
    echo "Usage: $0 <data_file> <output_folder> <start_time> <end_time>"
    exit 1
fi

# Assign parameters to variables
DATA_PATH=$1
FOLDER_B_PATH=$2
START_TIME=$3
END_TIME=$4

# Check if the input file exists
if [ ! -f "$DATA_PATH" ]; then
    echo "Error: File $DATA_PATH does not exist."
    exit 1
fi

# Check if the output folder exists, create it if not
if [ ! -d "$FOLDER_B_PATH" ]; then
    echo "Folder $FOLDER_B_PATH does not exist. Creating it..."
    mkdir -p "$FOLDER_B_PATH"
    if [ $? -ne 0 ]; then
        echo "Failed to create folder $FOLDER_B_PATH."
        exit 1
    fi
fi

# Extract the record bag filename
echo "Record bag $DATA_PATH"
echo "Start time" $START_TIME
echo "End time" $END_TIME
FILENAME=$(basename "$DATA_PATH")
# Play the record bag in the background
cyber_recorder play -f "$DATA_PATH" -b "$START_TIME" -e "$END_TIME" -k /st/pnc/pilot_planning_result /st/pnc/planning_debugframe >/dev/null 2>&1 &
PLAY_PID=$!
echo "Playing record data from $DATA_PATH with PID $PLAY_PID..."

# # Start recording the data in the background
cyber_recorder record -a -s 0 -o "${FOLDER_B_PATH}/${FILENAME%.*}_fillback.record" >/dev/null 2>&1 &
RECORD_PID=$!
echo "Recording data with PID $RECORD_PID..."

# Start the planning program in the background
mainboard -d "${PLANNING_DIR}/dag/planning.dag" >/dev/null 2>&1 &
PLANNING_PID=$!
if [ $? -ne 0 ]; then
    echo "Failed to start planning program."
    exit 1
fi
echo "Planning program started with PID $PLANNING_PID."

# Wait for the playback process to finish
wait $PLAY_PID
echo "Playback of record data from $DATA_PATH finished."
sleep 5 # Allow some time for processing

# Terminate the recording process
kill -SIGINT $RECORD_PID
wait $RECORD_PID 2>/dev/null
echo "Recording process with PID $RECORD_PID terminated."
sleep 5 # Allow some time for the process to terminate

# Terminate the planning program
kill $PLANNING_PID
wait $PLANNING_PID 2>/dev/null
echo "Planning  program PID $PLANNING_PID terminated."

# Final wait before exiting
echo "Waiting for 2 seconds before processing ..."
sleep 2
