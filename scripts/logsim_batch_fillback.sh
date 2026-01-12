#!/bin/bash

# Script：logsim_batch_fillback.sh
# Usage：./logsim_batch_fillback.sh -i <input_path> -o <output_folder> -m <logsim_mode>(optional)

print_help_and_exit() {
  echo "Usage: $0 -i <input_path> -o <output_folder> -m <logsim_mode>(optional)"
  echo "  -i: <input_path>（File or Folder）"
  echo "  -o: <output_folder>"
  echo "  -m: <logsim_mode>(optional) close / open, default open"
  exit 1
}

while getopts "i:o:m:h:" opt; do
  case $opt in
    i) INPUT_PATH="$OPTARG"
    ;;
    o) OUTPUT_FOLDER="$OPTARG"
    ;;
    m)
      if [[ "$OPTARG" == "open" || "$OPTARG" == "close" ]]; then
          LOGSIM_MODE="$OPTARG"
       else
          echo "Invalid value for LOGSIM_MODE: $OPTARG. Expected 'open' or 'close'." >&2
          print_help_and_exit
       fi
    ;;
    \?) echo "Invalid: -$OPTARG" >&2
        print_help_and_exit
    ;;
  esac
done

if [ -z "$INPUT_PATH" ] || [ -z "$OUTPUT_FOLDER" ]; then
  print_help_and_exit
fi

mkdir -p "$OUTPUT_FOLDER"
BINARY_PATH="/apollo/bazel-bin/modules/cnoa_pnc/planning/offboard/logsim_fillback/logsim_fillback"
if [ ! -f "$BINARY_PATH" ]; then
    echo "Error: Can Not Find $BINARY_PATH"
    exit 1
fi

# process single file
process_file() {
  local input_file="$1"
  local output_folder="$2"
  local logsim_mode="$3"

  echo "Fillback file: $input_file"
  "$BINARY_PATH" "$input_file" "$output_folder" "$logsim_mode"

  if [ $? -ne 0 ]; then
    echo "Error: Fillback $input_file Failed"
    return 1
  fi
  return 0
}

if [ -f "$INPUT_PATH" ]; then
  process_file "$INPUT_PATH" "$OUTPUT_FOLDER" "$LOGSIM_MODE"
elif [ -d "$INPUT_PATH" ]; then
  echo "Fillback Folder: $INPUT_PATH"

  find "$INPUT_PATH" -maxdepth 1 -type f \( -name "*.record" -o -name "*.record.*" \) | while read -r file; do
    process_file "$file" "$OUTPUT_FOLDER" "$LOGSIM_MODE"
  done
else
    echo "Error: Input Path Is Not Exsted: $INPUT_PATH"
    exit 1
fi

echo "All Records in Folder Fillback Finished!"
exit 0
