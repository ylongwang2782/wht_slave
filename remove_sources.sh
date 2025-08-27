#!/bin/bash
# usage: ./remove_sources.sh <project_dir>
# example: ./remove_sources.sh ./MyProject

PROJ_DIR=$1

if [ -z "$PROJ_DIR" ]; then
  echo "Usage: $0 <project_dir>"
  exit 1
fi

# 删除所有 .c 和 .cpp 文件
find "$PROJ_DIR" -type f \( -name "*.c" -o -name "*.cpp" \) -exec rm -v {} \;

echo "All .c and .cpp files removed from $PROJ_DIR"
