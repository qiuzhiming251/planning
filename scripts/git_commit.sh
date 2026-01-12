#!/bin/bash

# 获取所有子模块的路径
submodules=$(git submodule --quiet foreach --recursive 'echo $path')

# 更新所有子模块到最新状态
# for submodule in $submodules; do
#     cd "$submodule"
#     git pull --rebase
#     cd -
# done

# 提交所有子模块的更改
commit_message="sync optimization thread pool codes from build_demo"
for submodule in $submodules; do
    cd "$submodule"
    git checkout dev
    git pull
    git add .
    git commit -m "$commit_message"
    git push
    cd -
done

# 更新主仓库中的子模块引用
git add .
git commit -m "$commit_message"
# git push
# # 推送更改到远程仓库
# git push origin main
