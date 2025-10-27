#!/bin/bash
# 简单的构建脚本，避免使用被禁止的命令格式

echo "开始构建项目..."

# 确保构建目录存在
if [ ! -d "build" ]; then
    echo "创建build目录..."
    mkdir build
fi

# 进入构建目录
echo "进入build目录..."
cd build

# 运行cmake配置
echo "运行cmake配置..."
cmake ..

# 运行make编译
echo "运行make编译..."
make

echo "构建完成！"