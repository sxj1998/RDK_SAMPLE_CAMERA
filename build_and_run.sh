@echo off

:: 进入项目目录
cd /d %~dp0

:: 创建build目录
if not exist build (mkdir build)

:: 进入build目录
cd build

:: 运行cmake配置
echo 正在配置项目...
cmake .. -DCMAKE_BUILD_TYPE=Release
if %ERRORLEVEL% neq 0 (
    echo CMake配置失败!
    exit /b 1
)

:: 编译项目
echo 正在编译项目...
cmake --build . --config Release --parallel 4
if %ERRORLEVEL% neq 0 (
    echo 编译失败!
    exit /b 1
)

:: 检查可执行文件是否存在
if exist Release\standalone_vin_project.exe (
    echo 编译成功!
    echo 正在运行程序...
    Release\standalone_vin_project.exe
) else (
    echo 编译成功，但未找到可执行文件!
)