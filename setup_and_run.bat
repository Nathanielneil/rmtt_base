@echo off
chcp 65001 >nul
echo.
echo ============================================
echo    RMTT - Tello无人机控制系统
echo ============================================
echo.

REM 检查Python是否安装
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo 错误: 未找到Python，请先安装Python 3.11
    pause
    exit /b 1
)

REM 检查conda是否可用
conda --version >nul 2>&1
if %errorlevel% equ 0 (
    echo 发现Conda环境管理器
    echo.
    echo 选择安装方式:
    echo 1. 使用Conda创建环境 (推荐)
    echo 2. 使用pip安装到当前Python环境
    echo 3. 跳过安装直接运行
    echo.
    set /p choice=请选择 (1-3): 
    
    if "!choice!"=="1" goto conda_install
    if "!choice!"=="2" goto pip_install
    if "!choice!"=="3" goto run_program
    goto pip_install
) else (
    echo 未发现Conda，使用pip安装依赖
    goto pip_install
)

:conda_install
echo.
echo 使用Conda创建环境...
conda env create -f environment.yml
if %errorlevel% neq 0 (
    echo 错误: Conda环境创建失败
    pause
    exit /b 1
)
echo.
echo 环境创建成功！
echo 激活环境: conda activate rmtt
echo 然后运行: python main.py
pause
exit /b 0

:pip_install
echo.
echo 使用pip安装依赖...
pip install -r requirements.txt
if %errorlevel% neq 0 (
    echo 警告: 部分依赖安装可能失败，但程序可能仍可运行
)
goto run_program

:run_program
echo.
echo ============================================
echo 准备启动程序...
echo.
echo 请确保:
echo 1. 无人机已开机
echo 2. 已连接到无人机WiFi热点 RMTT-A93874
echo 3. 在空旷安全的区域操作
echo ============================================
echo.
pause

python main.py

echo.
echo 程序已结束
pause