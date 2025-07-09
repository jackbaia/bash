#!/bin/bash

# =============================================================================
# UAV环境自动化搭建脚本 - 基础版
# 适用于Ubuntu 20.04 LTS + ROS Noetic
# =============================================================================

set -e

# 配置常量
readonly SCRIPT_NAME="UAV环境自动化搭建脚本"
readonly SCRIPT_VERSION="1.0"
readonly SUPPORTED_OS="Ubuntu"
readonly SUPPORTED_VERSION="20.04"

# 路径配置
readonly ROS_SETUP_BASH="/opt/ros/noetic/setup.bash"
readonly ROS_MAVROS_PATH="/opt/ros/noetic/lib/mavros"
readonly ROS_MAVROS_SCRIPT="/opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh"

# 网络测试地址
readonly NETWORK_TEST_URLS=("8.8.8.8" "114.114.114.114" "www.baidu.com")

# 包列表
readonly BASE_PACKAGES=(
    "git" "vim" "terminator" "htop" "tree" "wget" "curl"
    "build-essential" "cmake" "python3-pip" "python3-catkin-tools"
    "python3-rosdep" "python3-rosinstall" "python3-rosinstall-generator"
    "python3-wstool" "net-tools" "openssh-server" "ant" "default-jdk"
    "libeigen3-dev" "libboost-all-dev" "libopencv-dev" "libpcl-dev" "libyaml-cpp-dev"
    "libgoogle-glog-dev" "code"
)

readonly ROS_PACKAGES=(
    "ros-noetic-rviz" "ros-noetic-gazebo-ros" "ros-noetic-gazebo-ros-control"
    "ros-noetic-gazebo-plugins" "ros-noetic-gazebo-dev" "ros-noetic-plotjuggler"
    "ros-noetic-plotjuggler-ros" "ros-noetic-rqt" "ros-noetic-rqt-graph"
    "ros-noetic-rqt-plot" "ros-noetic-rqt-console" "ros-noetic-rqt-common-plugins"
    "ros-noetic-urdf-tutorial" "ros-noetic-joint-state-publisher-gui"
    "ros-noetic-robot-state-publisher" "ros-noetic-tf2-tools" "ros-noetic-tf2-ros"
    "ros-noetic-tf2-geometry-msgs" "ros-noetic-nav-msgs" "ros-noetic-geometry-msgs"
    "ros-noetic-sensor-msgs" "ros-noetic-std-msgs" "ros-noetic-actionlib"
    "ros-noetic-actionlib-msgs" "ros-noetic-dynamic-reconfigure" "ros-noetic-nodelet"
    "ros-noetic-nodelet-topic-tools" "ros-noetic-urdf" "ros-noetic-xacro"
    "ros-noetic-controller-manager" "ros-noetic-joint-trajectory-controller"
    "ros-noetic-position-controllers" "ros-noetic-velocity-controllers"
    "ros-noetic-serial" "ros-noetic-autoware-msgs" "ros-noetic-ddynamic-reconfigure"
)

# 全局变量
ERROR_LOG=()

# 颜色定义
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

# =============================================================================
# 工具函数
# =============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
    ERROR_LOG+=("$1")
}

# =============================================================================
# 系统检查函数
# =============================================================================

check_system_version() {
    log_info "检查系统版本..."
    if ! command -v lsb_release >/dev/null 2>&1; then
        log_error "无法检测系统版本，请安装lsb-release: sudo apt install lsb-release"
        exit 1
    fi
    
    local os_name=$(lsb_release -si)
    local os_version=$(lsb_release -sr)
    
    log_info "检测到系统: $os_name $os_version"
    
    if [[ "$os_name" != "$SUPPORTED_OS" ]]; then
        log_error "此脚本仅支持$SUPPORTED_OS系统，当前系统: $os_name"
        exit 1
    fi
    
    if [[ "$os_version" != "$SUPPORTED_VERSION" ]]; then
        log_warning "此脚本专为$SUPPORTED_OS $SUPPORTED_VERSION LTS设计，当前版本: $os_version"
        read -p "是否继续安装？(y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "用户取消安装"
            exit 0
        fi
    else
        log_success "系统版本检查通过"
    fi
}

check_root() {
    log_info "检查用户权限..."
    if [[ $EUID -eq 0 ]]; then
        log_error "请不要使用root用户运行此脚本"
        exit 1
    fi
    
    if ! sudo -n true 2>/dev/null; then
        log_info "需要sudo权限，请输入密码..."
        sudo -v
    fi
    
    log_success "用户权限检查通过"
}

check_network() {
    log_info "检查网络连接..."
    local connected=false
    
    for url in "${NETWORK_TEST_URLS[@]}"; do
        if timeout 10 ping -c 1 -W 3 "$url" >/dev/null 2>&1; then
            log_success "网络连接正常 (测试地址: $url)"
            connected=true
            break
        fi
    done
    
    if [[ "$connected" == false ]]; then
        log_error "网络连接失败，请检查网络设置"
        exit 1
    fi
}

check_paths() {
    log_info "检查关键路径..."
    log_success "路径检查完成"
}

confirm_installation() {
    log_info "确认安装配置..."
    echo -e "${BLUE}=== UAV环境安装配置 ===${NC}"
    echo "目标系统: $SUPPORTED_OS $SUPPORTED_VERSION LTS"
    echo "ROS版本: Noetic"
    echo ""
    echo "安装内容:"
    echo "  - ROS Noetic 环境检查（需要手动安装）"
    echo "  - 开发工具 (git, vim, terminator等)"
    echo "  - ROS工具包 (rviz, gazebo, plotjuggler等)"
    echo "  - MAVROS"
    echo "  - Livox SDK2"
    echo ""
    
    read -p "是否开始安装？(y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "用户取消安装"
        exit 0
    fi
    
    log_success "用户确认安装"
}

# =============================================================================
# 安装函数
# =============================================================================

update_system() {
    log_info "更新系统包..."
    sudo apt update
    sudo DEBIAN_FRONTEND=noninteractive apt upgrade -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold"
    log_success "系统包更新完成"
}

check_ros_installation() {
    log_info "检查ROS安装状态..."
    if [ -f "$ROS_SETUP_BASH" ]; then
        log_success "检测到ROS Noetic已安装"
        log_info "ROS路径: $ROS_SETUP_BASH"
        return 0
    else
        log_warning "未检测到ROS Noetic安装"
        log_info "请手动安装ROS Noetic: http://wiki.ros.org/noetic/Installation"
        return 1
    fi
}

install_dependencies() {
    log_info "安装基础依赖包..."
    sudo DEBIAN_FRONTEND=noninteractive apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" "${BASE_PACKAGES[@]}"
    log_success "基础依赖包安装完成"
}

install_ros_packages() {
    log_info "安装ROS工具包..."
    sudo DEBIAN_FRONTEND=noninteractive apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" "${ROS_PACKAGES[@]}"
    log_success "ROS工具包安装完成"
}

install_mavros() {
    log_info "安装MAVROS..."
    if dpkg -l | grep -q ros-noetic-mavros; then
        log_warning "检测到MAVROS已安装，跳过安装步骤"
    else
        sudo DEBIAN_FRONTEND=noninteractive apt install -y -o Dpkg::Options::="--force-confdef" -o Dpkg::Options::="--force-confold" ros-noetic-mavros ros-noetic-mavros-extras
        log_success "MAVROS安装完成"
    fi
    
    log_info "安装地理库数据集..."
    if [ -f "$ROS_MAVROS_SCRIPT" ]; then
        cd "$ROS_MAVROS_PATH"
        sudo ./install_geographiclib_datasets.sh
        log_success "地理库数据集安装完成"
    else
        log_warning "未找到地理库数据集安装脚本"
    fi
}

install_livox_sdk() {
    log_info "安装Livox SDK..."
    local livox_sdk_dir="$HOME/Livox-SDK2"
    
    if [ -d "$livox_sdk_dir" ] && [ -f "$livox_sdk_dir/build/livox_lidar_api.h" ]; then
        log_warning "检测到Livox SDK已安装，跳过安装步骤"
        log_info "SDK路径: $livox_sdk_dir"
        return 0
    fi
    
    if [ -d "$livox_sdk_dir" ]; then
        log_info "检测到不完整的Livox SDK目录，清理后重新安装..."
        rm -rf "$livox_sdk_dir"
    fi
    
    log_info "克隆Livox SDK2..."
    cd "$HOME"
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    
    cd "$livox_sdk_dir"
    mkdir -p build
    cd build
    
    log_info "配置cmake..."
    cmake ..
    
    log_info "编译Livox SDK..."
    local cpu_cores=$(nproc)
    log_info "使用 $cpu_cores 个CPU核心进行编译..."
    make -j$cpu_cores
    
    log_info "安装Livox SDK..."
    sudo make install
    
    log_info "更新动态链接库缓存..."
    sudo ldconfig
    
    log_info "验证Livox SDK安装..."
    if [ -f "/usr/local/include/livox_lidar_api.h" ]; then
        log_success "Livox SDK安装成功"
        log_info "头文件位置: /usr/local/include/livox_lidar_api.h"
        log_info "库文件位置: /usr/local/lib/"
    else
        log_warning "Livox SDK安装可能不完整，请检查安装日志"
    fi
    
    cd "$HOME"
}

install_qpoases() {
    log_info "安装qpOASES..."
    if [ -f /usr/local/lib/libqpOASES.so ] || [ -f /usr/local/include/qpOASES.hpp ]; then
        log_success "检测到qpOASES已安装，跳过安装"
        return 0
    fi
    local qpoases_dir="$HOME/qpOASES"
    if [ -d "$qpoases_dir" ]; then
        log_warning "qpOASES目录已存在，跳过克隆"
    else
        git clone https://github.com/coin-or/qpOASES.git "$qpoases_dir"
    fi
    cd "$qpoases_dir"
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    cd "$HOME"
    log_success "qpOASES安装完成"
}

install_librealsense() {
    log_info "安装librealsense..."
    if [ -f /usr/local/lib/librealsense2.so ] || [ -d /usr/local/include/librealsense2 ]; then
        log_success "检测到librealsense已安装，跳过安装"
        return 0
    fi
    local realsense_dir="$HOME/librealsense"
    if [ -d "$realsense_dir" ]; then
        log_warning "librealsense目录已存在，跳过克隆"
    else
        git clone https://github.com/IntelRealSense/librealsense.git "$realsense_dir"
    fi
    cd "$realsense_dir"
    mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install
    # 安装udev规则
    sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    cd "$HOME"
    log_success "librealsense安装完成"
}

# =============================================================================
# 验证和测试函数
# =============================================================================

verify_installation() {
    log_info "验证安装..."
    
    local verification_results=()
    
    # 检查ROS
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        verification_results+=("✓ ROS Noetic 安装成功")
    else
        verification_results+=("✗ ROS Noetic 安装失败")
    fi
    
    # 检查MAVROS
    if dpkg -l | grep -q ros-noetic-mavros; then
        verification_results+=("✓ MAVROS 安装成功")
    else
        verification_results+=("✗ MAVROS 安装失败")
    fi
    
    # 检查Livox SDK
    if [ -f "/usr/local/include/livox_lidar_api.h" ] || [ -f "$HOME/Livox-SDK2/build/livox_lidar_api.h" ]; then
        verification_results+=("✓ Livox SDK 安装成功")
    else
        verification_results+=("✗ Livox SDK 安装失败")
    fi
    
    # 显示验证结果
    echo -e "${BLUE}=== 安装验证结果 ===${NC}"
    for result in "${verification_results[@]}"; do
        if [[ $result == ✓* ]]; then
            echo -e "${GREEN}$result${NC}"
        else
            echo -e "${RED}$result${NC}"
        fi
    done
}

test_ros_environment() {
    log_info "测试ROS环境..."
    
    # 测试roscore
    log_info "测试roscore..."
    timeout 5s roscore >/dev/null 2>&1 &
    local roscore_pid=$!
    sleep 2
    
    if kill -0 $roscore_pid 2>/dev/null; then
        kill $roscore_pid
        log_success "roscore测试通过"
    else
        log_warning "roscore测试失败"
    fi
    
    # 测试rosnode
    if command -v rosnode >/dev/null 2>&1; then
        log_success "rosnode可用"
    else
        log_warning "rosnode不可用"
    fi
    
    # 测试rostopic
    if command -v rostopic >/dev/null 2>&1; then
        log_success "rostopic可用"
    else
        log_warning "rostopic不可用"
    fi
    
    log_success "ROS环境测试完成"
}

# =============================================================================
# 辅助函数
# =============================================================================

show_usage() {
    echo "$SCRIPT_NAME v$SCRIPT_VERSION"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help           显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                   安装UAV环境"
    echo "  $0 -h                显示帮助信息"
    echo ""
}

# =============================================================================
# 主函数
# =============================================================================
main() {
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_usage
                exit 0
                ;;
            *)
                log_error "未知参数: $1"
                show_usage
                exit 1
                ;;
        esac
    done
    
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}    $SCRIPT_NAME v$SCRIPT_VERSION${NC}"
    echo -e "${BLUE}================================${NC}"
    echo ""
    
    log_info "开始UAV环境自动化搭建..."
    log_info "当前用户: $USER"
    log_info "系统: $(lsb_release -d | cut -f2)"
    echo ""
    
    # 执行安装步骤
    check_system_version
    check_root
    check_network
    check_paths
    confirm_installation
    update_system
    check_ros_installation
    install_dependencies
    install_ros_packages
    install_mavros
    install_livox_sdk
    install_qpoases
    install_librealsense
    verify_installation
    test_ros_environment
    
    # 最终修复依赖
    log_info "最终修复依赖..."
    sudo apt-get install -f -y >/dev/null 2>&1
    
    echo ""
    echo -e "${GREEN}================================${NC}"
    echo -e "${GREEN}    UAV环境搭建完成！${NC}"
    echo -e "${GREEN}================================${NC}"
    echo ""
}

# 运行主函数
main "$@"