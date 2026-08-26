// emergency_stop_node.cpp
// 底层 CAN 旁路急停节点
// 独立打开 can0~can3，直接向总线广播失能帧，不改动原有控制节点

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class EmergencyStopNode : public rclcpp::Node {
public:
    EmergencyStopNode() : Node("emergency_stop_node"), running_(true) {
        // CAN 接口名（可通过参数覆盖）
        declare_parameter("can_interfaces",
                          std::vector<std::string>{"can0", "can1", "can2", "can3"});
        can_interface_names_ = get_parameter("can_interfaces").as_string_array();

        // 底盘状态话题（chassis_control 发布的 JSON 字符串）
        declare_parameter("chassis_status_topic", "chassis/status");
        const auto chassis_status_topic =
            get_parameter("chassis_status_topic").as_string();
        declare_parameter("chassis_state_cmd_topic", "chassis/state_cmd");
        const auto chassis_state_cmd_topic =
            get_parameter("chassis_state_cmd_topic").as_string();

        initCanSockets();
        initMotorConfigs();
        chassis_state_pub_ = create_publisher<std_msgs::msg::String>(
            chassis_state_cmd_topic, 10);

        // ROS2 服务：~/trigger（备用触发途径）
        trigger_service_ = create_service<std_srvs::srv::Trigger>(
            "~/trigger",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                triggerEmergencyStop();
                response->success = true;
                response->message = "Emergency stop triggered";
            });

        // 订阅底盘状态：进入急停(estop)状态时立即触发失能
        chassis_status_sub_ = create_subscription<std_msgs::msg::String>(
            chassis_status_topic, 10,
            std::bind(&EmergencyStopNode::onChassisStatus, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "已订阅底盘状态话题: %s", chassis_status_topic.c_str());

        // 键盘监听线程（仅当 stdin 是终端时启动）
        if (isatty(STDIN_FILENO)) {
            keyboard_thread_ = std::thread(&EmergencyStopNode::keyboardLoop, this);
            RCLCPP_INFO(get_logger(), "急停节点已启动。按 [e] 键触发急停，或调用 ~/trigger 服务。");
        } else {
            RCLCPP_INFO(get_logger(), "急停节点已启动（非终端模式）。通过 ~/trigger 服务触发。");
        }
    }

    ~EmergencyStopNode() {
        running_ = false;
        // 恢复终端
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        for (int fd : can_socket_fds_) {
            if (fd >= 0) close(fd);
        }
    }

private:
    // 单个电机的失能配置
    struct MotorDisableConfig {
        int can_index;                 // 对应 can_interface_names_ 的下标
        canid_t can_id;                // CAN 仲裁 ID
        std::vector<uint8_t> data;     // 帧数据（≤8 字节）
        int repeat_count;              // 发送次数
        int interval_ms;               // 发送间隔
    };

    // ── 初始化 CAN sockets ──────────────────────────────────────────────
    void initCanSockets() {
        can_socket_fds_.clear();
        for (const auto& name : can_interface_names_) {
            int fd = openCanSocket(name);
            can_socket_fds_.push_back(fd);
            if (fd >= 0) {
                RCLCPP_INFO(get_logger(), "已打开 CAN 接口: %s (fd=%d)", name.c_str(), fd);
            } else {
                RCLCPP_WARN(get_logger(), "无法打开 CAN 接口: %s（跳过）", name.c_str());
            }
        }
    }

    static int openCanSocket(const std::string& interface) {
        int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd < 0) return -1;

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
            close(fd);
            return -1;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(fd);
            return -1;
        }
        return fd;
    }

    // ── 加载电机失能配置 ────────────────────────────────────────────────
    void initMotorConfigs() {
        // 达妙电机失能帧：0xFD 命令
        const std::vector<uint8_t> dm_disable = {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

        // can0: 右臂 7 关节 + 夹爪 (0x01~0x08)
        for (uint32_t id = 0x01; id <= 0x08; ++id) {
            motor_configs_.push_back({0, id, dm_disable, 5, 2});
        }

        // can1: 左臂 7 关节 + 夹爪 (0x01~0x08)
        for (uint32_t id = 0x01; id <= 0x08; ++id) {
            motor_configs_.push_back({1, id, dm_disable, 5, 2});
        }

        // can2: 腰部 (0x01, DMJ10422P)
        motor_configs_.push_back({2, 0x01, dm_disable, 5, 2});

        // can3: 升降 (CANopen SDO write, node_id=2)
        // 写 0x6040 (ControlWord) = 0x0007 (DISABLE_OPERATION)
        // CAN ID = 0x600 + 2 = 0x602
        const std::vector<uint8_t> canopen_disable = {
            0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
        motor_configs_.push_back({3, 0x602, canopen_disable, 3, 10});

        RCLCPP_INFO(get_logger(), "已加载 %zu 个电机失能配置", motor_configs_.size());
    }

    // ── 触发急停 ────────────────────────────────────────────────────────
    void triggerEmergencyStop() {
        std::lock_guard<std::mutex> lock(trigger_mutex_);
        RCLCPP_WARN(get_logger(), "=== 急停触发！正在失能底盘并发送失能帧 ===");

        std_msgs::msg::String chassis_command;
        chassis_command.data = "disable";
        chassis_state_pub_->publish(chassis_command);

        for (const auto& cfg : motor_configs_) {
            if (cfg.can_index >= static_cast<int>(can_socket_fds_.size())) continue;
            int fd = can_socket_fds_[cfg.can_index];
            if (fd < 0) continue;

            for (int i = 0; i < cfg.repeat_count; ++i) {
                sendCanFrame(fd, cfg.can_id, cfg.data);
                std::this_thread::sleep_for(std::chrono::milliseconds(cfg.interval_ms));
            }
        }

        RCLCPP_WARN(get_logger(), "=== 急停处理完成 ===");
    }

    // ── 底盘状态回调 ────────────────────────────────────────────────────
    // chassis_control 以 JSON 字符串发布状态，急停时含 "state_name": "estop"。
    // 仅在“进入”急停状态的上升沿触发一次，避免持续刷帧。
    void onChassisStatus(const std_msgs::msg::String::SharedPtr msg) {
        const bool estop = msg->data.find("\"state_name\": \"estop\"") != std::string::npos;
        if (estop && !chassis_estop_) {
            RCLCPP_WARN(get_logger(), "检测到底盘急停状态，联动触发急停！");
            triggerEmergencyStop();
        }
        chassis_estop_ = estop;
    }

    static bool sendCanFrame(int fd, canid_t can_id, const std::vector<uint8_t>& data) {
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = can_id;
        frame.can_dlc = std::min(static_cast<int>(data.size()), CAN_MAX_DLEN);
        std::memcpy(frame.data, data.data(), frame.can_dlc);
        return write(fd, &frame, sizeof(frame)) == sizeof(frame);
    }

    // ── 键盘监听线程 ────────────────────────────────────────────────────
    void keyboardLoop() {
        struct termios old_termios;
        tcgetattr(STDIN_FILENO, &old_termios);

        struct termios new_termios = old_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);  // 原始模式，无回显
        new_termios.c_cc[VMIN] = 1;
        new_termios.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

        char ch;
        while (running_) {
            // 用 select 带超时，避免 read 永久阻塞导致无法退出
            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(STDIN_FILENO, &read_fds);
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000;  // 100ms
            int ret = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &tv);
            if (ret > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {
                if (read(STDIN_FILENO, &ch, 1) > 0) {
                    if (ch == 'e' || ch == 'E') {
                        triggerEmergencyStop();
                    }
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios);
    }

    // ── 成员变量 ────────────────────────────────────────────────────────
    std::vector<std::string> can_interface_names_;
    std::vector<int> can_socket_fds_;
    std::vector<MotorDisableConfig> motor_configs_;
    std::thread keyboard_thread_;
    std::atomic<bool> running_;
    std::mutex trigger_mutex_;
    bool chassis_estop_ = false;  // 底盘急停状态边沿检测
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chassis_status_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chassis_state_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyStopNode>());
    rclcpp::shutdown();
    return 0;
}
