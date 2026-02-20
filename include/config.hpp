/**
 * config.hpp - 集中配置：端口、常量、魔数
 * 换车/调参时主要修改此文件
 */
#ifndef CONFIG_HPP
#define CONFIG_HPP

// -------------------------------
// 硬件端口（换车时修改）
// -------------------------------
// 底盘电机：左 {3, -11, 12} 右 {15, -16, 17}（负号=反向）
// 机构：intake 18, arm 20, wing 14
// ADI：气管 'G'
// 传感器：编码轮 4, IMU 6

// -------------------------------
// 超时（毫秒，防死循环）
// -------------------------------
constexpr int IMU_CAL_TIMEOUT_MS = 5000;   // IMU 校准超时
constexpr int DRIVE_TIMEOUT_MS = 5000;     // driveDistance 超时
constexpr int TURN_TIMEOUT_MS = 5000;      // turnToAngle 超时

// -------------------------------
// 闭环容忍度
// -------------------------------
constexpr double DRIVE_TOLERANCE_IN = 0.5;   // 直线到位阈值（英寸）
constexpr double TURN_TOLERANCE_DEG = 1.0;   // 转向到位阈值（度）

// -------------------------------
// 功率限幅
// -------------------------------
constexpr double DRIVE_POWER_CAP = 100.0;    // 直线最大功率
constexpr double TURN_POWER_CAP = 90.0;      // 转向最大功率

// -------------------------------
// 直线保持增益（IMU 纠偏）
// -------------------------------
constexpr double IMU_HOLD_GAIN = 1.2;

#endif // CONFIG_HPP
