// Example only. Copy this file to wifi_secrets.h in the same folder and edit it.
// wifi_secrets.h is listed in .gitignore: it is never committed. The firmware
// refuses to compile with the example password below or a password shorter
// than 8 characters.
#pragma once

// The robot's own Wi-Fi access point (SoftAP). Make the name unique per robot.
constexpr char ROBOT_SSID[] = "RobotESP32-SLAM-01";
// WPA2 password, 8 to 63 characters.
constexpr char ROBOT_PASSWORD[] = "change-this-password";
