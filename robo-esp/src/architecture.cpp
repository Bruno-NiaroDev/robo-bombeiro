#include "actuators/PumpController.h"
#include "config/ConfigManager.h"
#include "core/SystemManager.h"
#include "core/TaskScheduler.h"
#include "drivers/MotorDriver.h"
#include "drivers/SensorDriver.h"
#include "navigation/GridMap.h"
#include "navigation/PathPlanner.h"
#include "navigation/NavigationManager.h"
#include "network/NetworkManager.h"
#include "network/WebSocketClient.h"
#include "network/WebSocketManager.h"
#include "robot/Robot.h"
#include "robot/AutonomousRobotSystem.h"
#include "robot/RobotState.h"
#include "sensors/SensorBase.h"
#include "sensors/FireScanner.h"
#include "sensors/SensorManager.h"
#include "sensors/UltrasonicSensor.h"
#include "movement/MotionController.h"
#include "movement/MovementController.h"
#include "movement/TrajectoryPlanner.h"

namespace core {

void TaskScheduler::begin() {}
void TaskScheduler::update(unsigned long currentMillis) {}
void TaskScheduler::registerTask(void (*task)(unsigned long)) {}

void SystemManager::begin() {}
void SystemManager::update(unsigned long currentMillis) {}

} // namespace core

namespace drivers {

void SensorDriver::begin() {}
void SensorDriver::read() {}
bool SensorDriver::hasNewData() const { return false; }
float SensorDriver::value() const { return 0.0f; }
void SensorDriver::update(unsigned long currentMillis) {}

} // namespace drivers

namespace navigation {

} // namespace navigation

namespace network {

void WebSocketManager::begin(uint16_t port) {}
void WebSocketManager::sendText(const char* message) {}
void WebSocketManager::update(unsigned long currentMillis) {}
bool WebSocketManager::isClientConnected() const { return false; }

} // namespace network

namespace movement {

void TrajectoryPlanner::begin() {}
void TrajectoryPlanner::buildTrajectory(uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY) {}
const std::vector<TrajectoryPoint>& TrajectoryPlanner::trajectory() const { return _trajectory; }
void TrajectoryPlanner::update(unsigned long currentMillis) {}

void MotionController::begin() {}
void MotionController::update(unsigned long currentMillis) {}
void MotionController::moveTo(uint8_t x, uint8_t y) {}
bool MotionController::isMoving() const { return false; }

} // namespace movement
