#pragma once

#include <vector>
#include "../navigation/NavigationManager.h"
#include "RobotState.h"

namespace robot
{

    class Robot
    {
    public:
        void begin();
        void update(unsigned long currentMillis);

        bool setTarget(uint8_t x, uint8_t y, unsigned long currentMillis);
        void setPosition(uint8_t x, uint8_t y);
        void setHeading(uint8_t heading);
        void notifyCellReached(uint8_t x, uint8_t y);
        void notifyMovementFailed(const char *message);
        void notifyObstacleDetected(uint8_t x, uint8_t y);
        void clearObstacleDetection();
        void notifyFireDetected(bool detected);
        void notifyFireConfirmed(unsigned long currentMillis);
        void notifyFireSearchTimedOut(unsigned long currentMillis);
        void notifyExtinguishingComplete();
        void resetError();

        const RobotState &state() const;
        void setMode(RobotMode mode);
        AutonomousState autonomousState() const;
        bool hasNextWaypoint() const;
        navigation::Coordinate nextWaypoint() const;
        const std::vector<navigation::Coordinate> &currentPath() const;
        const char *lastError() const;

    private:
        void transitionTo(AutonomousState nextState, unsigned long currentMillis);
        void handleReceivingTarget(unsigned long currentMillis);
        void handleCalculatingRoute(unsigned long currentMillis);
        void handleMoving(unsigned long currentMillis);
        void handleAvoidingObstacle(unsigned long currentMillis);
        void handleSearchingFire(unsigned long currentMillis);
        void handleExtinguishingFire(unsigned long currentMillis);
        void handleReturningHome(unsigned long currentMillis);
        void enterError(const char *message);
        bool calculateRouteTo(uint8_t x, uint8_t y);
        bool planReverseReturnHome(unsigned long currentMillis);
        bool reachedTarget() const;
        bool reachedHome() const;
        void prepareReverseReturnRoute();
        void syncPositionFromRoute();
        void setLastError(const char *message);
        static const char *stateName(AutonomousState state);

        RobotState _state;
        navigation::NavigationManager _navigation;
        uint8_t _pathIndex = 0;
        uint8_t _obstacleX = 0;
        uint8_t _obstacleY = 0;
        bool _hasPendingObstacle = false;
        bool _extinguishingComplete = false;
        bool _routeRecalculationPending = false;
        unsigned long _stateStartedAt = 0;
    };

} // namespace robot
