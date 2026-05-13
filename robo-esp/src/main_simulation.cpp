#ifdef SIMULATION_MODE

#include <iostream>
#include <vector>
#include "navigation/GridMap.h"
#include "navigation/PathPlanner.h"

namespace {

const navigation::Coordinate kFirePosition = {3, 3};
const navigation::Coordinate kDynamicObstacle = {2, 0};

void printCoordinate(const navigation::Coordinate& coordinate) {
    std::cout << "(" << static_cast<int>(coordinate.x)
              << "," << static_cast<int>(coordinate.y) << ")";
}

void printPath(const std::vector<navigation::Coordinate>& path) {
    std::cout << "Caminho calculado: ";

    for (size_t i = 0; i < path.size(); ++i) {
        printCoordinate(path[i]);

        if (i + 1 < path.size()) {
            std::cout << " -> ";
        }
    }

    std::cout << "\n";
}

void printGrid(const navigation::GridMap& gridMap, const navigation::Coordinate& robotPosition) {
    std::cout << "\nGrid 4x4\n";

    for (uint8_t y = 0; y < navigation::GridMap::Height; ++y) {
        for (uint8_t x = 0; x < navigation::GridMap::Width; ++x) {
            char cell = '.';

            if (robotPosition.x == x && robotPosition.y == y) {
                cell = 'R';
            } else if (navigation::GridMap::Home.x == x && navigation::GridMap::Home.y == y) {
                cell = 'H';
            } else if (gridMap.isFixedBlocked(x, y)) {
                cell = 'X';
            } else if (gridMap.hasDynamicObstacle(x, y)) {
                cell = 'O';
            } else if (kFirePosition.x == x && kFirePosition.y == y) {
                cell = 'F';
            }

            std::cout << cell << ' ';
        }

        std::cout << "\n";
    }

    std::cout << "Posicao atual: ";
    printCoordinate(robotPosition);
    std::cout << "\n";
}

bool calculateRoute(
    navigation::PathPlanner& pathPlanner,
    navigation::Coordinate start,
    navigation::Coordinate goal
) {
    std::cout << "Calculando rota de ";
    printCoordinate(start);
    std::cout << " ate ";
    printCoordinate(goal);
    std::cout << "...\n";

    if (!pathPlanner.findPath(start, goal)) {
        std::cout << "Nao foi possivel calcular uma rota.\n";
        return false;
    }

    printPath(pathPlanner.currentPath());
    return true;
}

void simulateMovement(
    const navigation::GridMap& gridMap,
    const std::vector<navigation::Coordinate>& path,
    navigation::Coordinate& robotPosition
) {
    if (path.empty()) {
        return;
    }

    printGrid(gridMap, robotPosition);

    for (size_t i = 1; i < path.size(); ++i) {
        robotPosition = path[i];

        std::cout << "\nMovendo para ";
        printCoordinate(robotPosition);
        std::cout << "\n";

        printGrid(gridMap, robotPosition);
    }
}

} // namespace

int main() {
    navigation::GridMap gridMap;
    navigation::PathPlanner pathPlanner;

    gridMap.begin();
    gridMap.setDynamicObstacle(kDynamicObstacle.x, kDynamicObstacle.y, true);
    pathPlanner.begin();
    pathPlanner.setGridMap(gridMap);

    navigation::Coordinate robotPosition = {
        navigation::GridMap::Home.x,
        navigation::GridMap::Home.y
    };

    std::cout << "Simulacao do robo autonomo\n";
    std::cout << "Legenda: H=HOME R=ROBO X=BLOQUEADO O=OBSTACULO DINAMICO F=FOGO .=LIVRE\n";
    std::cout << "Incendio simulado em ";
    printCoordinate(kFirePosition);
    std::cout << "\n";
    std::cout << "Obstaculo dinamico simulado em ";
    printCoordinate(kDynamicObstacle);
    std::cout << "\n";

    if (!calculateRoute(pathPlanner, robotPosition, kFirePosition)) {
        return 1;
    }

    simulateMovement(gridMap, pathPlanner.currentPath(), robotPosition);

    std::cout << "\nRobo chegou ao incendio em ";
    printCoordinate(robotPosition);
    std::cout << "\n";

    return 0;
}

#endif // SIMULATION_MODE
