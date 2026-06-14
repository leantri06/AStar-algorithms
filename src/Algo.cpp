#include "Algo.h"
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
struct NodeInfo {
    int id;
    double g;
    double h;
    double f;
    int parent;
};

/*-------------------------------- TASK 1 --------------------------------*/
struct QueueElement {
    int id;
    int dist;
};

double calculateHeuristic(int current, int goal, double adjMatrix[100][100]) {
    if (current == goal) return 0.0;

    bool visited[100] = {false};
    
    QueueElement q[100];
    int head = 0;
    int tail = 0;

    q[tail].id = current;
    q[tail].dist = 0;
    tail++;
    visited[current] = true;

    while (head < tail) {
        int u = q[head].id;
        int dist = q[head].dist;
        head++;

        if (u == goal) {
            return (double)dist;
        }

        for (int v = 0; v < 100; ++v) {
            if (adjMatrix[u][v] > 0 && !visited[v]) {
                visited[v] = true;
                q[tail].id = v;
                q[tail].dist = dist + 1;
                tail++;
            }
        }
    }
    
    return 1e9;
}

PathNode* findSocialPath(double adjMatrix[100][100], int startPerson, int goalPerson) {
    const int MAX_NODES = 100;
    
    bool closedSet[MAX_NODES] = { false };
    
    NodeInfo nodes[MAX_NODES];
    for (int i = 0; i < MAX_NODES; ++i) {
        nodes[i].id = i;
        nodes[i].g = 1e9;
        nodes[i].h = 0;
        nodes[i].f = 1e9;
        nodes[i].parent = -1;
    }

    std::vector<int> openSet;
    nodes[startPerson].g = 0.0;
    nodes[startPerson].h = calculateHeuristic(startPerson, goalPerson, adjMatrix);
    nodes[startPerson].f = nodes[startPerson].g + nodes[startPerson].h;
    openSet.push_back(startPerson);

    bool found = false;

    while (!openSet.empty()) {
        auto minIt = openSet.begin();
        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
            
            if (nodes[*it].f < nodes[*minIt].f - 1e-6) {
                minIt = it;
            } 
            else if (std::abs(nodes[*it].f - nodes[*minIt].f) <= 1e-6) {
                if (nodes[*it].h < nodes[*minIt].h - 1e-6) {
                    minIt = it;
                }
                else if (std::abs(nodes[*it].h - nodes[*minIt].h) <= 1e-6) {
                    if (*it < *minIt) {
                        minIt = it;
                    }
                }
            }
        }

        int current = *minIt;
        openSet.erase(minIt);

        if (current == goalPerson) {
            found = true;
            break;
        }

        closedSet[current] = true;

        for (int neighbor = 0; neighbor < MAX_NODES; ++neighbor) {
            if (adjMatrix[current][neighbor] <= 0 || closedSet[neighbor]) {
                continue;
            }

            double tentative_g = nodes[current].g + adjMatrix[current][neighbor];

            if (tentative_g < nodes[neighbor].g) {
                nodes[neighbor].parent = current;
                nodes[neighbor].g = tentative_g;
                nodes[neighbor].h = calculateHeuristic(neighbor, goalPerson, adjMatrix);
                nodes[neighbor].f = nodes[neighbor].g + nodes[neighbor].h;

                if (std::find(openSet.begin(), openSet.end(), neighbor) == openSet.end()) {
                    openSet.push_back(neighbor);
                }
            }
        }
    }

    if (!found) return nullptr;

    std::vector<int> reversePath;
    int curr = goalPerson;
    while (curr != -1) {
        reversePath.push_back(curr);
        curr = nodes[curr].parent;
    }
    std::reverse(reversePath.begin(), reversePath.end());

    PathNode* head = nullptr;
    PathNode* tail = nullptr;

    for (int nodeID : reversePath) {
        PathNode* newNode = new PathNode();
        newNode->name = std::to_string(nodeID);
        newNode->g = nodes[nodeID].g;
        newNode->h = nodes[nodeID].h;
        newNode->f = nodes[nodeID].f;
        newNode->next = nullptr;

        if (head == nullptr) {
            head = newNode;
            tail = newNode;
        } else {
            tail->next = newNode;
            tail = newNode;
        }
    }

    return head;
}

/*-------------------------------- TASK 2 --------------------------------*/
double calculateDroneHeuristic(int current, int goal, int coords[100][2], int mode) {
    double x1 = coords[current][0];
    double y1 = coords[current][1];
    double x2 = coords[goal][0];
    double y2 = coords[goal][1];

    if (mode == 1) {
        return std::abs(x1 - x2) + std::abs(y1 - y2);
    } else if (mode == 2) {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    } else if (mode == 3) {
        return std::max(std::abs(x1 - x2), std::abs(y1 - y2));
    }
    return 0.0;
}

PathNode* findDronePath(double weightMatrix[100][100], int coords[100][2], int startPoint, int goalPoint, int mode) {
    const int MAX_NODES = 100;
    bool closedSet[MAX_NODES] = { false };
    
    NodeInfo nodes[MAX_NODES];
    for (int i = 0; i < MAX_NODES; ++i) {
        nodes[i].id = i;
        nodes[i].g = 1e9;
        nodes[i].h = 0;
        nodes[i].f = 1e9;
        nodes[i].parent = -1;
    }

    std::vector<int> openSet;
    nodes[startPoint].g = 0.0;
    nodes[startPoint].h = calculateDroneHeuristic(startPoint, goalPoint, coords, mode);
    nodes[startPoint].f = nodes[startPoint].g + nodes[startPoint].h;
    openSet.push_back(startPoint);

    bool found = false;

    while (!openSet.empty()) {
        auto minIt = openSet.begin();
        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
            
            if (nodes[*it].f < nodes[*minIt].f - 1e-6) {
                minIt = it;
            } 
            else if (std::abs(nodes[*it].f - nodes[*minIt].f) <= 1e-6) {
                if (nodes[*it].h < nodes[*minIt].h - 1e-6) {
                    minIt = it;
                }
                else if (std::abs(nodes[*it].h - nodes[*minIt].h) <= 1e-6) {
                    if (*it < *minIt) {
                        minIt = it;
                    }
                }
            }
        }

        int current = *minIt;
        openSet.erase(minIt);

        if (current == goalPoint) {
            found = true;
            break;
        }

        closedSet[current] = true;

        for (int neighbor = 0; neighbor < MAX_NODES; ++neighbor) {
            if (weightMatrix[current][neighbor] <= 0 || closedSet[neighbor]) {
                continue;
            }

            double tentative_g = nodes[current].g + weightMatrix[current][neighbor];

            if (tentative_g < nodes[neighbor].g) {
                nodes[neighbor].parent = current;
                nodes[neighbor].g = tentative_g;
                nodes[neighbor].h = calculateDroneHeuristic(neighbor, goalPoint, coords, mode);
                nodes[neighbor].f = nodes[neighbor].g + nodes[neighbor].h;

                if (std::find(openSet.begin(), openSet.end(), neighbor) == openSet.end()) {
                    openSet.push_back(neighbor);
                }
            }
        }
    }

    if (!found) return nullptr;

    std::vector<int> reversePath;
    int curr = goalPoint;
    while (curr != -1) {
        reversePath.push_back(curr);
        curr = nodes[curr].parent;
    }
    std::reverse(reversePath.begin(), reversePath.end());

    PathNode* head = nullptr;
    PathNode* tail = nullptr;

    for (int nodeID : reversePath) {
        PathNode* newNode = new PathNode();
        newNode->name = "(" + std::to_string(coords[nodeID][0]) + "," + std::to_string(coords[nodeID][1]) + ")";
        newNode->g = nodes[nodeID].g;
        newNode->h = nodes[nodeID].h;
        newNode->f = nodes[nodeID].f;
        newNode->next = nullptr;

        if (head == nullptr) {
            head = newNode;
            tail = newNode;
        } else {
            tail->next = newNode;
            tail = newNode;
        }
    }

    return head;
}

/*-------------------------------- TASK 3 --------------------------------*/
struct GridNodeInfo {
    double g;
    double h;
    double f;
    int parentX;
    int parentY;
    std::string moveName;
};

double calculateWarehouseHeuristic(int currentX, int currentY, int goalX, int goalY, int mode) {
    if (mode == 1) {
        return std::abs(currentX - goalX) + std::abs(currentY - goalY);
    } else if (mode == 2) {
        return std::max(std::abs(currentX - goalX), std::abs(currentY - goalY));
    }
    return 0.0;
}

PathNode* findWarehousePath(int warehouse[100][100], int m, int n, int startX, int startY, int goalX, int goalY, int mode) {
    const int MAX_SIZE = 100;
    GridNodeInfo nodes[MAX_SIZE][MAX_SIZE];
    bool closedSet[MAX_SIZE][MAX_SIZE] = { false };
    bool inOpenSet[MAX_SIZE][MAX_SIZE] = { false };

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            nodes[i][j].g = 1e9;
            nodes[i][j].h = 0;
            nodes[i][j].f = 1e9;
            nodes[i][j].parentX = -1;
            nodes[i][j].parentY = -1;
            nodes[i][j].moveName = "";
        }
    }

    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    double moveCost[] = {1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5};
    std::string dirNames[] = {"Up", "Down", "Left", "Right", "Up-Left", "Up-Right", "Down-Left", "Down-Right"};

    std::vector<std::pair<int, int>> openSet;
    
    nodes[startX][startY].g = 0.0;
    nodes[startX][startY].h = calculateWarehouseHeuristic(startX, startY, goalX, goalY, mode);
    nodes[startX][startY].f = nodes[startX][startY].g + nodes[startX][startY].h;
    nodes[startX][startY].moveName = "Start";
    
    openSet.push_back({startX, startY});
    inOpenSet[startX][startY] = true;

    bool found = false;

    while (!openSet.empty()) {
        auto minIt = openSet.begin();
        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
            auto& minNode = nodes[minIt->first][minIt->second];
            auto& currNode = nodes[it->first][it->second];

            if (currNode.f < minNode.f) {
                minIt = it;
            } 
            else if (currNode.f == minNode.f) {
                if (currNode.h < minNode.h) {
                    minIt = it;
                }
                else if (currNode.h == minNode.h) {
                    if (it->first < minIt->first) {
                        minIt = it;
                    } else if (it->first == minIt->first && it->second < minIt->second) {
                        minIt = it;
                    }
                }
            }
        }

        int cx = minIt->first;
        int cy = minIt->second;
        openSet.erase(minIt);
        inOpenSet[cx][cy] = false;

        if (cx == goalX && cy == goalY) {
            found = true;
            break;
        }

        closedSet[cx][cy] = true;

        for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx >= 0 && nx < m && ny >= 0 && ny < n && warehouse[nx][ny] == 0 && !closedSet[nx][ny]) {
                double tentative_g = nodes[cx][cy].g + moveCost[i];

                if (tentative_g < nodes[nx][ny].g) {
                    nodes[nx][ny].parentX = cx;
                    nodes[nx][ny].parentY = cy;
                    nodes[nx][ny].moveName = dirNames[i];
                    nodes[nx][ny].g = tentative_g;
                    nodes[nx][ny].h = calculateWarehouseHeuristic(nx, ny, goalX, goalY, mode);
                    nodes[nx][ny].f = nodes[nx][ny].g + nodes[nx][ny].h;

                    if (!inOpenSet[nx][ny]) {
                        openSet.push_back({nx, ny});
                        inOpenSet[nx][ny] = true;
                    }
                }
            }
        }
    }

    if (!found) return nullptr;

    std::vector<std::pair<int, int>> reversePath;
    int currX = goalX, currY = goalY;
    while (currX != -1 && currY != -1) {
        reversePath.push_back({currX, currY});
        int px = nodes[currX][currY].parentX;
        int py = nodes[currX][currY].parentY;
        currX = px;
        currY = py;
    }
    std::reverse(reversePath.begin(), reversePath.end());

    PathNode* head = nullptr;
    PathNode* tail = nullptr;

    for (size_t i = 1; i < reversePath.size(); ++i) {
        int r = reversePath[i].first;
        int c = reversePath[i].second;
        
        PathNode* newNode = new PathNode();
        newNode->name = nodes[r][c].moveName;
        newNode->g = nodes[r][c].g;
        newNode->h = nodes[r][c].h;
        newNode->f = nodes[r][c].f;
        newNode->next = nullptr;

        if (head == nullptr) {
            head = newNode;
            tail = newNode;
        } else {
            tail->next = newNode;
            tail = newNode;
        }
    }

    return head;
}

/*-------------------------------- TASK 4 --------------------------------*/
struct EvacNodeInfo {
    int id;
    double g;
    double h;
    double f;
    int parent;
};

double calculateEvacuationHeuristic(int currentID, int exitID, int n, int mode) {
    int currX = currentID / n;
    int currY = currentID % n;
    int exitX = exitID / n;
    int exitY = exitID % n;

    if (mode == 1) {
        return std::abs(currX - exitX) + std::abs(currY - exitY);
    } else if (mode == 2) {
        return std::max(std::abs(currX - exitX), std::abs(currY - exitY));
    }
    return 0.0;
}

PathNode* findEvacuationPath(int floorPlan[100][100], int m, int n, int startX, int startY, int exitX, int exitY, double weightMatrix[100][100], int mode) {
    int totalVertices = m * n;
    if (totalVertices > 100) return nullptr; 

    for (int i = 0; i < totalVertices; ++i) {
        for (int j = 0; j < totalVertices; ++j) {
            weightMatrix[i][j] = 0.0; 
        }
    }

    int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};
    double cost[] = {1.0, 1.0, 1.0, 1.0, 1.5, 1.5, 1.5, 1.5};

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            if (floorPlan[i][j] != 0) continue;

            int u = i * n + j;
            
            for (int d = 0; d < 8; ++d) {
                int ni = i + dx[d];
                int nj = j + dy[d];

                if (ni >= 0 && ni < m && nj >= 0 && nj < n && floorPlan[ni][nj] == 0) {
                    int v = ni * n + nj;
                    weightMatrix[u][v] = cost[d];
                }
            }
        }
    }

    int startNode = startX * n + startY;
    int exitNode = exitX * n + exitY;

    EvacNodeInfo nodes[100];
    bool closedSet[100] = { false };
    bool inOpenSet[100] = { false };

    for (int i = 0; i < totalVertices; ++i) {
        nodes[i].id = i;
        nodes[i].g = 1e9;
        nodes[i].h = 0;
        nodes[i].f = 1e9;
        nodes[i].parent = -1;
    }

    std::vector<int> openSet;
    nodes[startNode].g = 0.0;
    nodes[startNode].h = calculateEvacuationHeuristic(startNode, exitNode, n, mode);
    nodes[startNode].f = nodes[startNode].g + nodes[startNode].h;
    
    openSet.push_back(startNode);
    inOpenSet[startNode] = true;

    bool found = false;

    while (!openSet.empty()) {
        auto minIt = openSet.begin();
        for (auto it = openSet.begin(); it != openSet.end(); ++it) {
            if (nodes[*it].f < nodes[*minIt].f) {
                minIt = it;
            } else if (nodes[*it].f == nodes[*minIt].f) {
                if (nodes[*it].h < nodes[*minIt].h) {
                    minIt = it;
                }
            }
        }

        int curr = *minIt;
        openSet.erase(minIt);
        inOpenSet[curr] = false;

        if (curr == exitNode) {
            found = true;
            break;
        }

        closedSet[curr] = true;

        for (int neighbor = 0; neighbor < totalVertices; ++neighbor) {
            if (weightMatrix[curr][neighbor] > 0 && !closedSet[neighbor]) {
                double tentative_g = nodes[curr].g + weightMatrix[curr][neighbor];

                if (tentative_g < nodes[neighbor].g) {
                    nodes[neighbor].parent = curr;
                    nodes[neighbor].g = tentative_g;
                    nodes[neighbor].h = calculateEvacuationHeuristic(neighbor, exitNode, n, mode);
                    nodes[neighbor].f = nodes[neighbor].g + nodes[neighbor].h;

                    if (!inOpenSet[neighbor]) {
                        openSet.push_back(neighbor);
                        inOpenSet[neighbor] = true;
                    }
                }
            }
        }
    }

    if (!found) return nullptr;

    std::vector<int> reversePath;
    int currNode = exitNode;
    while (currNode != -1) {
        reversePath.push_back(currNode);
        currNode = nodes[currNode].parent;
    }
    std::reverse(reversePath.begin(), reversePath.end());

    PathNode* head = nullptr;
    PathNode* tail = nullptr;

    for (int p : reversePath) {
        PathNode* newNode = new PathNode();
        newNode->name = "(" + std::to_string(p / n) + ", " + std::to_string(p % n) + ")"; 
        newNode->g = nodes[p].g;
        newNode->h = nodes[p].h;
        newNode->f = nodes[p].f;
        newNode->next = nullptr;

        if (head == nullptr) {
            head = newNode;
            tail = newNode;
        } else {
            tail->next = newNode;
            tail = newNode;
        }
    }

    return head;
}