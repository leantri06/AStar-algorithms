#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include "Algo.h"
#include "PathNode.h"

using namespace std;

void printPath(PathNode* head) {
    cout << "   [Solution Path]:" << endl;
    if (head == nullptr) {
        cout << "   -> Không tìm thấy đường đi (Path == nullptr)!" << endl;
        return;
    }
    PathNode* curr = head;
    int step = 1;
    while (curr != nullptr) {
        cout << "     Step " << step << " | Node: " << left << setw(8) << curr->name 
             << " | f: " << setw(6) << curr->f 
             << " | g: " << setw(6) << curr->g 
             << " | h: " << setw(6) << curr->h << endl;
        curr = curr->next;
        step++;
    }
}

void freePath(PathNode* head) {
    while (head != nullptr) {
        PathNode* temp = head;
        head = head->next;
        delete temp;
    }
}

int main() {
    cout << fixed << setprecision(2);

    // =========================================================================
    // TEST TASK 1: Degrees of Separation (findSocialPath)
    // =========================================================================
    cout << "======================= TEST TASK 1 =======================" << endl;
    cout << "Mo phong mang xa hoi gom 6 nguoi (0 den 5)" << endl;
    double adjMatrix[100][100] = {0.0};
    
    adjMatrix[0][1] = 1.0; adjMatrix[1][0] = 1.0;
    adjMatrix[1][2] = 1.0; adjMatrix[2][1] = 1.0;
    adjMatrix[2][3] = 1.0; adjMatrix[3][2] = 1.0;
    adjMatrix[3][5] = 1.0; adjMatrix[5][3] = 1.0;
    adjMatrix[0][4] = 1.0; adjMatrix[4][0] = 1.0;

    int startPerson = 0;
    int goalPerson = 5;
    
    cout << "-> Tim duong di ngan nhat tu Nguoi " << startPerson << " den Nguoi " << goalPerson << ":" << endl;
    PathNode* path1 = findSocialPath(adjMatrix, startPerson, goalPerson);
    printPath(path1);
    freePath(path1);
    cout << endl;


    // =========================================================================
    // TEST TASK 2: Drone Delivery in 2D Space (findDronePath)
    // =========================================================================
    cout << "======================= TEST TASK 2 =======================" << endl;
    cout << "Mo phong 4 diem giao hang bang Drone voi toa do thuc te" << endl;
    double weightMatrix[100][100] = {0.0};
    int coords[100][2] = {0};
    
    coords[0][0] = 0;  coords[0][1] = 0;
    coords[1][0] = 3;  coords[1][1] = 0;
    coords[2][0] = 3;  coords[2][1] = 4;
    coords[3][0] = 0;  coords[3][1] = 4;
    
    weightMatrix[0][1] = 3.0;  weightMatrix[1][0] = 3.0;
    weightMatrix[1][2] = 4.0;  weightMatrix[2][1] = 4.0;
    weightMatrix[0][3] = 4.0;  weightMatrix[3][0] = 4.0;
    weightMatrix[3][2] = 3.0;  weightMatrix[2][3] = 3.0;

    int startPoint = 0;
    int goalPoint = 2;
    int modeTask2 = 2;
    
    cout << "-> Tim duong bay tu Tram " << startPoint << " den Tram " << goalPoint << ":" << endl;
    PathNode* path2 = findDronePath(weightMatrix, coords, startPoint, goalPoint, modeTask2);
    printPath(path2);
    freePath(path2);
    cout << endl;


    // =========================================================================
    // TEST TASK 3: Warehouse Robot Navigation (findWarehousePath)
    // =========================================================================
    cout << "======================= TEST TASK 3 =======================" << endl;
    cout << "Mo phong luoi kho hang 5x5 voi cac chuong ngai vat (gia ke)" << endl;
    int warehouse[100][100] = {0};
    int m3 = 5, n3 = 5;
    
    warehouse[1][2] = 1;
    warehouse[2][2] = 1;
    warehouse[3][2] = 1; 

    int startX = 0, startY = 0;
    int goalX = 4, goalY = 4;
    int modeTask3 = 1;
    
    cout << "-> Tim duong di cho Robot tu (" << startX << "," << startY 
         << ") den (" << goalX << "," << goalY << "):" << endl;
    PathNode* path3 = findWarehousePath(warehouse, m3, n3, startX, startY, goalX, goalY, modeTask3);
    printPath(path3);
    freePath(path3);
    cout << endl;


    // =========================================================================
    // TEST TASK 4: Evacuation Route Planning
    // =========================================================================
    cout << "======================= TEST TASK 4 =======================" << endl;
    cout << "Mo phong so do thoat hiem 4x4 va tinh toan ma tran trong so" << endl;
    int floorPlan[100][100] = {0};
    double evacWeightMatrix[100][100] = {0.0};
    int m4 = 4, n4 = 4;
    int mode = 1;
    floorPlan[1][1] = 1;
    floorPlan[2][2] = 1;

    int evacStartX = 0, evacStartY = 0;
    int exitX = 3, exitY = 3;
    
    cout << "-> Tim duong thoat hiem tu (" << evacStartX << "," << evacStartY 
         << ") den Cua ra (" << exitX << "," << exitY << "):" << endl;
         
    PathNode* path4 = findEvacuationPath(floorPlan, m4, n4, evacStartX, evacStartY, exitX, exitY, evacWeightMatrix, mode);
    printPath(path4);
    freePath(path4);
    
    cout << "\n-> Kiem tra mot phan Ma tran trong so (Sinh tu Task 4):" << endl;
    int checkNodeId = evacStartX * n4 + evacStartY;
    cout << "   Cac canh ket noi hop le tu o (0,0) sang o ke canh:" << endl;
    for (int j = 0; j < m4 * n4; ++j) {
        if (evacWeightMatrix[checkNodeId][j] > 0) {
            int r = j / n4;
            int c = j % n4;
            cout << "     Tu Node 0 den Node " << j << " (vi tri [" << r << "][" << c << "]) co Weight = " << evacWeightMatrix[checkNodeId][j] << endl;
        }
    }
    cout << "===========================================================" << endl;

    return 0;
}