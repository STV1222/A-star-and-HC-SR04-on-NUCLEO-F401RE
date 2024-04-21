#include "mbed.h"
#include "hcsr04.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

//D12 TRIGGER D11 ECHO
HCSR04 sensor(PC_10, PC_11); 
InterruptIn button(D4);


#define MAX_ROWS 40
#define MAX_COLS 40
#define TOTAL_NODES (MAX_ROWS * MAX_COLS)
#define LARGE_VALUE 9999

int openList[TOTAL_NODES] = {0};
int closedList[TOTAL_NODES] = {0};
double g[TOTAL_NODES];
double h[TOTAL_NODES];
double f[TOTAL_NODES];
int parents[TOTAL_NODES];

#define MAX_OBSTACLES 20

Serial pc(USBTX, NC);

typedef enum {
    NORTH, SOUTH, EAST, WEST, NORTH_EAST, NORTH_WEST, SOUTH_EAST, SOUTH_WEST, UNKNOWN
} Direction;

typedef struct {
    int x1, y1; // Top-left corner
    int x2, y2; // Bottom-right corner
} Obstacle;

Obstacle obstacles[MAX_OBSTACLES];
int numObstacles = 0;

// Example initialization (could be done in a setup function)
void initObstacles() { //initialize known obstacles
    obstacles[0] = (Obstacle){.x1 = 5, .y1 = 5, .x2 = 11, .y2 = 7}; // First obstacle
    obstacles[1] = (Obstacle){.x1 = 12, .y1 = 15, .x2 = 15, .y2 = 25}; // Second obstacle
    obstacles[2] = (Obstacle){.x1 = 16, .y1 = 8, .x2 = 17, .y2 = 20}; // Second obstacle
    obstacles[3] = (Obstacle){.x1 = 17, .y1 = 1, .x2 = 23, .y2 = 20}; // Second obstacle
    obstacles[4] = (Obstacle){.x1 = 7, .y1 = 30, .x2 = 17, .y2 = 32}; // Second obstacle
    obstacles[5] = (Obstacle){.x1 = 23, .y1 = 27, .x2 = 25, .y2 = 40}; // Second obstacle
    obstacles[6] = (Obstacle){.x1 = 30, .y1 = 20, .x2 = 34, .y2 = 35}; // Second obstacle
    numObstacles = 7; // Update this as you add more obstacles
}

void expandObstacles() { //considering the size of the robot to be a maximum radius of 10 cm 
    for (int i = 0; i < numObstacles; i++) {
        // Expand each obstacle by 2 nodes in each direction
        // Make sure not to exceed the boundaries of your environment
        obstacles[i].x1 = (obstacles[i].x1 - 2 >= 1) ? obstacles[i].x1 - 2 : 1;
        obstacles[i].y1 = (obstacles[i].y1 - 2 >= 1) ? obstacles[i].y1 - 2 : 1;
        obstacles[i].x2 = (obstacles[i].x2 + 2 <= MAX_COLS) ? obstacles[i].x2 + 2 : MAX_COLS;
        obstacles[i].y2 = (obstacles[i].y2 + 2 <= MAX_ROWS) ? obstacles[i].y2 + 2 : MAX_ROWS;
    }
}

void printObstacleDimensions() {
    for (int i = 0; i < numObstacles; i++) {
        printf("\n\rObstacle %d: (%d, %d) to (%d, %d)\n", i+1, 
               obstacles[i].x1, obstacles[i].y1, 
               obstacles[i].x2, obstacles[i].y2);
    }
}

// Custom max and min functions
int customMax(int a, int b) {
    return (a > b) ? a : b;
}

int customMin(int a, int b) {
    return (a < b) ? a : b;
}

void addObstacleAhead(int currentX, int currentY, const char* direction) { // add new obstacle from ultrasonic sensor and expand the obstacle avoidance area
    if (numObstacles >= MAX_OBSTACLES) {
        printf("\rMaximum number of obstacles reached. Cannot add more.\n");
        return;
    }

    int newX = currentX, newY = currentY;

    // Adjust the obstacle position based on the direction
    if (strcmp(direction, "North") == 0) newY += 1;
    else if (strcmp(direction, "South") == 0) newY -= 1;
    else if (strcmp(direction, "East") == 0) newX += 1;
    else if (strcmp(direction, "West") == 0) newX -= 1;
    else if (strcmp(direction, "North-East") == 0) {
        newX += 1;
        newY += 1;
    }
    else if (strcmp(direction, "North-West") == 0) {
        newX -= 1;
        newY += 1;
    }
    else if (strcmp(direction, "South-East") == 0) {
        newX += 1;
        newY -= 1;
    }
    else if (strcmp(direction, "South-West") == 0) {
        newX -= 1;
        newY -= 1;
    }

    obstacles[numObstacles].x1 = newX;
    obstacles[numObstacles].y1 = newY;
    obstacles[numObstacles].x2 = newX;
    obstacles[numObstacles].y2 = newY;

    // Expand obstacle except on the robot's current side
    if (strcmp(direction, "North") == 0) {
        obstacles[numObstacles].y2 += 2;
        obstacles[numObstacles].x1 -= 2;
        obstacles[numObstacles].x2 += 2;
    } 
    else if (strcmp(direction, "South") == 0) {
        obstacles[numObstacles].y1 -= 2;
        obstacles[numObstacles].x1 -= 2;
        obstacles[numObstacles].x2 += 2;
    } 
    else if (strcmp(direction, "East") == 0) {
        obstacles[numObstacles].x2 += 2;
        obstacles[numObstacles].y1 -= 2;
        obstacles[numObstacles].y2 += 2;
    } 
    else if (strcmp(direction, "West") == 0) {
        obstacles[numObstacles].x1 -= 2;
        obstacles[numObstacles].y1 -= 2;
        obstacles[numObstacles].y2 += 2;
    }
    else if (strcmp(direction, "North-East") == 0) {
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY, .x2 = newX, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY-1, .x2 = newX+1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY, .x2 = newX+1, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+2, .y1 = newY, .x2 = newX+2, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY+1, .x2 = newX+1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY+1, .x2 = newX, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY+1, .x2 = newX-1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY+2, .x2 = newX, .y2 = newY+2};
    }
    else if (strcmp(direction, "North-West") == 0) {
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY, .x2 = newX, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY, .x2 = newX-1, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-2, .y1 = newY, .x2 = newX-2, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY-1, .x2 = newX-1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY+1, .x2 = newX-1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY+1, .x2 = newX, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY+1, .x2 = newX+1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY+2, .x2 = newX, .y2 = newY+2};
    }
    else if (strcmp(direction, "South-East") == 0) {
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY, .x2 = newX, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY, .x2 = newX+1, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+2, .y1 = newY, .x2 = newX+2, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY-1, .x2 = newX-1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY-1, .x2 = newX+1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY-1, .x2 = newX, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY+1, .x2 = newX+1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY-2, .x2 = newX, .y2 = newY-2};
    }
    else if (strcmp(direction, "South-West") == 0) {
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY, .x2 = newX, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY, .x2 = newX-1, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-2, .y1 = newY, .x2 = newX-2, .y2 = newY};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY+1, .x2 = newX-1, .y2 = newY+1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX-1, .y1 = newY-1, .x2 = newX-1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY-1, .x2 = newX, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX+1, .y1 = newY-1, .x2 = newX+1, .y2 = newY-1};
        obstacles[numObstacles++] = (Obstacle){.x1 = newX, .y1 = newY-2, .x2 = newX, .y2 = newY-2};
    }
    
    // Make sure the obstacle stays within bounds
    obstacles[numObstacles].x1 = customMax(1, obstacles[numObstacles].x1);
    obstacles[numObstacles].y1 = customMax(1, obstacles[numObstacles].y1);
    obstacles[numObstacles].x2 = customMin(MAX_COLS, obstacles[numObstacles].x2);
    obstacles[numObstacles].y2 = customMin(MAX_ROWS, obstacles[numObstacles].y2);

    numObstacles++;
    printObstacleDimensions();
    // printf("\rNew obstacle added and expanded at position (%d, %d) to (%d, %d).\n",
    //     obstacles[numObstacles - 1].x1, obstacles[numObstacles - 1].y1, 
    //     obstacles[numObstacles - 1].x2, obstacles[numObstacles - 1].y2);
}

volatile bool trigger_flag = false;
volatile int globalCurrentNodeX = 0, globalCurrentNodeY =0; // current running position
volatile int previousNodeX = -1, previousNodeY = -1; // Previous position
char* currentFacingDirection;
int start_x = 40; //set goal positions
int start_y = 40;
int goal_x = 1; //set start positions
int goal_y = 1;

int xyToIndex(int x, int y) { return (y - 1) * MAX_COLS + (x - 1); }
void indexToXY(int index, int *x, int *y) {
    *x = index % MAX_COLS + 1;
    *y = index / MAX_COLS + 1;
}

int goalIdx = xyToIndex(goal_x, goal_y);
int idx = goalIdx;
int parentIdx;

void initializeGrid() {
    for (int i = 0; i < TOTAL_NODES; i++) {
        g[i] = LARGE_VALUE;
        h[i] = LARGE_VALUE;
        f[i] = LARGE_VALUE;
        parents[i] = -1;
        openList[i] = 0;
        closedList[i] = 0;
    }
}

int isValidPosition(int x, int y) {

    if (x < 1 || x > MAX_COLS || y < 1 || y > MAX_ROWS) return 0; // Check bounds first

    for (int i = 0; i < numObstacles; i++) {
        // Check if position is inside the current obstacle
        if (x >= obstacles[i].x1 && x <= obstacles[i].x2 && y >= obstacles[i].y1 && y <= obstacles[i].y2) {
            return 0; // Position is invalid if it's inside any obstacle
        }
    }
    return 1; // Position is valid if it's outside all obstacles
}

double calculateDistance(int x1, int y1, int x2, int y2) {
    return sqrt(pow((double)abs(x1 - x2), 2) + pow((double)abs(y1 - y2), 2));
}

void printPathAndDistance(int goalIdx) {
    int idx = goalIdx;
    double pathDistance = 0;
    int steps = 0;
    int parentIdx;

    printf("\rPath Generated:\n\r");
    while (parents[idx] != -1) {
        parentIdx = parents[idx];
        pathDistance += calculateDistance(idx % MAX_COLS + 1, idx / MAX_COLS + 1, parentIdx % MAX_COLS + 1, parentIdx / MAX_COLS + 1);
        printf("(%d, %d) -> ", idx % MAX_COLS + 1, idx / MAX_COLS + 1);
        idx = parentIdx;
        steps++;
        if (steps % 5 == 0) printf("\n\r"); // Break line for readability
    }
    printf("(%d, %d)\n", idx % MAX_COLS + 1, idx / MAX_COLS + 1); // Print start node
    
    printf("\rTotal path distance: %.2f units\n", pathDistance);
}

void printpath(int startX, int startY, int goalX, int goalY){    // Not using now, but can be use for testing 
    int goalIdx = xyToIndex(goalX, goalY);
    int idx = goalIdx;
    int parentIdx;
    while (parents[idx] != -1) {

        parentIdx = parents[idx];
        printf("\r(%d, %d)\n", idx % MAX_COLS + 1, idx / MAX_COLS + 1);
        wait(1);
        idx = parentIdx;
    }
    printf("\r(%d, %d)\n", idx % MAX_COLS + 1, idx / MAX_COLS + 1); // Print start node
}

void aStarSearch(int startX, int startY, int goalX, int goalY) {
    initializeGrid();
    int startIdx = xyToIndex(startX, startY);
    int goalIdx = xyToIndex(goalX, goalY);
    g[startIdx] = 0;
    h[startIdx] = calculateDistance(startX, startY, goalX, goalY);
    f[startIdx] = h[startIdx];
    openList[startIdx] = 1;

    while (1) {
        int currentIdx = -1;
        double minF = LARGE_VALUE;
        for (int i = 0; i < TOTAL_NODES; i++) {
            if (openList[i] && f[i] < minF) {
                minF = f[i];
                currentIdx = i;
            }
        }

        if (currentIdx == -1) {
            pc.printf("\rPath not found.\n");
            return;
        }

        if (currentIdx == goalIdx) {
            printPathAndDistance(goalIdx);
            return;
        }

        openList[currentIdx] = 0;
        closedList[currentIdx] = 1;

        int currentX, currentY;
        indexToXY(currentIdx, &currentX, &currentY);

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                int newX = currentX + dx, newY = currentY + dy;
                if (!isValidPosition(newX, newY)) continue;

                int neighborIdx = xyToIndex(newX, newY);
                if (closedList[neighborIdx]) continue;

                double tentativeG = g[currentIdx] + calculateDistance(currentX, currentY, newX, newY);
                if (!openList[neighborIdx]) {
                    openList[neighborIdx] = 1;
                } else if (tentativeG >= g[neighborIdx]) {
                    continue;
                }

                parents[neighborIdx] = currentIdx;
                g[neighborIdx] = tentativeG;
                h[neighborIdx] = calculateDistance(newX, newY, goalX, goalY);
                f[neighborIdx] = g[neighborIdx] + h[neighborIdx];
            }
        }
    }
}

Direction getDirection(int fromX, int fromY, int toX, int toY) {
    if (toY > fromY) {
        if (toX > fromX) return NORTH_EAST;
        else if (toX < fromX) return NORTH_WEST;
        else return NORTH;
    }
    else if (toY < fromY) {
        if (toX > fromX) return SOUTH_EAST;
        else if (toX < fromX) return SOUTH_WEST;
        else return SOUTH;
    } 
    else {
        if (toX > fromX) return EAST;
        else if (toX < fromX) return WEST;
    }
    return UNKNOWN;
}

void buttonISR() // The ISR for the InterruptIn button. This function has minimal code for efficiency
{
    trigger_flag=true; // A single flag is set to indicate that the button has been pressed.
     // Use this time to perform new action
    printf("\rCurrent position: (%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY);
    printf("\rPerforming obstacles avoiding...");
    wait(5);
    printf("\rPerforming A star algorithm again...\n");
    wait(5);
    aStarSearch(start_x, start_y, globalCurrentNodeX, globalCurrentNodeY);
    printf("\rA star algorithm finished computing\n");
}

void resetPathfindingData() { // rest the array when obstacle detection arrived, clean it for new path storing 
    // Reset the parents array
    for (int i = 0; i < TOTAL_NODES; i++) {
        parents[i] = -1;
    }

    // Reset the closedList array
    for (int i = 0; i < TOTAL_NODES; i++) {
        closedList[i] = 0;
    }

    // Reset the g, h, and f arrays
    for (int i = 0; i < TOTAL_NODES; i++) {
        g[i] = LARGE_VALUE;
        h[i] = LARGE_VALUE;
        f[i] = LARGE_VALUE;
    }
}

void obstacleDetect(){ // Triggered when HC-SR04 measured distance under 15
    printf("\rObstalce detect\n");
    printf("\rCurrent position: (%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY);
    printf("\rPerforming obstacles avoiding...\n\r"); // turn right 90 degree or turn left 90 degree or take a U-turn sense if there is obstacles
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Direction dir = getDirection(previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
    switch(dir) {
        case NORTH: currentFacingDirection = "North"; break;
        case SOUTH: currentFacingDirection = "South"; break;
        case EAST: currentFacingDirection = "East"; break;
        case WEST: currentFacingDirection = "West"; break;
        case NORTH_EAST: currentFacingDirection = "North-East"; break;
        case NORTH_WEST: currentFacingDirection = "North-West"; break;
        case SOUTH_EAST: currentFacingDirection = "South-East"; break;
        case SOUTH_WEST: currentFacingDirection = "South-West"; break;
        default: currentFacingDirection = "Unknown"; break;
    }
    int testNodeX = globalCurrentNodeX;
    int testNodeY = globalCurrentNodeY;
    addObstacleAhead(globalCurrentNodeX, globalCurrentNodeY, currentFacingDirection); // add the obstacle detect infront to the obstacles array 
    if (strcmp(currentFacingDirection, "North") == 0){
        printf("\rFacing direction before avoiding: North\n\r");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX+1, testNodeY) == 1){
            globalCurrentNodeX+= 1;
            currentFacingDirection = "East";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX-2, testNodeY) == 1){
            globalCurrentNodeX-= 1;
            currentFacingDirection = "West";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "South") == 0){
        printf("\rFacing direction before avoiding: South\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX-1, testNodeY) == 1){
            globalCurrentNodeX-= 1;
            currentFacingDirection = "West";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX+2, testNodeY) == 1){
            globalCurrentNodeX+= 1;
            currentFacingDirection = "East";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "East") == 0){
        printf("\rFacing direction before avoiding: East\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX, testNodeY-1) == 1){
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX, testNodeY+2) == 1){
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX-= 1;
            currentFacingDirection = "West";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "West") == 0){
        printf("\rFacing direction before avoiding: West\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX, testNodeY+1) == 1){
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX, testNodeY-2) == 1){
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX+= 1;
            currentFacingDirection = "East";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "North-East") == 0){
        printf("\rFacing direction before avoiding: North-East\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX+1, testNodeY-1) == 1){
            globalCurrentNodeX+= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-East";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX-2, testNodeY+2) == 1){
            globalCurrentNodeX-= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-West";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX-= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-West";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "North-West") == 0){
        printf("\rFacing direction before avoiding: North-West\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX+1, testNodeY+1) == 1){
            globalCurrentNodeX+= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-East";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX-2, testNodeY-2) == 1){
            globalCurrentNodeX-= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-West";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX+= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-East";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "South-East") == 0){
        printf("\rFacing direction before avoiding: South-East\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX-1, testNodeY-1) == 1){
            globalCurrentNodeX-= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-West";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX+2, testNodeY+2) == 1){
            globalCurrentNodeX+= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-East";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX-= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-West";
            printf("\rMake a U-turn");
        }
    }
    else if(strcmp(currentFacingDirection, "South-West") == 0){
        printf("\rFacing direction before avoiding: South-West\n");
        // check the node on the right is occupied or free 
        if (isValidPosition(testNodeX-1, testNodeY+1) == 1){
            globalCurrentNodeX-= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-West";
            printf("\rTurn right");
        }
        else if(isValidPosition(testNodeX+2, testNodeY-2) == 1){
            globalCurrentNodeX+= 1;
            globalCurrentNodeY-= 1;
            currentFacingDirection = "South-East";
            printf("\rTurn left");
        }
        else {
            globalCurrentNodeX+= 1;
            globalCurrentNodeY+= 1;
            currentFacingDirection = "North-East";
            printf("\rMake a U-turn");
        }
    }
    else {
        printf("Unknown or no direction specified.\n");
    }

    wait(2);
    printf("\n\rPerforming A star algorithm again...\n");
    resetPathfindingData();
    wait(2);
    aStarSearch(start_x, start_y, globalCurrentNodeX, globalCurrentNodeY);

    goalIdx = xyToIndex(globalCurrentNodeX, globalCurrentNodeY);
    idx = goalIdx;

    printf("\rA star algorithm finished computing\n");
    wait(10);
}

void runThePath(){
    button.rise(&buttonISR);
    int lastParentIdx = -1;

while (parents[idx] != -1) {

            parentIdx = parents[idx];
            lastParentIdx = idx;
            
            globalCurrentNodeX = idx % MAX_COLS + 1;
            globalCurrentNodeY = idx / MAX_COLS + 1;
            float distance = sensor.distance(); // measures the distance from HC-SR04
            if (distance < 20){
                obstacleDetect();
                previousNodeX = -1;
                previousNodeY = -1;
                break;
            }

            printf("\r(%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY);
            // Skip direction calculation/printing for the first position

            if (previousNodeX != -1 && previousNodeY != -1) {
                Direction dir = getDirection(previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
                switch(dir) {
                    case NORTH: currentFacingDirection = "North"; break;
                    case SOUTH: currentFacingDirection = "South"; break;
                    case EAST: currentFacingDirection = "East"; break;
                    case WEST: currentFacingDirection = "West"; break;
                    case NORTH_EAST: currentFacingDirection = "North-East"; break;
                    case NORTH_WEST: currentFacingDirection = "North-West"; break;
                    case SOUTH_EAST: currentFacingDirection = "South-East"; break;
                    case SOUTH_WEST: currentFacingDirection = "South-West"; break;
                    default: currentFacingDirection = "Unknown"; break;
                }
                printf("Moving from (%d, %d) to (%d, %d)\n", previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
                printf("Facing direction: %s\n", currentFacingDirection);
            }
            else {
                printf("Starting position: (%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY);
            }
            previousNodeX = globalCurrentNodeX;
            previousNodeY = globalCurrentNodeY;
            wait(1);
            idx = parentIdx;
            if (idx % MAX_COLS + 1 == start_x & idx / MAX_COLS + 1 == start_y){ // if reaches the final position then prints the final position
                globalCurrentNodeX = idx % MAX_COLS + 1;
                globalCurrentNodeY = idx / MAX_COLS + 1;
                Direction dir = getDirection(previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
                char* currentFacingDirection;
                switch(dir) {
                    case NORTH: currentFacingDirection = "North"; break;
                    case SOUTH: currentFacingDirection = "South"; break;
                    case EAST: currentFacingDirection = "East"; break;
                    case WEST: currentFacingDirection = "West"; break;
                    case NORTH_EAST: currentFacingDirection = "North-East"; break;
                    case NORTH_WEST: currentFacingDirection = "North-West"; break;
                    case SOUTH_EAST: currentFacingDirection = "South-East"; break;
                    case SOUTH_WEST: currentFacingDirection = "South-West"; break;
                    default: currentFacingDirection = "Unknown"; break;
                }
                printf("\r(%d, %d)\n", globalCurrentNodeX, globalCurrentNodeY); 
                printf("Moving from (%d, %d) to (%d, %d)\n", previousNodeX, previousNodeY, globalCurrentNodeX, globalCurrentNodeY);
                printf("Facing direction: %s\n", currentFacingDirection);
                printf("\rDone\n");
            }
        }

}

int main() {
    pc.printf("\rStarting\n");
    button.rise(&buttonISR);
    initObstacles();
    expandObstacles(); 
    printObstacleDimensions();
    aStarSearch(start_x, start_y, goal_x, goal_y); // Adjust the start and goal positions as needed
    //int goalIdx = xyToIndex(goal_x, goal_y);
    //int idx = goalIdx;
    //int parentIdx;
    int lastParentIdx = -1;

    while(1) {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            runThePath();
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    }
    return 0;
}




