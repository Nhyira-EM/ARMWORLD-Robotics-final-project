// RRT PATH PLANNING FOR MANIPULATION

#include <Servo.h>

Servo base;  // create Servo object to control the base servo
Servo limb1;  // create Servo object to control limb1
Servo limb2;  // create Servo object to control limb2
//Servo clawx;  // create Servo object to control clawx
Servo clawy;  // create Servo object to control clawy
Servo grip;  // create Servo object to control grip

const int MAX_ANGLE = 180;
const int STEP_SIZE = 20;
const int MAX_NODES = 50;  // Further reduced
const int SERVO_COUNT = 5;

// Define the Node structure
struct Node {
    byte angles[SERVO_COUNT];
    Node* parent;
};

// RRT tree and size
Node rrtTree[MAX_NODES];
int treeSize = 1;

// Global goalNode
Node goalNode = { {180, 120, 40, 40, 25}, NULL };

void setup() {
    base.attach(3);  // Pin for base servo
    limb1.attach(5); // Pin for limb1 servo
    limb2.attach(6); // Pin for limb2 servo
    //clawx.attach(9); // Pin for clawx servo
    clawy.attach(9); // Pin for clawy servo
    grip.attach(10); // Pin for grip servo

    Serial.begin(9600);
    Serial.println("Serial Communication Started");

    // Initialize RRT tree with start node
    Node startNode = { {94, 120, 60, 15, 80}, NULL };
    rrtTree[treeSize++] = startNode;
    setServoPositions(startNode.angles);
}

void loop() {
    if (treeSize == 0) {
        Serial.println("Tree size is zero. RRT might not be initialized correctly.");
        return;
    }

    Serial.println("Loop started");
    
    // RRT Path Planning
    Node randomNode;
    generateRandomNode(randomNode);
    Node nearestNode = findNearestNode(randomNode);
    Node newNode = extendTree(nearestNode, randomNode);
    
    if (!checkCollision(newNode)) {
        rrtTree[treeSize++] = newNode;
        if (reachedGoal(newNode, goalNode)) {
            tracePath(newNode);
            while (1);  // Stop further execution
        }
    }
    delay(100);
}

Node findNearestNode(Node randomNode) {
    Node nearestNode;
    float minDist = 1e9;  // Large initial value

    for (int i = 0; i < treeSize; i++) {
        float dist = 0;
        for (int j = 0; j < SERVO_COUNT; j++) {
            dist += sq(rrtTree[i].angles[j] - randomNode.angles[j]);
        }
        if (dist < minDist) {
            minDist = dist;
            nearestNode = rrtTree[i];
        }
    }
    return nearestNode;
}

void generateRandomNode(Node &node) {
    if (random(0, 100) < 30) {  // 30% chance to pick goal-biased node
        for (int i = 0; i < SERVO_COUNT; i++) {
            node.angles[i] = goalNode.angles[i];
        }
    } else {
        for (int i = 0; i < SERVO_COUNT; i++) {
            node.angles[i] = random(0, MAX_ANGLE + 1);
        }
    }
    
    // Print Random Node Angles
    for (int i = 0; i < SERVO_COUNT; i++) {
        Serial.print("Random Node Angle ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(node.angles[i]);
    }
}

Node extendTree(Node nearestNode, Node randomNode) {
    Node newNode;
    for (int i = 0; i < SERVO_COUNT; i++) {
        float direction = randomNode.angles[i] - nearestNode.angles[i];
        float proportion = min(1.0, sqrt(pow(direction, 2)) / MAX_ANGLE);  // Proportional control
        newNode.angles[i] = nearestNode.angles[i] + constrain(direction * proportion, -STEP_SIZE, STEP_SIZE);
    }
    newNode.parent = &nearestNode;
    return newNode;
}

bool checkCollision(Node node) {
    // Simplified collision check
    // You can add a specific check for your environment here
    return false;
}

bool reachedGoal(Node node, Node goalNode) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        Serial.print("Checking Angle ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(abs(node.angles[i] - goalNode.angles[i]));
        Serial.print(" > ");
        Serial.println(STEP_SIZE);
        if (abs(node.angles[i] - goalNode.angles[i]) > STEP_SIZE) {
            return false;
        }
    }
    return true;
}

void tracePath(Node node) {
    Serial.println("Tracing Path:");
    while (node.parent != NULL) {
        printNode(node); // Print the current node's angles
        setServoPositions(node.angles);
        delay(1000); // Adjust delay if needed
        node = *node.parent;
    }
    printNode(node); // Final position
}

void printNode(Node node) {
    Serial.print("Angles: ");
    for (int i = 0; i < SERVO_COUNT; i++) {
        Serial.print(node.angles[i]);
        if (i < SERVO_COUNT - 1) {
            Serial.print(", ");
        }
    }
    Serial.println();
}

void setServoPositions(byte angles[SERVO_COUNT]) {
    base.write(angles[0]);
    limb1.write(angles[1]);
    limb2.write(angles[2]);
    clawy.write(angles[3]);
    // grip.write(angles[4]);
}
