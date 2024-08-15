#include <Servo.h>

Servo base;
Servo limb1;
Servo limb2;
Servo clawy;
Servo grip;

const int SERVO_COUNT = 5;

struct Node {
    byte angles[SERVO_COUNT];
    Node* parent;
};

Node rrtTree[100];
int treeSize = 0;

void setup() {
    Serial.begin(9600);

    base.attach(3);
    limb1.attach(5);
    limb2.attach(6);
    clawy.attach(9);
    grip.attach(10);

    Node startNode = { {94, 120, 60, 15, 80}, NULL };
    rrtTree[treeSize++] = startNode;
    setServoPositions(startNode.angles);
}

void loop() {
    Node goalNode = { {180, 120, 40, 40, 25}, NULL };
    tracePath(rrtTree[0]); // Simplified to use startNode
    while (1);
}

void tracePath(Node node) {
    Serial.println("Starting Path Trace");
    while (node.parent != NULL) {
        printNode(node); // Print the current node's angles
        setServoPositions(node.angles);
        delay(1000); // Adjust delay if needed
        node = *node.parent;
    }
    printNode(node); // Final position
    Serial.println("Path Trace Completed");
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
    grip.write(angles[4]);
}
