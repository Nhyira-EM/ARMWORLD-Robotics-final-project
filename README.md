## ARMWORLD-Robotics-final-project
 A 5DOF robotic arm that segregates waste using an YOLOv8 model and an RRT algorithm.

 # Authors
 - Emmanuel Nhyira Freduah-Agyemang(https://github.com/Nhyira-EM)
 - Ako Eyo Oku
 - Francine Arthur(https://github.com/Ci-ne)

# Summary
Ghana generates 840,000 tonnes of plastic waste annually, but only 9.5% is recycled. Overall recycling rates are at a low 10%. This project aims to address waste segregation using a 5DOF Robotic Arm and an image detection model to classify and sort waste into appropriate bins, focusing on metal and plastic.

 This is the culmination of our efforts for a final project for an Introduction to AI Robotics class.
 
 The project involves building a robotic arm that can segregate waste into different categories using a YOLO model for waste detection adn an RRT algorithm for path planning.
 
 The project leverages object detection and classification, and path planning for robotic arms. We fine-tuned the pre-trained YOLOv8 model, a CNN-based model by Ultralytics, for object detection. The robotic arm’s movement is guided by path planning algorithms, translating in a joint space.

 The YOLOv8 model was fine-tuned on a dataset of about 3081 images of plastic and metal waste. Due to equipment constraints, a laptop webcam was used, limiting 3D space determination. The model was trained to classify waste into ‘plastic’ and ‘metal’, showing improved results over the base model.
 
 We got help for the inverse kinematics simulation from a video by RoTechnic(https://youtu.be/XDSzbJAwJKA?si=G5__IAsTKwgZgutm).
