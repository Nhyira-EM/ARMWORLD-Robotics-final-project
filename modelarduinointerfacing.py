#importing necessary libraries
import os
import contextlib
from ultralytics import YOLO
import serial.tools.list_ports

ports = list(serial.tools.list_ports.comports()) # List all available serial ports and get their information
port_names = [port.device for port in ports]# Extract the device name (e.g., 'COM3', 'COM4') from each port
portlist = [str(onePort) for onePort in ports]# Create a list of string representations of each port
print(port_names)
SerialInst = serial.Serial()# Initialize the serial communication instance
SerialInst.baudrate = 9600# Set the baud rate for the serial communication
SerialInst.port = 'COM8'#port_names[0]# Assign the first available port to the serial instance (e.g., 'COM3')
SerialInst.open()# Open the serial communication port

print(port_names)# Print the list of available port names (for debugging or informational purposes)

# Load the YOLO model
model = YOLO("C:/Users/nhyir/Documents/INTRO TO AI/Group40_FINALPROJECT_Image-Detection/Imgdetecv2.pt")

while True:
    print('test')
    with contextlib.redirect_stdout(open(os.devnull, 'w')):
        results = model.predict(source="0", show=True, verbose=False,stream=True)  # Set show=False if you don't want to display the predictions
        print('result')
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls.item())  # Get the class ID
            print(class_id)
            label = 'other'  # Default label

            # Assign a label based on the class ID
            if class_id == 0:
                label = 'metal'
            elif class_id == 1:
                label = 'plastic'

            # Print the label of the detected object
            print(f"Detected: {label}")
            SerialInst.write(f"{label}\n".encode('utf-8'))# Write the label to the serial port. encode('utf-8') converts the string to bytes in UTF-8 encoding, which is required for serial communication