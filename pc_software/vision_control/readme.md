                                Vision Control Module (PC Software)

This module runs on a PC/laptop and acts as the high-level controller of the system.
It uses computer vision (OpenCV) to detect LED-based sensor nodes, guides a mobile robot toward the selected node,
triggers sensor sampling, and securely collects data from the embedded system.

📌 Responsibilities

The Vision Control software performs the following tasks:
Detects LED-marked sensor nodes using a live ESP32-CAM stream
Estimates distance and alignment using image geometry
Commands the car controller over TCP
Triggers sensor sampling on the SD ESP32 (Data Postman)
Initiates secure data transfer via the Base Station ESP32
Logs received sensor data into CSV files
Supports manual override for speed control and calibration
