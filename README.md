Neuro-Hand: An Award-Winning, Brain-Controlled Prosthetic
🏆 Winner: Best in Neurotech Pipeline at the 2025 Florida Health Hacks Competition 🏆

This repository contains the full source code, presentation deck, and a video demonstration for our prize-winning BCI prosthetic hand, which our team designed, built, and presented in just 48 hours.

Live Demo
[IMPORTANT: Replace the link above with a YouTube link to your demo video. You can also replace the placeholder image with a screenshot of your video.]

The Vision
Our goal was to create a functional, end-to-end pipeline that translates live brain signals (EEG) into direct motor commands for a prosthetic device. We aimed to build a proof-of-concept for a new generation of intuitive, accessible, and affordable assistive technology.

How It Works: The Data Pipeline
The entire system operates on a simple but powerful pipeline:

Brain Wave data (EEG) > Emotiv's Cortex API > Python Script > Arduino > Motor Command

Technology Stack
BCI Headset: Emotiv Insight

Data Streaming: Emotiv Cortex API via WebSockets

Backend & Command Classification: Python (websocket-client, pyserial)

Hardware Control: Arduino Uno, Servo Motors

Prosthetic Design: Fusion 360, 3D Printing

The Pitch Deck
For a full overview of our business case, market analysis (TAM, SAM, SOM), and future vision, please see our final presentation.


How to Run This Project
Prerequisites
Hardware:

An Emotiv BCI Headset (Insight, EPOC+, etc.)

An Arduino Uno

Servo motor(s) for the prosthetic wrist

Software:

Python 3.x

The Emotiv Launcher and Cortex Service running on your machine.

Required Python libraries. Install them via pip:

pip install websocket-client pyserial

Steps to Run
Arduino Setup:

Open the Arduino IDE.

Upload the winch_mvp.ino sketch to your Arduino Uno.

Ensure the Arduino is connected to your computer via USB.

Python Backend:

Open the Jupyter Notebook file (your_notebook_name.ipynb).

Important: Update the SERIAL_PORT variable in the script to match the port your Arduino is connected to (e.g., COM3 on Windows, /dev/tty.usbmodem... on Mac).

Run the cells in the notebook to start the data pipeline.

Emotiv Headset:

Put on the Emotiv headset and ensure it has a good connection.

Train a mental command (e.g., "lift") using the Emotiv app.

Ensure the TRAINED_COMMAND variable in the Python script matches the command you trained.

Once all steps are complete, the Python script will listen for your trained mental command from the headset and send the corresponding motor command to the Arduino.
