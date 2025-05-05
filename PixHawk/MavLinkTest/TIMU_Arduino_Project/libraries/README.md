# TIMU Arduino Project Libraries

This project utilizes several libraries to facilitate communication with the autopilot system and manage data logging. Below is a brief overview of the libraries used in this project:

## Required Libraries

1. **MAVLink**: A lightweight messaging protocol for communicating with drones and other unmanned vehicles. This library is essential for handling MAVLink messages.

2. **SD**: This library provides an interface for reading and writing data to SD cards. It is used for logging MAVLink messages to an SD card.

3. **Streaming**: A library that simplifies the process of sending data to serial ports. It enhances the readability of the code when outputting messages to the serial console.

## Installation

To use this project, ensure that the required libraries are installed in your Arduino IDE. You can find these libraries in the Library Manager or download them from their respective repositories.

## Usage

After setting up the libraries, you can upload the `TIMU_Arduino_Project.ino` file to your Arduino board. The project initializes the serial communication, sets up the logger, and continuously reads messages from the autopilot interface.

For detailed instructions on how to run the project, refer to the main `README.md` file located in the root directory of the project.