## Project Name: Embedded-Systems-Project

### Project Description

This project demonstrates communication between two microcontrollers: one programmed in C and the other in Python. The C microcontroller code handles low-level hardware interactions, while the Python microcontroller code handles higher-level logic and operations. The two microcontrollers communicate to achieve a cohesive functionality, showcasing the integration of different programming environments in embedded systems.

### Files in the Repository

- **main.c**: The main source code file for the C microcontroller.
- **CMakeLists.txt**: The CMake configuration file for building the C project.
- **game.py**: A Python script for the microcontroller running Python, possibly handling game logic or communication protocols.
- **OurTeamwebPage.py**: Another Python script related to the project, likely handling different functionality such as a web page or data visualization.
- **Project.py**: Additional Python script that might contain core functionalities or auxiliary support for the project.

### Getting Started

#### Prerequisites

1. **Pico SDK**: Ensure that the Pico SDK is installed and properly configured on your development machine.
2. **Python Environment**: Ensure that you have Python installed along with any necessary libraries.

#### Setup Instructions

1. **Setting up the C Project**:
    - Navigate to the directory containing `CMakeLists.txt`.
    - Run the following commands to build the project:
      ```sh
      mkdir build
      cd build
      cmake ..
      make
      ```
    - Upload the generated binary to the C microcontroller.

2. **Setting up the Python Project**:
    - Ensure the Python microcontroller is connected and accessible.
    - Upload the Python scripts (`game.py`, `OurTeamwebPage.py`, `Project.py`) to the Python microcontroller.
    - Run the main Python script (`Project.py`), which will handle communication and logic.

### Communication Protocol

The communication between the two microcontrollers can be established via serial communication (UART, SPI, I2C, etc.). The specific implementation details should be checked within the source code:

- **main.c**: Check for any initialization and handling of communication peripherals.
- **Python Scripts**: Check for any libraries or modules that handle the communication on the Python side.

### Usage

1. Power on both microcontrollers.
2. Start the Python script on the Python microcontroller.
3. The C microcontroller will automatically start upon receiving power.
4. Observe the communication and interaction between the two microcontrollers.

### Contributing

Feel free to fork this repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.


