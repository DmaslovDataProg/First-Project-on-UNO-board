# First-Project-on-UNO-board
- Project for Ardruino UNO R3 board with the ultrasonic sensor HC-SR04. This device has the same logic as parktronic device, that measures the distance using ultrasonic waves in order to provide the driver information towards proximity of obstacles.
 Embedded code processes pinging the distance and applying Kalman filter for smoothing. 
As a result, output raw and filtered distance, along with the reflected object velocity are monitored in real time via Ardruino IDE.

- Sample of the collected data on the serial output and the Python script for post processesing is provided as well. 
This script serves to demonstrate the practical benefit of Kalman filter (smoother tracking), as well as residual analysis.

- How to launch this code for just reading the output data on serial output from Ardruino:
1) Import UltrasonicKalman.ino file In Ardruino IDE with ultrasonic sensor connected as provided in the code. 
2) Compile and upload the script to the board and observe serial monitor. 
3) Move reflected object to observe variation

- How to launch this code for recording and post processing the output data:
1) Import ultrasonicKalmanCsv.ino file In Ardruino IDE with ultrasonic sensor connected as provided in the code. 
2) Monitor the output data in serial monitoring software e.g. Putty.
3) Copy serial output data to the .csv file and save it in the folder of the project with arbitrary name, such as "test.csv", as provided in the github repository.
4) In order to process the output data, import the ultrasoundCsvReader.ipynb file to the Jupyter notebook 
5) Run the script, don't forget to change the name of the .csv file to the one that was stored.

- Demo of the functionality: https://www.youtube.com/watch?v=cbBgFX7qj2Y

- Limitations on the hardware: 
The UNO R3 board is very simple chipset, that allows to run such project, so any other Ardruino with similar or higher values of memory and CPU frequency will be capable to execute the code.

- Future implementations: 
1) Add buzzer that can provide sound clue about the distance measured based on filtered values;
2) Add more ultrasound sensors for a 2D problem solution
3) Add LCD screen for visualizing numerical data for distance and velocity