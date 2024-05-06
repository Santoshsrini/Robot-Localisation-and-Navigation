# **ROB-6213 Project 1: Extended Kalman Filter for State Estimation**

## **Overview**

In this project, Extended Kalman Filter (EKF) was implemented in MATLAB for state estimation of a Micro Aerial Vehicle. The filter fused data from an onboard IMU and a Vicon motion capture system to estimate the vehicle's position, velocity, orientation, and sensor biases. Two versions of the filter were developed, one using Vicon position and orientation measurements, and another using only velocity measurements. The project involved designing process and measurement models, implementing the EKF prediction and update steps, and evaluating the filter's performance on three datasets. The results were summarized in a report, discussing the approach, presenting the estimation results, and comparing them with the ground truth data. This project helped improve my understanding of state estimation techniques, sensor fusion, and proficiency in MATLAB programming for robotics applications.


## **Project Structure**
The project is divided into two parts:
1. **Part 1**: EKF with measurement update using position and orientation from Vicon.
2. **Part 2**: EKF with measurement update using velocity from Vicon.

Each part has a separate folder containing the necessary code files.

## **Dataset**
The project uses three datasets provided in MAT files. Each MAT file contains Vicon data (timestamp and pose/velocity measurements) and synchronized IMU data (body frame acceleration and angular velocity).

## **Extended Kalman Filter**
The EKF implementation follows the approach presented in the course lectures. The process model remains the same for both parts, while the measurement models differ based on the available Vicon data (position and orientation for Part 1, velocity for Part 2).

The EKF code is implemented in the `KalmanFilt_Part1.m` and `KalmanFilt_Part2.m` files for each respective part. The `init.m` function is used to initialize the process and select the specific dataset.

## **Results**
The EKF estimates are compared with the ground truth provided by the Vicon data. The results are plotted using the provided `plot` function for each of the three datasets.

### Part 1:

Dataset 1:

<img width="1188" alt="image" src="https://github.com/Santoshsrini/Robot-Localisation-and-Navigation/assets/28926309/22350796-acfb-491b-88a5-3d9e4de86fde">

Dataset 4:

<img width="1199" alt="image" src="https://github.com/Santoshsrini/Robot-Localisation-and-Navigation/assets/28926309/d8e2217a-ca98-448b-944a-cb203745ca82">

Dataset 9:

<img width="1221" alt="image" src="https://github.com/Santoshsrini/Robot-Localisation-and-Navigation/assets/28926309/d52b9dba-af3a-4d21-9a21-21848794aeb0">

### Part 2:

Dataset 1:

<img width="1192" alt="image" src="https://github.com/Santoshsrini/Robot-Localisation-and-Navigation/assets/28926309/45491d91-537c-4d25-ac19-a45661806cc5">

Dataset 4:

<img width="1182" alt="image" src="https://github.com/Santoshsrini/Robot-Localisation-and-Navigation/assets/28926309/c7e3314d-2e2a-4242-90bd-a75327838ddc">

Dataset 9:

<img width="1174" alt="image" src="https://github.com/Santoshsrini/Robot-Localisation-and-Navigation/assets/28926309/7dc8e89e-0655-4aba-bc92-921ad24e7336">

## **Dependencies**
- MATLAB 2023b

## **Running the Code**
1. Navigate to the respective part folder (`Part1` or `Part2`).
2. Run the corresponding `KalmanFilt_Part1.m` or `KalmanFilt_Part2.m` script.
3. The code will process the selected dataset and generate plots comparing the EKF estimates with the Vicon ground truth.

