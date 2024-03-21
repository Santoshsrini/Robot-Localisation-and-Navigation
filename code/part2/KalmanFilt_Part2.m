clear; % Clear variables
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(7:9,:);%all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time

%write your code here calling the pred_step.m and upd_step.m functions


% Declare global variables for shared access across different scripts/functions

global f J_A J_N x u n ; 


% Define symbolic column vectors of size 3*1 for state 

syms x1 [3 1]
syms x2 [3 1]
syms x3 [3 1]
syms x4 [3 1]
syms x5 [3 1]

% Define symbolic column vectors of size 3*1 for noise

syms ng [3 1]
syms na [3 1]
syms nbg [3 1]
syms nba [3 1]

% Define symbolic column vectors of size 3*1 for IMU angular velocity and
% acceleration

syms wm [3 1]
syms am [3 1]

% Define the state vector x
x = [x1; x2; x3; x4; x5];

% Define the control input vector u
u = [wm; am];

% Define the noise vector n
n = [ng; na; nbg; nba];

% Extract the Euler angles from x2 where phi corresponds to Z, theta to Y
% and psi to X
phi = x2(3);
theta = x2(2);
psi = x2(1);

% Compute the rotation matrix R using the ZYX Euler angles
Rz = [cos(phi), -sin(phi), 0;
      sin(phi), cos(phi),  0;
      0,        0,         1];

Ry = [cos(theta), 0, sin(theta);
      0,          1, 0;
      -sin(theta), 0, cos(theta)];

Rx = [1, 0,         0;
      0, cos(psi), -sin(psi);
      0, sin(psi), cos(psi)];

R = Rz * Ry * Rx;

% Define the matrix T symbolically that relates angular veclocity to euler
% angle derivatives
T = [cos(theta)*cos(phi), -sin(phi), 0;
     cos(theta)*sin(phi), cos(phi),  0;
     -sin(theta),         0,         1];

% Compute G = R' * T, where R' is the transpose of R
G = R' * T;

% Define the gravity vector
g = [0;0;-9.8];

% Define the state transition function f using symbolic vectors
f =  [x3;G\(wm-x4-ng);g+R*(am-x5-na);nbg;nba];


% Compute the Jacobian of f with respect to the state vector
J_A = jacobian(f, x);

% Compute the Jacobian of f with respect to the noise vector
J_N = jacobian(f, n);


% Main loop to process each time step
for i = 1:length(sampledTime)


    disp(i); % Display the current iteration number

    % Extract angular velocity and acceleration from the sampled data IMU

    angVel = sampledData(i).omg;
    acc = sampledData(i).acc;

    % Calculate the time elapsed since the last measurement
    dt = sampledTime(i)- prevTime; 

    % Extract the current measurement of velocity from
    % vicon
    z_t = sampledVicon(7:9,i);

    % Prediction step: estimate the current state and covariance
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt); 

    % Update step: refine the estimate using the current measurement
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);

    % Save the current state
    savedStates(:,i) = uCurr;

    % Update the variables for the next iteration
    prevTime = sampledTime(i);
    uPrev = uCurr;
    covarPrev= covar_curr;



end

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);