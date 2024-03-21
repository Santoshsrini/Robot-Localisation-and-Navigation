 function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%covarPrev and uPrev are the previous mean and covariance respectively
%angVel is the angular velocity
%acc is the acceleration
%dt is the sampling time

% Declare global variables to access the symbolic representations
% and Jacobians of the process model and noise model

global f J_A J_N x u n ; 


% Evaluate the state transition function 'f', Jacobian of 'f' with
% respect to the state 'J_A', and Jacobian of 'f' with respect to 
% the noise 'J_N', substituting previous values for state, control input, 
% and noise. The noise is assumed to be zero for prediction.

f_eval = double(subs(f, [x; u; n], [uPrev; [angVel; acc]; zeros(12, 1)]));
At = double(subs(J_A, [x; u; n], [uPrev; [angVel; acc]; zeros(12, 1)]));
Ut = double(subs(J_N, [x; u; n], [uPrev; [angVel; acc]; zeros(12, 1)]));

% Calculate the discrete-time state transition matrix 'Ft'
% and the input-noise-to-state matrix 'Vt' for the time step 'dt'

Ft = eye(15) + dt * At;
Vt = Ut;


% Define the process noise covariance matrix 'Q'
% and scale it by the time step 'dt' to get 'Qd'
Q = 2*eye(12);
Qd = Q * dt;

% make the mean and covariance estimate of state

uEst = uPrev + dt*f_eval;
covarEst = Ft * covarPrev* Ft'+ Vt*Qd*Vt';


end
