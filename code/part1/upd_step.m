function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively

% Define the measurement model. Here, Ct is the measurement matrix that
% maps the state space into the measurement space. It consists of an identity
% matrix to select the relevant states, with zeros elsewhere to match dimensions.
Ct = [eye(6), zeros(6, 9)];

% Define the measurement noise covariance matrix.
Rt = 2*eye(6); 

% Compute Kalman gain
A = Ct * covarEst * Ct' + Rt;
B = covarEst * Ct';
Kt = B/A;

% Update state and covariance estimate
uCurr = uEst + Kt * (z_t - Ct * uEst);
covar_curr = covarEst - Kt*Ct*covarEst;

end