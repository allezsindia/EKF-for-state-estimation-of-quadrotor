clear; % Clear variables
datasetNum = 9; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(1:6,:);%all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time
%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)

    dt = sampledTime(i) - prevTime;
    angVel = sampledData(i).omg;
    acc = sampledData(i).acc;
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt); % Making a prediction of the mean and covariance
    [uCurr,covarCurr] = upd_step(Z(:,i),covarEst,uEst); %Making the update
    savedStates(:,i)= uCurr; %Saving the states so that we can plot them
    % update the previous covariance and mean get updated to the current mean and conariance along with the time
    uPrev = uCurr; 
    covarPrev = covarCurr;
    prevTime = sampledTime(i);

end
plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);
