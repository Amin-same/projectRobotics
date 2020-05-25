%% Partikel-Filter
%  =========================
%  Map 03.) Roboter-Pose Scan-Matching
%  A. Same     22.05.2020
%  =========================

close all; clear all; clc

%% initialize the variables
set(0,'DefaultFigureWindowStyle','docked') %dock the figures..just a personal preference you don't need this.
tic
x = 0; % initial actual state
x_N = 0.1; % Noise covariance in the system (i.e. process noise in the state update, here, we'll use a gaussian.)
x_R = 0.1; % Noise covariance in the measurement (i.e. the Quail creates complex illusions in its trail!)
T = 10; % duration the chase (i.e. number of iterations).
N = 10; % The number of particles the system generates. The larger this is, the better your approximation, but the more computation you need.

%initilize our initial, prior particle distribution as a gaussian around
%the true initial value

V = 2; %define the variance of the initial esimate
x_P = []; % define the vector of particles

% generate N number of randomly distributed particals (random positions)
% from the initial prior gaussian distribution
for i = 1:N
    x_P(i) = x + sqrt(V) * randn;
end

%  %{
% show the distribution the particles around this initial value of x.
figure(1)
% clf
subplot(121)
plot(1,x_P,'.k','markersize',5)
xlabel('time step')
ylabel('particle positions')
subplot(122)
histogram(x_P,100)
xlabel('particle positions histogram (x_p_(_i_))')
ylabel('count')
pause(1)
%}

%the functions used to estimate the position and measurements update are:
%Process model, or state update model. This is the function which can be
%any function which shows the robots movement in ideal case.
% xt = 0.5*xt_1 + 25*x/(1 + x^2) + 8*cos(1.2*(t-1)) + PROCESS NOISE --> sqrt(x_N)*randn
% z = x^2/20 + MEASUREMENT NOISE -->  sqrt(x_R)*randn; (observation which we get) measurement update

%generate the observations from the randomly selected particles, based upon
%the given function
 z_out = [x^2 / 20 + sqrt(x_R) * randn];  %the actual output vector for measurement values.
 x_out = [x];  %the actual output vector for state values.
 x_est = [x]; % time by time output of the particle filters estimate
 x_est_out = [x_est]; % the vector of particle filter estimates.

%% Update the movement and measurement, weight the particles, resampling, approach, repeat
for t = 1:T
    %from the previous time step, update the movement position, and observed
    %position (i.e. update the robots position with the non linear function
    %and update from this position the positions which we guess the robot
    %might be.
    x = 0.5*x + 25*x/(1 + x^2) + 8*cos(1.2*(t-1)) +  sqrt(x_N)*randn;
    z = x^2/20 + sqrt(x_R)*randn; % new observation values will be callculated
    %from the given position (measurement update)
    %Here, we do the particle filter
    for i = 1:N % Number of particles
        % given the prior set of particle (i.e. randomly generated (locations),
        % run each of these particles through the state update model to make
        % a new set of transitioned particles(since we have an nonlinear aquation of 
        % robots movements, we will move all particals in the same way,
        % lets say all particals 1 meter forward, if the movements was 1 meter forward)
        x_P_update(i) = 0.5*x_P(i) + 25*x_P(i)/(1 + x_P(i)^2) + 8*cos(1.2*(t-1)) + sqrt(x_N)*randn;
        % with these new updated particle locations, update the observations
        % for each of these particles.
        z_update(i) = x_P_update(i)^2/20;
        % Generate the weights for each of these particles.
        % The weights are based upon the probability of the given
        % observation for a particle, GIVEN the actual observation.
        % That is, if we observe a location z, and we know our observation error is
        % guassian with variance x_R, then the probability of seeing a given
        % z centered at that actual measurement is (from the equation of a
        % gaussian)
        P_w(i) = (1/sqrt(2*pi*x_R)) * exp(-(z_update(i) - z)^2/(2*x_R));
    end
    
    % Normalize to form a probability distribution (i.e. sum to 1).
    P_w = P_w./sum(P_w);
    
%      %{
    figure(2)
    clf
    subplot(151)
    plot(P_w,z_update,'.k','markersize',5)
    hold on
    plot(0,z,'.r','markersize',50)
    xlabel('weight magnitude')
    ylabel('observed values (z update)')    
    subplot(152)
    plot(P_w,x_P_update,'.k','markersize',5)
    hold on
    plot(0,x,'.r','markersize',50)
    xlabel('weight magnitude')
    ylabel('updated particle positions (x P update)')    
    
    %plot the before and after
    subplot(153)
    plot(0,x_P_update,'.k','markersize',5)
    title('raw estimates')
    xlabel('fixed time point')
    ylabel('estimated particles for flight position')
    subplot(154)
    plot(P_w,x_P_update,'.k','markersize',5)
    hold on
    plot(0,x,'.r','markersize',40)
    xlabel('weight magnitude')
    title('weighted estimates')
    %}
    %% Resampling: From this new distribution, now we randomly sample from it to generate our new estimate particles
    
    %what this code specifically does is randomly, uniformally, sample from
    %the cummulative distribution of the probability distribution
    %generated by the weighted vector P_w.  If you sample randomly over
    %this distribution, you will select values based upon there statistical
    %probability, and thus, on average, pick values with the higher weights
    %(i.e. high probability of being correct given the observation z).
    %store this new value to the new estimate which will go back into the
    %next iteration
    for i = 1 : N
        % the cummulative sum of P_w will be calculated. A random number
        % in range of [0,1] will be created. all particals, which are less
        % than this random number will be removed and the other positions
        % will be considered. From the 'x_P_update' vector, the value of the
        % first position which has fulfilled this goal, will be taken.
        x_P(i) = x_P_update(find(rand <= cumsum(P_w),1));
    end
    
    %The final estimate is some metric of these final resampling, such as
    %the mean value or variance
    x_est = mean(x_P);
    
%      %{
    %the after
    subplot(155)
    plot(0,x_P_update,'.k','markersize',5)
    hold on
    plot(0,x_P,'.r','markersize',5)
    plot(0,x_est,'.g','markersize',40)
    xlabel('fixed time point')
    title('weight based resampling')
    pause(1)
    %}
    % Save data in arrays for later plotting
    x_out = [x_out x];
    z_out = [z_out z];
    x_est_out = [x_est_out x_est];
    
end
toc
t = 0:T;
figure(3)
clf
plot(t, x_out, '.-b', t, x_est_out, '-.r','linewidth',3);
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('time step'); ylabel('True position');
legend('True position', 'Particle filter estimate');

set(0,'DefaultFigureWindowStyle','normal')

