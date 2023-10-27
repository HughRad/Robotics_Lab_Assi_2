%% Robotics
% Lab 11 - Question 2 skeleton code

%% setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information


%% Set up robot
mdl_puma560;                    % Load Puma560
robot = p560;                   % Create copy called 'robot'
robot.tool = transl(0.1,0,0);   % Define tool frame on end-effector


%% Start "real-time" simulation
q = qn;                 % Set initial robot configuration 'q'

HF = figure(1);         % Initialise figure to display robot
robot.plot(q);          % Plot robot in initial configuration
robot.delay = 0.001;    % Set smaller delay when animating
set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 300;  % Set duration of the simulation (seconds)
dt = 0.05;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    [axes, buttons, povs] = read(joy);
       
    % -------------------------------------------------------------
    % YOUR CODE GOES HERE
    % 1 - turn joystick input into an end-effector velocity command
    Kv = 0.05; %linear velocity gain
    Kw = 0.8;

    vx = Kv*axes(1);
    vy = Kv*axes(2);
    vz = Kv*buttons(1);

    wx = Kw*axes(3);
    wy = Kw*axes(6);
    wz = Kw*buttons(2);

    dx = [vx;vy;vz;wx;wy;wz];

    % 2 - use J inverse to calculate joint velocity
    lambda = 0.1;
    J = robot.jacob0(q);
    Jinv = inv((J'*J)+lambda*eye(6))*J';
    qi = Jinv*dx;

    % 3 - apply joint velocity to step robot joint angles 
    q = q + qi'*dt;
    % -------------------------------------------------------------
    
    % Update plot
    robot.animate(q);  
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end
      
