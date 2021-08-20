% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% connection status
connected = false;

% ultrasonic sensor handles
rangefinder = zeros(1, 6); 

% motor handles
leftMotor = 0;
rightMotor = 0;

% sensor reading buffers
detectionState = zeros(1, 6);   % Detection status
detectedPoint = zeros(6, 3);    % Point (vector)
range = zeros(1, 6);             % Euclidean distance
% driving velocity
driveVel = 2;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % get object handles
    [~,leftMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,rightMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    % proximity sensor handles
    [~,rangefinder(1)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',vrep.simx_opmode_blocking);
    [~,rangefinder(2)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_blocking);
    [~,rangefinder(3)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3',vrep.simx_opmode_blocking);
    [~,rangefinder(4)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4',vrep.simx_opmode_blocking);
    [~,rangefinder(5)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    [~,rangefinder(6)]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor6',vrep.simx_opmode_blocking);
    % initialize proximity sensor reading
    for i = 1:6
    [~,detectionState(i),detectedPoint(i, :),~,~]=vrep.simxReadProximitySensor(clientID,rangefinder(i),vrep.simx_opmode_streaming );
    end
    
    % ADD CODE HERE IF NECESSARY
    [returnCode,bodyFrame]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    
    
    noDetectionDist=0.5;
    maxDetectionDist=0.2;
    detect=[0,0,0,0,0,0];
    braitenbergL=[-1,-2,-3,0,0,0]; % braitenberg matrix for the left motor
    braitenbergR=[-6,-5,-4,0,0,0]; % braitenberg matrix for the right motor
    v0=driveVel; % driving velocity
    j = 0; % for plotting
    t = 0; % for plitting


end

 

 if (connected)   
     
     hold off;
     [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_streaming);
    
    while true
        % run motors
        [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, driveVel, vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, driveVel, vrep.simx_opmode_blocking);
        
        % read rangefinders sensors
        for n = 1:6
            [~,detectionState(n),detectedPoint(n, :),~,~]=vrep.simxReadProximitySensor(clientID,rangefinder(n),vrep.simx_opmode_buffer  );
            range(n) = norm(detectedPoint(n, :)); % Euclidean distance to obstacles 
            
        % Braitenberg algorithm implementation
        
        if (detectionState(n) > 0 && range(n)<noDetectionDist)
            if (range(n)<maxDetectionDist)
                range(n)=maxDetectionDist;
            end
            detect(n)= 1-((range(n)-maxDetectionDist)/(noDetectionDist-maxDetectionDist));
        else
            detect(n) = 0;
            
        end
        end
        vLeft = v0;  
        vRight = v0; 
        
    for n = 1:6
        
        vLeft = vLeft + braitenbergL(n)*detect(n); % set left motor velocity
        vRight = vRight + braitenbergR(n)*detect(n); % set right motor velocity
        [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, vLeft, vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, vRight, vrep.simx_opmode_blocking);

        
    end
    %{
    % Plotting Robot Position 
    
    [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);
    j = j+1;
    positionX(j) = currentPosition(1,1); %X-coordinate robot position
    positionY(j) = currentPosition(1,2); %Y-coordinate robot position   
    p1 = plot(positionX,positionY, '-r', 'linewidth',1.1);
    grid on;
    xlabel('Robot Position X-coordinate');
    ylabel('Robot Position Y-coordinate');
    xlim([-2.1 2.1]);
    ylim([-2.1 2.1]);
    title('Robot Motion');
    legend('Robot Coordinates'); 
    %}
    hold on;
    
    end
    

    % stop wheels
    [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor, 0, vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, 0, vrep.simx_opmode_blocking);
    
    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!