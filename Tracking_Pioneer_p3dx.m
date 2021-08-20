% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
clear;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% connection status
connected = false;

% ultrasonic sensor handles
rangefinder = zeros(1, 6); 

% Object handles
yawMotor = 0;
pitchMotor = 0;
camera = 0;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % get camera system handles
    [~,camera]=vrep.simxGetObjectHandle(clientID,'VisionSensor',vrep.simx_opmode_blocking);
    [~,pitchMotor]=vrep.simxGetObjectHandle(clientID,'Pitch',vrep.simx_opmode_blocking);
    [~,yawMotor]=vrep.simxGetObjectHandle(clientID,'Yaw',vrep.simx_opmode_blocking);
    
    % initialize camera
    [~, resolution, newImage]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
    
    % ADD CODE HERE IF NECESSARY
    [returnCode,bodyFrame]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
end

 if (connected)   
     
    [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_streaming);
    
    for i = 1 % true
        
        % get image
        [~, resolution, newImage]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_blocking);
        
        % Get the old position.
        Reference_xcoordinate = 16;
        Reference_ycoordinate = -16;        
        
        %ADD CODE HERE
        
        % PID parameter
        Ts = 5;             % time period
        Kcr = 0.05;        
        kp = 0.6;           % the value of P controller
        Ti = Ts * 0.5;      % the value of I controller
        Td = Ts * 0.125;    % the Value of D controller
        integralX = 0;
        integralY = 0;
        last_errorX = 0;
        last_errorY = 0;
        j=0;
     
        while (1)

            % Get the current position.
            [~, resolution, newImage]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_blocking);
            [YcoordinateNew,XcoordinateNew] = find(newImage);
            XcoordinateNew = mean(XcoordinateNew);
            YcoordinateNew = mean(YcoordinateNew);

            % Calculate the error.
            errorX = Reference_xcoordinate - XcoordinateNew;
            errorY = Reference_ycoordinate + YcoordinateNew;

            % Calculate the integral.
            integralX = integralX + errorX;
            integralY = integralY + errorY;

            % Calculate the derivative.
            derivativeX = errorX - last_errorX;
            derivativeY = errorY - last_errorY;

            % Calculate the Control Variable.
            ControlX = Kcr * ((kp * errorX) + (integralX / Ti) + (Td * derivativeX));
            ControlY = Kcr * ((kp * errorY) + (integralY / Ti) + (Td * derivativeY));
       
            % Make the output Value to drive a motor
            [~]=vrep.simxSetJointTargetVelocity(clientID, yawMotor, ControlX , vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID, pitchMotor, ControlY , vrep.simx_opmode_blocking);

            % Feedback 
            last_errorX = errorX;
            last_errorY = errorY;
            
            % Plotting Robot Position 

            [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_buffer);
            j = j+1;
            positionX(j) = currentPosition(1,1); 
            positionY(j) = currentPosition(1,2);
            p1 = plot(positionX,positionY, '-r', 'linewidth',1.1);
            grid on;
            xlabel('Robot Position X-coordinate');
            ylabel('Robot Position Y-coordinate');
            xlim([-2.1 2.1]);
            ylim([-2.1 2.1]);
            title('Robot Motion');
            legend('Robot Coordinates');
            hold on;

            
            
        end

    end
    
 
    % stop camera system motors
    [~]=vrep.simxSetJointTargetVelocity(clientID, pitchMotor, 0, vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID, yawMotor, 0, vrep.simx_opmode_blocking);
    
    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!