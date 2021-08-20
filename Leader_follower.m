% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% connection status
connected = false; 

% motor handles
leftMotor = 0;
rightMotor = 0;

% sensor reading buffers
detectionState = zeros(1, 6);   % Detection status
detectedPoint = zeros(6, 3);    % Point (vector)
range = zeros(1, 6);                % Euclidean distance
% driving velocity
driveVel = 0;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % get object handles
    [~,leftMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,rightMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    
    % get camera system handles
    [~,rgbdCam]=vrep.simxGetObjectHandle(clientID,'rgbd_sensor',vrep.simx_opmode_blocking);    
    
    % initialize camera
    [~, resolution, rgb_image]=vrep.simxGetVisionSensorImage2(clientID,rgbdCam,1,vrep.simx_opmode_streaming);
    [~, d_resolution, depth_image]=vrep.simxGetVisionSensorDepthBuffer2(clientID,rgbdCam,vrep.simx_opmode_streaming);
    
    % ADD CODE HERE IF NECESSARY
    [returnCode,Leader]=vrep.simxGetObjectHandle(clientID,'Pioneer_leader',vrep.simx_opmode_blocking);
    [returnCode,Follower]=vrep.simxGetObjectHandle(clientID,'Pioneer_follower',vrep.simx_opmode_blocking);
end

 if (connected)   

     j = 0;
     [returnCode,currentPositionLeader]=vrep.simxGetObjectPosition(clientID,Leader,-1,vrep.simx_opmode_streaming);
     [returnCode,currentPositionFollower]=vrep.simxGetObjectPosition(clientID,Follower,-1,vrep.simx_opmode_streaming);
     
	% Reference Values.
	Reference_xcoordinate = 32;
	Reference_ycoordinate = 32;
	safeDistance = 1;
        
	%ADD CODE HERE
        
	% PID parameter
    Ts = 5;             % time period
	Kcr = 0.12;
	kp = 0.6;           % the value of P controller
	Ti = Ts * 0.5;      % the value of I controller
	Td = Ts * 0.125;    % the Value of D controller
      
	% Initialise the values
	integralX = 0;
	last_errorX = 0;
	
	integralD = 0;
	last_errorD = 0;

	
    while true  

% Initialise the values and read from the camera        
        % get image
	[~, d_resolution, depth_image]=vrep.simxGetVisionSensorDepthBuffer2(clientID,rgbdCam,vrep.simx_opmode_buffer );
	[~, resolution, rgb_image]=vrep.simxGetVisionSensorImage2(clientID,rgbdCam,1,vrep.simx_opmode_buffer );

	% Get the current position for the X-axsis.
    [YcoordinateNew,XcoordinateNew] = find(rgb_image);
	XcoordinateNew = mean(XcoordinateNew);
	YcoordinateNew = mean(YcoordinateNew);

    % Search for the leader
     while (isnan(XcoordinateNew))
            
           [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor,  -1 , vrep.simx_opmode_blocking);
           [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, 1 , vrep.simx_opmode_blocking);
           [~, d_resolution, depth_image]=vrep.simxGetVisionSensorDepthBuffer2(clientID,rgbdCam,vrep.simx_opmode_buffer );
           [~, resolution, rgb_image]=vrep.simxGetVisionSensorImage2(clientID,rgbdCam,1,vrep.simx_opmode_buffer );
           [YcoordinateNew,XcoordinateNew] = find(rgb_image);
           XcoordinateNew = mean(XcoordinateNew);
           YcoordinateNew = mean(YcoordinateNew);
        
       end

       
           [~, d_resolution, depth_image]=vrep.simxGetVisionSensorDepthBuffer2(clientID,rgbdCam,vrep.simx_opmode_buffer );
           [~, resolution, rgb_image]=vrep.simxGetVisionSensorImage2(clientID,rgbdCam,1,vrep.simx_opmode_buffer );
          
           % Get the current position for the X-axsis.

            [YcoordinateNew,XcoordinateNew] = find(rgb_image);
            XcoordinateNew = mean(XcoordinateNew);
            YcoordinateNew = mean(YcoordinateNew);
            currentDistance = depth_image(int8(YcoordinateNew), int8(XcoordinateNew)) * 3.5;
           
            % Calculate the error
            errorX =  Reference_xcoordinate  - XcoordinateNew;
            errorD = currentDistance - safeDistance;
            
            % Calculate the integral.
            integralX = integralX + errorX;
            integralD = integralD + errorD;
            
            % Calculate the derivative.
            derivativeX = errorX - last_errorX;
            derivativeD = errorD - last_errorD;
            
            % Calculate the Control Variable.
            ControlX = Kcr * ((kp * errorX) + (integralX / Ti) + (Td * derivativeX));
            ControlD = Kcr * ((kp * errorD) + (integralD / Ti) + (Td * derivativeD));
            
            %Robot kinematic
            VL = (ControlD - ControlX * 0.16550);
            VR = (ControlD + ControlX * 0.16550);
            
            % Make the output Value to drive a motor
            [~]=vrep.simxSetJointTargetVelocity(clientID, leftMotor,  VL , vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(clientID, rightMotor, VR , vrep.simx_opmode_blocking);

            
            % Feedback 
            last_errorX = errorX;
            last_errorD = errorD;
           
            [returnCode,currentPositionLeader]=vrep.simxGetObjectPosition(clientID,Leader,-1,vrep.simx_opmode_buffer);
            [returnCode,currentPositionFollower]=vrep.simxGetObjectPosition(clientID,Follower,-1,vrep.simx_opmode_buffer);
            j = j+1;
            positionXLeader(j) = currentPositionLeader(1,1); 
            positionYLeader(j) = currentPositionLeader(1,2);
            positionXFollower(j) = currentPositionFollower(1,1); 
            positionYFollower(j) = currentPositionFollower(1,2);
            grid on;
            xlabel('Robot Position X-coordinate');
            ylabel('Robot Position Y-coordinate');
            xlim([-2.1 2.1]);
            ylim([-2.1 2.1]);
            title('Robot Motion');
            
            p1 = plot(positionXLeader,positionYLeader, '-r', 'linewidth',1.1);
            legend();
            hold on;
            p2 = plot(positionXFollower,positionYFollower, '-g', 'linewidth',1.1);
            legend('Leader Coordinates','Follower Coordinates'); 
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