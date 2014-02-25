delete(instrfindall)
clc
clear all


%% Serial port
% robotCOM = serial('/dev/tty.usbserial-FTEA5PVT', 'BaudRate', 115200);    % OSX [FTDI Cable]
robotCOM = serial('COM14', 'BaudRate', 115200);    % Windows [XBee]
% robotCOM = serial('/dev/ttyUSB0','BaudRate',115200);    % Linux [XBee]
% robotCOM = serial('/dev/ttyUSB0','BaudRate',115200);    % Linux [FTDI]

cameraCOM = serial('COM13','BaudRate', 115200);      % Windows

%% Open serial ports
fopen(robotCOM);    % Robot
fopen(cameraCOM);   % Camera

%% Video input
% video = videoinput('macvideo', 1);    % OSX
video = videoinput('winvideo', 1);    % Windows
% video = videoinput('winvideo', 1, 'YUY2_640x480');    % Windows
% video = videoinput('linuxvideo', 2);     % Linux

set(video, 'ReturnedColorSpace', 'RGB');    % Set video format to RGB
start(video);       % Open video capture
preview(video);     %

%% Main variables
resolution = video.VideoResolution;

f_continue = true;  % Continue flag
imageIndex = 0;
capture = uint8(ones(resolution(2), resolution(1), 1));  % Path images
allCapture = capture;                       % All taken images
commandSent = '*';

%% Comunication variables
head = '#';         %
command = '';       % 'a','d','w','s'
data = ['','',''];  % 'tens','units','decimals' (ex: 12.5 = ['1','2','5'])
tail = '$';         %

%% Nodes variables
nodeIndex = 0;      % Nodes
nNodeImages = 3;     % n Images per node
nodeImages = uint8(zeros(resolution(2), resolution(1), nNodeImages));
nodeAngles = zeros(1,nNodeImages);
node = struct('nodeImages', nodeImages, 'nodeAngles', nodeAngles);

%% Main loop
while f_continue
%     command = input('Command to send: ', 's');
    command = input(']: ', 's');
    switch command
        case {'q'}
            fprintf('\nEND OF TRAINING\n');
            f_continue = false;

        case {'w'}
            nodeIndex = nodeIndex+1;    % New Node
            imageIndex = imageIndex+1;  % New image capture

            % Get new node
            node(nodeIndex) = getNode(nNodeImages, video, cameraCOM);
%             pause(1)
            
            % Get images
            capture(:,:,nodeIndex) = rgb2gray(getsnapshot(video)); % Path image
            allCapture(:,:,imageIndex) = capture(:,:,nodeIndex);   % All images

            commandSent(imageIndex,1) = command;% Get command sent

            % Send commands to robot
            data = ['4','0','0'];               % data = 40.0
            buffer = [head command data tail];
            fwrite(robotCOM, buffer);          % Send command to robot
            robotResponse = fscanf(robotCOM);   % Read robot's response
            disp(robotResponse);                % Display robot's response


        case {'a','s','d','?'}
            imageIndex = imageIndex+1;  % New image capture (No new node)

            % Get images
            allCapture(:,:,imageIndex) = rgb2gray(getsnapshot(video));  %

            commandSent(imageIndex,1) = command;% Get command sent

            % Send commands to robot
            data = ['0','5','7'];               % data = 05.7
            buffer = [head command data tail];
            fwrite(robotCOM, buffer);          % Send command to robot
            robotResponse = fscanf(robotCOM);   % Read robot's response
            disp(robotResponse);                % Display robot's response

        case {'-i'}
            fprintf('%d images taken\n', imageIndex);


        case {'-v'}
            preview(video);
            input('Press any key to quit')
            closepreview


        case {'i'}
            if (imageIndex > 0)
                fprintf('%d images taken, ', imageIndex);
                imageNumber = input('image to display ("q" to quit): ');
                if (imageNumber <= imageIndex)
                    imshow(capture(:,:,imageNumber))
                else disp('Exceeds image number')
                end
            else disp('No images to diplay')
            end


        case {'-h'}
            fprintf('\nCommands (Movements):\n\n');
            fprintf('\t[Action]\t[Character]\n');
            fprintf('\tForwards:\t\t"w" \n\tRight:\t\t\t"d" \n');
            fprintf('\tLeft:\t\t\t"a" \n\tStop:\t\t\t"s" \n');
            fprintf('\nCommands (Program):\n\n');
            fprintf('\t[Action]\t[Character]\n');
            fprintf('\tRobot state:\t"?" \n');
            fprintf('\tQuit program:\t"q" \n');
            fprintf('\tDisplay image:\t"i" \n');

        otherwise
            errordlg('Type -h for help');
    end
end


%% Save data
if (nodeIndex > 0)
    % save('../Datos/capture_6.mat','capture')
    % save('../Datos/allCapture_6.mat','allCapture')
    % save('../Datos/commandSent_6.mat','commandSent')
    % save('../Datos/node_6.mat','node')
    pathImages = capture;
    allImages = allCapture;
    save('../Datos/trainingData_2','node','pathImages','allImages','commandSent')
end

%% Close objects
% Close serial port
fclose(robotCOM);
delete(robotCOM);
fclose(cameraCOM);
delete(cameraCOM);

% Close video capture
stop(video);
delete(video)

% Delete serial ports
delete(instrfindall)
