delete(instrfindall)
clc
clear all

%% Serial port
% robotCOM = serial('/dev/tty.usbserial-FTEA5PVT', 'BaudRate', 115200);    % OSX [FTDI Cable]
robotCOM = serial('COM4', 'BaudRate', 115200);    % Windows [XBee]
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

%% Load training images
load('../Datos/trainingData_2.mat','pathImages');
[training,~,~] = histeqAndResize(pathImages);
[height, width, nTraining] = size(training);
% clear capture;

%% Variables
% Flags
f_mainContinue = true;      % Main program continue flag
f_autonContinue = false;    % Autonomous run flag

% Images
imageIndex = 1;     % Current image index
matchIndex = 1;     % training image index
captureToMatch = uint8(ones(480, 640, 1));          % Raw image
matchImage = uint8(ones(height, width, 1));  % Proccesed image

% Template matching
errorTH = 0.2;      % Error treshold
offsetTH = 12;      % Offset treshold
lastOffset = 0;     % Last offset error
error = 0;          % Matching error
direction = '';     % Movement direction
match = false;      % Match

% Create template
templateWidth = floor(width/3);
templateRect = [templateWidth, 1, templateWidth, height];

% Movement commands
forward = 'w';
right = 'd';
left = 'a';
stop = 's';

% Data to save
offsetData = zeros(1);
corrCoeffData = zeros(1);

%% Comunication variables
head = '#';         %
command = '';       % 'a','d','w','s'
data = ['','',''];  % 'tens','units','decimals' (ex: 12.5 = ['1','2','5'])
tail = '$';         %

% Program
endString = '';     % End string
iteration = 0;
maxIteration = 10;   % Max number of iterations

%% Main Loop
while f_mainContinue
%     command = input('Command to send: ', 's');
    command = input(']: ', 's');
    switch command
        case {'q'}
            fprintf('\nPROGRAM END\n');
            f_mainContinue = false;

        case {'g'}
            f_autonContinue = true;
            % TODO: getPosition function
            % [node, angle] = getPosition(cameraCOM, video);
            % imageIndex = node;
            % TODO: re-position robot <adjustOrientation>
            % adjustOrientation();
           
            while f_autonContinue
%                 pause()
                data = ['0','0','0'];
                buffer = [head stop data tail]; % Stop
                fwrite(robotCOM, buffer)
                captureToMatch(:,:,imageIndex) = rgb2gray(getsnapshot(video)); % Get current image

                % Template matching
                [matchImage,~,~] = histeqAndResize(captureToMatch(:,:,imageIndex));  % Image to match
                templateImage = training(:,:,matchIndex);           % Template image
                [match, maxCorrCoeff, offset, matchedImageIndex, template, matchSubImage] = templateMatch(matchImage, templateImage, templateRect);
                
                % Save data
                offsetData(imageIndex) = offset;
                corrCoeffData(imageIndex) = maxCorrCoeff;
                
                % Visualized matching process
                figure(1);

                subplot(221)
                imshow(templateImage,'InitialMagnification',100)
                title('Training')
                setPatch(templateWidth, 1, templateWidth, height)

                subplot(222)
                imshow(template)

                subplot(223)
                imshow(matchImage,'InitialMagnification',100)
                title('Match')
                text(matchedImageIndex+(templateWidth/2),height/2,['' num2str(offset)],...
                    'FontSize',18,...
                    'Color','r',...
                    'HorizontalAlignment','center')
                setPatch(matchedImageIndex, 1, templateWidth, height)

                subplot(224)
                imshow(matchSubImage)
                
                % Movie file
                movieRect = get(gcf, 'Position');
                movieRect(1:2) = [0 0];
                movieFile(imageIndex) = getframe(gcf, movieRect);


                if( maxCorrCoeff < errorTH )            % Lost?
                    f_autonContinue = false;            % Lost? [yes]: End
                    f_mainContinue = false;

                    endString = '## LOST (corr. coeff. > error treshold) ##';
                    endError = ['CorrCoeff: ' num2str(maxCorrCoeff)];
                    % TODO: Error handling
                else                                    % Lost? [no]: Move
                    if (abs(offset) > abs(lastOffset))  % Error decreasing?
                        iteration = iteration + 1;      %   [no]: iterations ++
                    else
                        iteration = 0;                  %   [yes]: iterations = 0
                    end

                    if (iteration < maxIteration)       % Too many iterations? [no]: Continue
                        if ( offset < -offsetTH )           % Offset left?
                            data = ['0','5','7'];           %
                            buffer = [head left data tail]; %   [yes]: Move left
                            fwrite(robotCOM, buffer)        %
                        elseif ( offset > offsetTH )        % Offset right?
                            data = ['0','5','7'];           %
                            buffer = [head right data tail];%   [yes]: Move right
                            fwrite(robotCOM, buffer)        %
                        else                                %   [no]: "Image matched" iteration = 0;
                            data = ['4','0','0'];           %
                            buffer = [head forward data tail]; % Move forward
                            fwrite(robotCOM, buffer);       %
                            pause
                            if (matchIndex == nTraining)    % Goal reached?[yes]: Done
                                f_autonContinue = false;
                                f_mainContinue = false;

                                endString = '** Goal reached **';
                                endError = 'No ERROR';

                                matchIndex = -2;
                            end                             % Goal reached?[no]: Continue
                            matchIndex = matchIndex + 1;    % increase index of matched images
                            iteration = 0;
                        end
                    else                                % Too many iterations? [yes]: End
                        f_autonContinue = false;
                        f_mainContinue = false;

                        endString = '## LOST (too many iterarions) ##';
                        endError = ['[' num2str(iteration) '] iterations'];
                    end
                end
                
                imageIndex = imageIndex + 1;
                pause(3);   % Pause for 3 seconds before continuing
                % TODO: Add a better way to know when to continue
            end

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
                    imshow(captureToMatch(:,:,imageNumber))
                else disp('Exceeds image number')
                end
            else disp('No images to diplay')
            end

        otherwise
            errordlg('Type -h for help');
    end
end

fwrite(robotCOM, stop);
fwrite(robotCOM, stop);

fprintf('Program ended with message:\n');
fprintf(endString);
fprintf(endError);
fprintf('\n\n');

% save 'movieFile_6_c' 'movieFile'
% save 'captureToMatch_6_c' 'captureToMatch'
save('autonData_2_e','movieFile', 'captureToMatch', 'offsetData', 'corrCoeffData')

%% Close objects
% Close serial port
fclose(robotCOM);
delete(robotCOM);
fclose(cameraCOM);
delete(cameraCOM);

% Close video capture
stop(video);
delete(video);

% Delete serial ports
delete(instrfindall);