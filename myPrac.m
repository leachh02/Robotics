% Coppelia Sim
% simRemoteApi.start(19999)
% MATLAB
% addpath interface
% dobot = coppeliaRobot()
% clear dobot

function myPrac(word)
    keyboardCoords = [247.5 -77.25 50];
    
    % Convert Rotation (in radians)
    keyboardRot = pi/2;
    Rz1 = [cos(keyboardRot) -sin(keyboardRot) 0;
          sin(keyboardRot)  cos(keyboardRot)  0; 
          0                 0                 1];
    %Homogenous Solution for Dobot -> Keyboard Transformation Matrix
    
    U_T_K = [Rz1 keyboardCoords';
                 0 0 0 1];
    
    M = keyboardLetters(word);
    
    for i = word
        letter = [M(i)'; 1];
        pstar = U_T_K * letter;
        pstar(end) = [];
        keyPress(pstar)
    end
    letter = [M('ENTER')'; 1];
    pstar = U_T_K * letter;
    pstar(end) = [];
    keyPress(pstar)
end

% Helper Functions
function keyPress(pstar)
    % Hovers Desired Key at a height of 50mm
    % angles given out in radians
    angles = [inverseKinematicsGeo(pstar), 0];
    dobot.setJointAngles(angles) 
    
    % Presses Down
    pstar(3) = pstar(3) - 51;
    angles = [inverseKinematicsGeo(pstar), 0];
    dobot.setJointAngles(angles) 
    
    % Goes Back Up
    pstar(3) = pstar(3) + 51;
    angles = [inverseKinematicsGeo(pstar), 0];
    dobot.setJointAngles(angles) 
end

function theta = inverseKinematicsGeo(pstar)
    % pstar - 1x3 vector of desired robot end effector position in millimetres
    % theta - 1x3 vector of robot joint angles in radians

    % robot dimensions - please note the change in dimensions
    L0 = 138;  % height of shoulder raise joint above ground plane
    L1 = 135;  % length of upper arm
    L2 = 147;  % length of lower arm
    L3 = 60;   % horizontal displacement from wrist "passive" joint
    L4 = -70;  % vertical displacement down from wrist "passive" joint
    
    r = sqrt(pstar(1)^2 + pstar(2)^2);

    theta(1) = atan2(pstar(2), pstar(1));

    b = L0 - (-L4 + pstar(3));

    a = r - L3;

    delta = atan2(a,b);

    c = sqrt(a^2 + b^2);

    alpha = acos((L1^2 + L2^2 - c^2)/(2*L1*L2));

    beta = asin(L2*sin(alpha)/c);

    theta(2) = pi - (beta + delta);

    theta(3) = pi - (alpha + (pi/2 - theta(2)));
end

function coords = keyboardLetters(word)
    keyboardRow1 = ['q','w','e','r','t','y','u','i','o','p'];
    keyboardRow2 = ['a','s','d','f','g','h','j','k','l'];
    keyboardRow3 = ['z','x','c','v','b','n','m'];
    
    letterSet = {word(1), word(2), word(3), word(4), word(5), 'ENTER'};
    coordsSet = {};
    
    for letter = word
        counter = 1;
        for i = keyboardRow1
            if i == letter
                %fprintf('%c is in Row 1, position %i \n',i,counter)
                coordsSet{1,length(coordsSet)+1} = [13.5*(counter-0.5), 28.75, 0];
            else
                counter = counter + 1;
            end
        end
        counter = 1;
        for i = keyboardRow2
            if i == letter
                %fprintf('%c is in Row 2, position %i \n',i,counter)
                coordsSet{1,length(coordsSet)+1} = [13.5*(counter-0.5) + 6.75, 17.25, 0];
            else
                counter = counter + 1;
            end
        end
        counter = 1;
        for i = keyboardRow3
            if i == letter
                %fprintf('%c is in Row 3, position %i \n',i,counter)
                coordsSet{1,length(coordsSet)+1} = [13.5*(counter-0.5) + 22.5, 5.75, 0];
            else
                counter = counter + 1;
            end
        end
    end
    %ENTER Co-Ordinates
    coordsSet{1,length(coordsSet)+1} = [137.75, 17.25, 0];
    coords = containers.Map(letterSet,coordsSet);
end
