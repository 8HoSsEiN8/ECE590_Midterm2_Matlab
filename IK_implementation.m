clear all; close all; clc;
  
L = [.3 .2 .1];         % Kinematic Values
T = [1 1 1] * pi/50;    % Initial Pose

%% Set 1:
close all; clc;

G = [.1 .1]';           % Desired EE location 
jacobianIK(L, T, G);

%% Set 2:
close all; clc;

G = [.2 .2]';           % Desired EE location 
jacobianIK(L, T, G);

%% Set 3:
close all; clc;

G = [.3 .3]';           % Desired EE location 
jacobianIK(L, T, G);

%% Set 4:
close all; clc;

G = [.0 .3]';           % Desired EE location 
jacobianIK(L, T, G);

%% Set 5:
close all; clc;

G = [-.1 .1]';           % Desired EE location 
jacobianIK(L, T, G);

%% Set 6:
close all; clc;

G = [-.2 .2]';           % Desired EE location 
jacobianIK(L, T, G);

%% Set 7:
close all; clc;

G = [.3 -.2]';           % Desired EE location 
jacobianIK(L, T, G);

%% Set 8:
close all; clc;

G = [.3 .8]';           % Desired EE location 
jacobianIK(L, T, G);

%% Test 1 with 11 DOFs:
close all; clc;
L = [.3 .2 .1  .1 .2 .1  .3 .1 .1  .1 .1];          % Kinematic Values
T = [.4 .6 .2  .5 .1 .7  .3 .2 .1  .1 .1] * pi/50;  % Initial Pose

G = [.3 .8]';           % Desired EE location 
jacobianIK(L, T, G);

%% Test 2 with 11 DOFs:
close all; clc;
L = [.3 .2 .1  .1 .2 .1  .3 .1 .1  .1 .1];          % Kinematic Values
T = [.4 .6 .2  .5 .1 .7  .3 .2 .1  .1 .1] * pi/50;  % Initial Pose

G = [-.5 .8]';           % Desired EE location 
jacobianIK(L, T, G);

%% Test 3 with 11 DOFs:
close all; clc;
L = [.3 .2 .1  .1 .2 .1  .3 .1 .1  .1 .1];          % Kinematic Values
T = [.4 .6 .2  .5 .1 .7  .3 .2 .1  .1 .1] * pi/50;  % Initial Pose

G = [-.5 -1]';           % Desired EE location 
jacobianIK(L, T, G);

