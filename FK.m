function [ee, Jnt] = FK(L, T)
% FK:  Forward Kinematic calculates the position of end effector in 2D for
% any number of DOFs.
%
%  Inputs:
%   L       = Joint Lengths.
%   T       = Joint Angles.
%
%  Outputs:
%   ee      = [x, y] of the end effector.
%   Jnt     = matrix of (2, DOF) that gives the position (x, y)  
%   of all joints starting with base and ending with end effector.
%

    if (length(L) ~= length(T))
        error('myApp:argChk', 'Lenght of L and T vectors must be the seam!')
    end

    tMat = eye(4);
    Jnt = [0 0]';
    for i = 1:length(L)
        Tr  = [   1   0   0   L(i);
                  0   1   0   0 ;
                  0   0   1   0 ;
                  0   0   0   1  ];

        Rt  = [   cos(T(i))  -sin(T(i))   0   0;
                  sin(T(i))   cos(T(i))   0   0;
                  0          0            1   0;
                  0          0            0   1 ];

        tMat = tMat * Rt * Tr;
        Jnt(:,i+1) = [tMat(1,4) tMat(2,4)]';
    end
    tMat;
    msg = ['Answer:     x = ', num2str(tMat(1,4), '%6.4f'), ...
                    ' & y = ', num2str(tMat(2,4), '%6.4f')];
    %disp(msg)
    ee = [tMat(1,4) tMat(2,4)]';
end