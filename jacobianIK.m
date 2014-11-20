function T = jacobianIK(L, T, G, iMax)
% jacobianIK: Jacobian mplementation of Forward Kinematic using Pseudo 
% Inverse calculates the joint space solution  to reach the goal 2D 
% for any number of DOFs.
%
%  Inputs:
%   L       = Joint Lengths.
%   T       = Joint Angles.
%   G       = Goal position of end effector as [x, y].
%   iMax    = Maximum number of iterations.
%
%  Outputs:
%   T       = Joint space solution in rad.
%
%   

    % Step Size for theta:
    dT = .005;
    % Tolerance for Error:
    tolerance = .002;
    
    % Maximum number of iterations is 500 by default:
    if nargin < 4
        iMax = 500;
    end
    cc = hsv(iMax + 1);
    i = 1;
    DOF = length(T);
    
    % Maximul reach of the arm:
    lim = sum(L);
    % Check if goal is in reach:
    if (lim < norm(G))
        warning('Goal is out of reach! But lets try ...')
        lim = max(G) + .1;
    else
        lim = lim + .1;
    end
    % Start:
    [ee(:,i), Jnt] = FK(L, T);

    figure(1)
    plot(G(1), G(2),'xr', 'MarkerSize', 18, 'LineWidth', 2)
    title('X-Y Plane')
    ylabel('Y')
    xlabel('X')
    axis([-lim lim -lim lim])
    axis square       
    grid on
    hold on
    plot(0, 0, 'sk', 'MarkerFaceColor', 'k', 'MarkerSize', 12)

    % Get euclidean distance from goal and repeate until close enough:
    dist = pdist2(G', ee(:,i)','euclidean');
    while ( dist > tolerance )
        % Plot:
        if ( mod(i, 5) == 0 || i == 1) % So we have cleaner plots            
            figure(1)
            % EE current position:
            plot(ee(1,i), ee(2,i),'o', 'MarkerFaceColor', cc(i,:))
            % Links (Joints):
            for k = 1:length(Jnt)-1
                line([Jnt(1, k), Jnt(1, k+1)],  [Jnt(2, k), Jnt(2, k+1)])
            end

            pause(.1); drawnow
            if (i == 1)
                %input('Press any key to continue ...', 's')
            end
        end

        J = zeros(2, DOF);
        for k = 1:DOF
            % Change each Theta by a small delta and get change in EE:
            delta = zeros(1, DOF);
            delta(k) = dT;
            ee1 = FK(L, T + delta);
            % Build Jacobian Matrix from partial derivatives:
            J(:,k) = (ee1 - ee(:,i));
        end
        
        % Build Jacobian Matrix from partial derivatives:
        J = J / dT;

        % Get Pseudo Inverse of Jacobian Matrix:
        psJ = J'*inv(J*J');


        % Get Theta. value from a normalized error E. (using the distance):    
        dTheta = psJ * ( (G - ee(:,i) ) / dist );

        % Updtate each angle and get the new EE position:
        T = T + (dTheta' * dT);
        i = i + 1;
        [ee(:,i), Jnt] = FK(L, T); 

        % Update the distance:
        dist = pdist2(G', ee(:,i)','euclidean'); 

        % Check if it has taken more than maximum alowable iterations:
        if (i > iMax)
            disp(['Now up to ', num2str(iMax), ' iterations!'])
            disp('That is the max and cannot reach the goal ...')
            disp('Perhaps it is out of reach!')
            break
        end
    end

    figure(1)
    % EE current position:
    plot(ee(1,i), ee(2,i),'o', 'MarkerFaceColor', cc(i,:))
    % Links (Joints):
    for k = 1:length(Jnt)-1
        line([Jnt(1, k), Jnt(1, k+1)],  [Jnt(2, k), Jnt(2, k+1)])
    end


    disp('*  Joint Space Solution (rad.): ')
    disp(T)

    disp(['** Error (Euclidean Distance to Gaol): ', ...
                    num2str(dist), ' m (in ', ...
                    num2str(i), ' steps)'])
end

