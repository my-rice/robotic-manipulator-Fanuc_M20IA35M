function result = parabola(starting_position, R, translation_point, final_position,lambda,concavity, points)
    % This function computes the parabolic path between two points given as input, using the rotation matrix and the translation vector and the length lambda of the path
    % It returns the points of the parabola in the world frame
    
    result = 0;
    
    % rotating and translating the points
    starting_point_rotated = R' * starting_position' - R'*translation_point';
    final_point_rotated = R' * final_position' - R' *translation_point';

    x1 = starting_point_rotated(1);
    y1 = starting_point_rotated(2);

    x2 = final_point_rotated(1);
    y2 = final_point_rotated(2);

    z = final_point_rotated(3);

    %% Rotation of the starting and final point in the rotated frame with y-axis aligned with the line connecting the two points

    % Calculate the angle between the two points
    dx = x2 - x1;
    dy = y2 - y1;
    theta = atan2(dy, dx);

    % Conversion from radians to degrees
    %angle_deg = rad2deg(theta);

    % Rotation matrix along the z-axis
    R_z = [cos(theta), -sin(theta), 0;
           sin(theta), cos(theta), 0;
           0, 0, 1];

    % Compute the inverse of the rotation matrix
    R_z_inverse = inv(R_z);

    
    % Rotation of the starting and final point in the rotated frame with y-axis aligned with the line connecting the two points
    point = [x1; y1; 0];
    transformed_point1 = R_z_inverse * point;

    point = [x2; y2; 0];
    transformed_point2 = R_z_inverse * point;

    %disp('[DEBUG]: Points in the rotated frame with y-axis aligned with the line connecting the two points:')
    %disp(transformed_point1)
    %disp(transformed_point2)

    % Check if the points are in the right order
    if transformed_point1(1) > transformed_point2(1)
        temp = transformed_point1;
        transformed_point1 = transformed_point2;
        transformed_point2 = temp;
    end

    %% Define the symbolic variables
    syms a b c
    x1 = transformed_point1(1);
    y1 = transformed_point1(2);
    x2 = transformed_point2(1);
    y2 = transformed_point2(2);
    
    % Define the equations
    eq1 = y1 == a*x1^2 + b*x1 + c;
    eq2 = y2 == a*x2^2 + b*x2 + c;
    
    % Define the third equation with the integral for arc length
    eq3 = lambda == (sqrt((2*a*x2+b)^2+1)*(2*a*x2+b)+asinh(2*a*x2+b))/(4*a) - (sqrt((2*a*x1+b)^2+1)*(2*a*x1+b)+asinh(2*a*x1+b))/(4*a);
    
    sol = solve([eq1, eq2, eq3], [a, b, c]);

    if isempty(sol.a) || isempty(sol.b) || isempty(sol.c)
        disp(sprintf('1 DEBUG: Failed to solve the system of equations. the distance between the two points is %s', string(sqrt((starting_position(1)-final_position(1))^2+(starting_position(2)-final_position(2))^2+(starting_position(3)-final_position(3))^2))))
        error(sprintf('Failed to solve the system of equations. the distance between the two points is %s', string(sqrt((starting_position(1)-final_position(1))^2+(starting_position(2)-final_position(2))^2+(starting_position(3)-final_position(3))^2))))
    end

    
    %% Display the solution
    disp('Solution:')
    disp(sol)
    disp(sol.a)
    disp(sol.b)
    disp(sol.c)

    % subs the solution in the symbolic function
    a = subs(a, a,sol.a);
    b = subs(b, b,sol.b);
    c = subs(c,c,sol.c);
    
    if strcmp(concavity,'DOWN') & a > 0
        a = -a;
        b = -b;
        c = -c;
    elseif strcmp(concavity,'UP') & a < 0
        a = -a;
        b = -b;
        c = -c;
    end

    %% Check if the solution satisfies the constraint of the arc length
    % additional checks on the solution computed by Matlab
    check = (sqrt((2*a*x2+b)^2+1)*(2*a*x2+b)+asinh(2*a*x2+b))/(4*a) - (sqrt((2*a*x1+b)^2+1)*(2*a*x1+b)+asinh(2*a*x1+b))/(4*a);
    
    if double(check) == double(lambda)
        disp('Coefficients a, b, c satisfy the constraint of the arc length');
    else
        msg = 'check = '+ string(check) +' lambda = '+ string(lambda) +'\nCoefficients a, b, c: ' + string(sol.a)+ ' ' + string(sol.b)+ ' ' + string(sol.c) + ' do not satisfy the constraint of the arc length';
        error(msg)
    end

    %% Plot the function
    %print2D_function(transformed_point1(1), transformed_point2(1), a, b, c)

    %% Compute the x and y coordinates of the parabola
    x_computed = linspace(x1, x2, points);
    y_computed = a*x_computed.^2 + b*x_computed + c;
    
    %% Rotate the sampled points

    points = [x_computed; y_computed; 0*ones(1, length(x_computed))];
    
    % Rotate the sampled points so they are not aligned with the y-axis anymore
    points_rotated = R_z*points;
    
    % Rotate the previous points so they are expressed in the world frame
    points = [points_rotated(1,:); points_rotated(2,:); z*ones(1, length(x_computed))];
    points_rotated = R*points;

    % Translating the points
    points_rotated = points_rotated + translation_point';

    % Return the result
    result = points_rotated';

end
    

