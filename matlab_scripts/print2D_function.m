function print2D_function(x1, x2, a, b, c)
    % This function prints a 2D function given two points x1, x2 and the coefficients of the parabola a, b, c, given as as input

    % Define the symbolic variable
    syms x
    
    % Define the symbolic function
    f(x) = a*x^2 + b*x + c;
    
    % Plot the function
    fplot(f, [x1, x2])
    
    % Add title and labels
    title('Plot of the function f(x) = '+string(a)+ '*x^2+'+string(b)+'*x+'+string(c))
    xlabel('x-axis')
    ylabel('y-axis')
end