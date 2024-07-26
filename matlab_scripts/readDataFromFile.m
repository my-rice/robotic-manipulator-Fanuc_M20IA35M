function [starting_position, final_position, rotation_matrix, traslation_vector] = readDataFromFile(filename)
    % This function reads data from a file, which name is taken as input, assuming that the file contains the following data:
    % STARTING POSITION
    % END POSITION
    % ROTATION MATRIX
    % TRANSLATION VECTOR


    % Open the file for reading
    fid = fopen(filename, 'r');

    % Check if the file is successfully opened
    if fid == -1
        error('Unable to open the file.');
    end

    % Initialize vectors
    starting_position = [];
    final_position = [];
    rotation_matrix = [];
    traslation_vector = [];

    % Read data from the file
    while ~feof(fid)
        % Read a line from the file
        line = fgetl(fid);
        
        % Skip empty lines
        if isempty(line)
            continue;
        end

        if strcmp(line, 'STARTING POSITION')
            % Read the starting position
            line = fgetl(fid);  
            % Split the line into values (assuming space-separated values)
            disp("DEBUG: ")
            disp(line);
            values = str2double(strsplit(line));
            starting_position = values(1:3);
            disp('Starting position:');
            disp(starting_position);
            
        elseif strcmp(line, 'FINAL POSITION')
            % Read the final position
            line = fgetl(fid); 
            % Split the line into values (assuming space-separated values)
            values = str2double(strsplit(line));

            final_position = values(1:3);
            disp('Final position:');
            disp(final_position);
        elseif strcmp(line, 'ROTATION MATRIX')
            % Read the rotation matrix
            line = fgetl(fid); 
            % Split the line into values (assuming space-separated values)
            values = str2double(strsplit(line));

            rotation_matrix = [values(1:3);values(4:6);values(7:9)];
            disp('Rotation matrix:');
            disp(rotation_matrix);
        elseif strcmp(line, 'TRANSLATION VECTOR')
            % Read the translation vector
            line = fgetl(fid);
            % Split the line into values (assuming space-separated values)
            values = str2double(strsplit(line));

            traslation_vector = values(1:3);
            disp('Translation vector:');
            disp(traslation_vector);
        else
            % Skip the line
            continue;
        end
    end

    % Close the file
    fclose(fid);
end