function [F, V, N] = stlread(file)
% STLREAD imports geometry from an STL file into MATLAB.
%
%   [F, V, N] = STLREAD(FILE) reads an STL file (binary or ASCII)
%   and returns the face-vertex data.
%
%   F is a triangulation matrix where each row is a triangle defined by the
%   indices of its 3 vertices.
%   V is a vertex matrix where each row is a vertex defined by its 3
%   coordinates (x, y, z).
%   N is the normal vector for each face (not necessarily a unit vector).
%
%   Based on the work of:
%   - Doron Harlev (original version, 2003)
%   - Eric Johnson (binary file support, 2006)
%   - Francis Esmonde-White (robustness and speed improvements, 2011)

if ~exist(file, 'file')
    error('File ''%s'' not found.', file);
end

fid = fopen(file, 'r');
if fid == -1
    error('Cannot open file ''%s''.', file);
end

% Check for binary or ASCII format by checking the file size
fseek(fid, 0, 'eof');
file_size = ftell(fid);
fseek(fid, 80, 'bof');
num_triangles = fread(fid, 1, 'uint32');
fclose(fid);

expected_binary_size = 84 + 50 * num_triangles;

if file_size == expected_binary_size
    is_binary = true;
else
    is_binary = false;
end

if is_binary
    [F, V, N] = stlread_binary(file);
else
    [F, V, N] = stlread_ascii(file);
end

end

function [F, V, N] = stlread_binary(file)
    fid = fopen(file, 'rb');
    
    % Skip header
    fread(fid, 80, 'uint8');
    
    % Read number of triangles
    num_triangles = fread(fid, 1, 'uint32');
    
    % Pre-allocate arrays
    N = zeros(num_triangles, 3, 'single');
    V_flat = zeros(num_triangles * 3, 3, 'single');
    
    % Read triangle data
    for i = 1:num_triangles
        N(i, :) = fread(fid, 3, 'single');
        V_flat((i-1)*3 + 1, :) = fread(fid, 3, 'single');
        V_flat((i-1)*3 + 2, :) = fread(fid, 3, 'single');
        V_flat((i-1)*3 + 3, :) = fread(fid, 3, 'single');
        fread(fid, 1, 'uint16'); % Attribute byte count
    end
    
    fclose(fid);
    
    % Consolidate vertices
    [V, ~, ic] = unique(V_flat, 'rows');
    F = reshape(ic, 3, num_triangles)';
end

function [F, V, N] = stlread_ascii(file)
    fid = fopen(file, 'rt');
    if fid == -1
        error('Cannot open file: %s', file);
    end
    
    v_list = [];
    n_list = [];
    
    line = fgetl(fid);
    while ischar(line)
        line = strtrim(line);
        if startsWith(line, 'facet normal')
            n_list(end+1, :) = sscanf(line, 'facet normal %f %f %f');
        elseif startsWith(line, 'vertex')
            v_list(end+1, :) = sscanf(line, 'vertex %f %f %f');
        end
        line = fgetl(fid);
    end
    fclose(fid);
    
    if isempty(v_list) || mod(size(v_list, 1), 3) ~= 0
        error('Invalid ASCII STL file: Number of vertices is not a multiple of 3.');
    end
    
    N = n_list;
    V_flat = v_list;
    
    [V, ~, ic] = unique(V_flat, 'rows');
    num_triangles = size(V_flat, 1) / 3;
    if mod(num_triangles, 1) ~= 0
        error('Invalid ASCII STL file: Number of vertices is not a multiple of 3.');
    end
    F = reshape(ic, 3, num_triangles)';
end
