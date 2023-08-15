% Replace 'your_stl_file.stl' with the path to your STL file
filename = 'bitetra.stl';

% Use stlread function to read the STL file
[TR,~] = stlread(filename);
faces = TR.ConnectivityList;
vertices=TR.Points;

% Initialize an empty edge list
edgeList = [];

% Loop through each face (triangle) and add its three edges to the edge list
for i = 1:size(faces, 1)
    % Get the indices of the three vertices forming the current face
    vertexIndices = faces(i, :);
    
    % Add each edge of the face to the edge list
    % Edge 1: vertexIndices(1) -> vertexIndices(2)
    % Edge 2: vertexIndices(2) -> vertexIndices(3)
    % Edge 3: vertexIndices(3) -> vertexIndices(1)
    edgeList = [edgeList; [vertexIndices(1), vertexIndices(2)]; [vertexIndices(2), vertexIndices(3)]; [vertexIndices(3), vertexIndices(1)]];
end

% Sort the edge list to remove duplicate edges
edgeList = sort(edgeList, 2);

% Remove duplicate edges
edgeList = unique(edgeList, 'rows', 'stable');

l = length(edgeList);

% Rearrange the edges to form a continuous path
for i = 1:length(edgeList)-1
    if edgeList(i,2) ~= edgeList(i+1,1)
        for j = i+1:length(edgeList)
            lallu = perms(edgeList(j,:));
            for k = 1:length(lallu)
                if lallu(k,1) == edgeList(i,2)
                    temp = edgeList(i+1,:);
                    edgeList(j,:) = temp();
                    edgeList(i+1,:) = lallu(k,:);
                end
            end
        end
    end
end

path = [];
j = length(edgeList);

% Store the final path
while j ~= 0
    path = [path, edgeList(length(edgeList)-j+1,:)];
    j = j - 1;
end

path_final = [path(1)];
j = 1;

% Filter out duplicate vertices to get the final path
for i = 2:length(path)
    if path_final(j) ~= path(i)
        path_final(j+1) = path(i);
        j = j + 1;
    end
end

px = [];
py = [];
pz = [];

% Extract coordinates of the vertices in the final path
for i = 1:length(path_final)
    px = [px; vertices(path_final(i),1)];
    py = [py; vertices(path_final(i),2)];
    pz = [pz; vertices(path_final(i),3)];
end

px_n = [];
py_n = [];
pz_n = [];

stepper = 100;

% Interpolate and generate more points along the path
for i = 1:length(px)-1
    for j = 1:stepper
        px_n = [px_n; px(i) + (j-1) * (px(i+1) - px(i)) / stepper];
        py_n = [py_n; py(i) + (j-1) * (py(i+1) - py(i)) / stepper];
        pz_n = [pz_n; pz(i) + (j-1) * (pz(i+1) - pz(i)) / stepper];
    end
end

multiplier = 2;

% Scale the coordinates by a multiplier
px_n = px_n * multiplier;
py_n = py_n * multiplier;
pz_n = pz_n * multiplier;

p=[px_n,py_n,pz_n,zeros(length(px_n),3)];

% Save the variable 'p' to a .csv file
csv_filename = 'output_points.csv';  % Replace with the desired file name

% Use writematrix to save the data to a .csv file
writematrix(p, csv_filename);
