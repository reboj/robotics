%%loads PLY object files
function load = loadObject(x,y)
%Most parts of the code refer to Gavin's video, copy and modified.
%f = faceData;
%v = vertexData
[f,v,data] = plyread(x,'tri'); %read ply format file
partVertexCount = size(v,1); % Get vertex count
%move centrepoint to origin
midPoint = sum(v)/partVertexCount; 

objectV = v - repmat(midPoint,partVertexCount,1); %specify position of the verticals

objectPose = eye(4); % create the matrix 4x4 for partpose XXXX
  
%Scale the colours to be 0-to-1 (they are originally 0-to-255 - need
%further info on this
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

%Plot the TriSurf - plots the 3D triangular surface defined by the points in vectors x, y, z     
objectMesh_h =  trisurf(f,objectV(:,1),objectV(:,2), objectV(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

faceNormals = zeros(size(f,1),3);
    for faceIndex = 1:size(f,1)
        v1 = v(f(faceIndex,1)',:);
        v2 = v(f(faceIndex,2)',:);
        v3 = v(f(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
fn = faceNormals;
%move forward (facing in -y direction)
forwardTR = makehgtform('translate',y); %to be able to translate the object



%Transform the verticies
objectPose = objectPose * forwardTR; %update the location
%objectPose = objectPose * forwardTR * randRotateTR; - if needed to pick up
%object and rotate

%Transform the verticals - new destination and multiply - matrix
%manipulation
updatedPoints = [objectPose * [objectV,ones(partVertexCount,1)]']';  %update the location
objectMesh_h.Vertices = updatedPoints(:,1:3);

end

