function dataout = convertFrame(datain,rawdata,tablename,options)
% CONVERTFRAME Converts array data from one frame to another. Note: this is
% specifically designed for Optitrack rigid body assets and their extracted
% csv marker tracking data. 
% 
%     dataout = convertFrame(datain,rawdata,tablename)
% 
%     Arguments:
%     datain is a n x d array of data to be converted from one frame to
%     another.
%     rawdata is a m x k array of data which contains the rigid body pose
%     tablename is a k x 1 cell array of names which correspond to the
%     markers and/or rigid body assets
% 
%     Examples:
%     dataout = convertFrame(... , options) specifies other parameters for this operation. Parameters are:
%     'markername'  - the name of the markerset with a 7 column pose (XYZW quaternion, then XYZ translation)
%                     Default markername is 'RigidBody4'
%     'offset'      - a 4x4 transform of an offset in opensim frame of reference
%                     Default offset is makehgtform('translate',[0.1 -0.1433 0.002])
% 
arguments
    datain              {mustBeNumeric,mustBeNonNan}
    rawdata             {mustBeNumeric}
    tablename           {mustBeText}
    options.markername  {mustBeText}='RigidBody4'
    options.offset      {mustBeNumeric}=makehgtform('translate',[0.1 -0.1433 0.002]);
end
% check inputs
if(size(options.offset)~=size(zeros(4)))
    error('Incorrect offset transform size. Must be a 4x4 transform.');
elseif(sum(vecnorm(options.offset(1:3,1:3))~=[1 1 1])~=0)
    error(['Rotation part of offset transform is not normalized ' ...
        'correctly. Double check that the rotation matrix is valid.']);
end
% move everything including the offset so thorax is zeroed
temp = reshape(datain',3,[]);
idx = find(strcmp(tablename,options.markername),1,'first');
% specifically for difference between rawdata and preprocessed datain
mset = ceil(((idx-3)/4)/4);
tempdata = temp - rawdata(1,idx+4:idx+6)';
% find transform from rigid body asset
transform = [quat2rotm(rawdata(1,idx:idx+3)) zeros(3,1); 0 0 0 1];
% convert data to rigid body coordinate
tempdata = transform\[tempdata; ones(1,size(tempdata,2))];

% find additional angle difference between ground and chest markers' norm
points = tempdata(1:3,(mset-1)*3+1:mset*3);
V1 = (points(:,2)-points(:,3))/(norm(points(:,2)-points(:,3)));
V2 = cross(V1,points(:,1)-points(:,3))/(norm(cross(V1,points(:,1)-points(:,3))));
V3 = cross(V2,V1);
tran = [V2 V3 -V1 zeros(3,1); 0 0 0 1];
% modify data to include this diff too
tempdata = tran\tempdata;
tempdata = makehgtform('zrotate',pi)*tempdata;

% add offset
tempdata = options.offset*tempdata;

% reshape to previous data array size
% dataout = reshape(tempdata(1:3,:)-options.offset(1:3,4),size(datain,2),[])';
dataout = reshape(tempdata(1:3,:),size(datain,2),[])';
end