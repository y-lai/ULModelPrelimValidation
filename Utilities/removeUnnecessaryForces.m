function [outforces,outfnames,outcnames,outidx] = removeUnnecessaryForces(inforces,forcenames,coordnames,tolerance)
% REMOVEUNNECESSARYFORCES trawls through coordinates to group coordinates
% which belong to the same skeletal group, checks maximum absolute forces
% induced from inverse dynamics, and removes force data for those groups if
% forces are less than tolerance value. Updates all 3 inputs.
% 
% Use case:
%     [outforces,outforcenames,outcoordnames] =
%     removeUnnecessaryForces(inputforces,inputcoords,forcenames,coordnames,tolerance);
% 
% Arguments:
%     inputforces - a nxm array of forces calculated from ID and
%                   converted/filtered with loadFilterCropArray function.
%     forcenames  - a mx1 cell array of names corresponding to the m
%                   columns of force data from inverse dynamics results.
%                   Force names must be re-arranged from ID results to 
%                   align with coordinate names cell array.
%     coordnames  - a mx1 cell array of names corresponding with the
%                   coordinates in inverse dynamics.
%     tolerance   - a scalar value indicating the minimum absolute force
%                   which contributes towards the optimization
%                   (Default value is 0.01)
% 
% Outputs:
%     outforces   - a nxk array of forces which indicate necessary forces
%                   for static optimization.
%     outfnames   - a kx1 cell array of the force names corresponding to
%                   necessary forces for static optimization.
%     outcnames   - a kx1 cell array of the coordinate names corresponding
%                   to necessary forces for static optimization.
%     outidx      - a kx1 array of the indexes for necessary forces
% 
% Example:
%     [oforce,ofnames,ocnames,~] = removeUnnecessaryForces(forces,fnames,cnames,0.004);
% 
% Refer to <a
% href="https://simtk-confluence.stanford.edu/display/OpenSim/Custom+Static+Optimization+in+MATLAB">custom static optimization in MATLAB</a>
% to find out more about this function's part in the opensim workflow
% 
arguments
    inforces {mustBeNumeric,mustBeNonNan}
    forcenames
    coordnames
    tolerance {mustBeNumeric,mustBeNonNan,mustBeNonnegative}=0.01
end
groups = cell(200,2); groupnum = 1;
% first force group name
idx = strfind(coordnames{1},'_'); curridx = 1;
if(~isempty(idx))
    curr = coordnames{1}(1:idx);
else
    curr = [coordnames{1} '_'];
end
% for subsequent names
for i=2:length(coordnames)
    if(contains(coordnames{i},curr))
        curridx = [curridx; i];
    else
        groups(groupnum,:) = [{curr} {curridx}];
        groupnum = groupnum+1;
        idx = strfind(coordnames{i},'_'); curridx = i;
        if(~isempty(idx))
            curr = coordnames{i}(1:idx);
        else
            curr = [coordnames{i} '_'];
        end
    end
end
groups(groupnum,:) = [{curr} {curridx}];
groups = groups(1:groupnum,:);
% max absolute forces for groups (to be necessary or not)
outidx = [];
for i=1:length(groups)
    maxforces = max(abs(reshape(inforces(:,groups{i,2}),[],1)));
    if(maxforces>tolerance)
        outidx = [outidx; groups{i,2}];
    end
end
outforces = inforces(:,outidx');
outcnames = coordnames(outidx);
outfnames = forcenames(outidx);
end