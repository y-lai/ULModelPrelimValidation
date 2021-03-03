function extractTRCmarkerfiles(foldername,markernames,convert)
% EXTRACTTRCMARKERFILES Parses through a folder to find csv files which
% consist of marker data for motion capture. Produces trc files for inverse
% kinematics for OpenSim.
% 
%     extractTRCmarkerfiles(fname,mnames,convert)
% 
%     Arguments:
%     fname         - name of the folder for the data to process
%     markernames   - a cell array of the names to search for in the csv
%                     file to match for the trc files
%     convert       - boolean variable to apply user-defined function named
%                     convertFrames to convert to different frames of 
%                     reference. (Default value is 1)
% 
%     NOTE: this function is defined for the csv files exported from
%     Optitrack (NaturalPoint). Further changes might be needed if using
%     another motion capture software.
% 
arguments
    foldername {mustBeText}
    markernames {mustBeText}
    convert {mustBeNumeric,mustBeNonnegative}=1
end
if(~((convert==1)||(convert==0)))
    disp('Invalid value for convert is used. Setting to default value of 1.');
    convert = 1;
end

addpath('Utilities');
% find participant subfolders for marker data
pf = dir(fullfile(pwd,foldername));
pf = pf([pf.isdir]==1); pf = pf(3:end);
for np = 1:length(pf)
    tposefile = fullfile(pf(np).folder,[pf(np).name '_tpose.csv']);
    if(isfile(tposefile))
        % deal with tpose csv files
        extractFromCSV(tposefile,markernames,'_marker.txt',convert);
    else
        disp(['No tpose file for ' pf(np).name '.']);
    end
    
    % subdir for each participant
    csvdir = dir(fullfile(pf(np).folder,pf(np).name,'*.csv'));
    for j=1:length(csvdir)
        filename = fullfile(csvdir(j).folder,csvdir(j).name);
        extractFromCSV(filename,markernames,'_marker.txt',convert);
    end
end
end

function extractFromCSV(csvfile,markernames,suffix,convert)
    fid = fopen(csvfile);
    varNames = strsplit(fgetl(fid),',');
    fclose(fid);
    
    % trc header variables
    takename = varNames{4};
    framerate = str2double(varNames{6});
    frames = str2double(varNames{12});
    
    % extract csv data from optitrack markers csv files
    T = readtable(csvfile,'HeaderLines',3);
    A = table2array(T);
    
    % find column indexes matching the markernames
    Tname = T.Properties.VariableDescriptions;
    tmp = strcmp(Tname,'');
    for i = 1:length(markernames)
        cols = find(strcmp(Tname,markernames{i}));
        cold = find(diff(cols)>1,1,'first'); 
        tmp(cols(1:cold-1)) = 1;
    end
    % NOTE: we are using rigid body assets in Motive software which gives
    % additional column for fit of rigid body and marker error
    trchdata = A(:,1:2);
    trcdata = A(:,tmp);
    % remove nan readings
    trchdata = trchdata(sum(isnan(trcdata),2)==0,:);
    trchdata(:,1) = (1:size(trchdata,1))';
    trcdata = trcdata(sum(isnan(trcdata),2)==0,:);
    A = A(sum(isnan(trcdata),2)==0,:);
    
    % sanity check whether data fits markers listed
    % check whether it's xyz only
    if(mod(size(trcdata,2),3)~=0) 
        error(['Size of extracted marker data does not fit cartesian xyz.' ...
            '\nPlease double check data.']);
    end
    % whether all marker names are accounted for
    if(size(trcdata,2)/3 ~= length(markernames)) 
        error(['Not all markernames detected from data.' ...
            'Please double check cell array of markernames passed matches the data.']);
    end
    
    % (optional) convert everything to thorax frame
    if(convert==1)
        % convert data to coord system of thorax rigid body
        trcdata = convertFrame(trcdata,A,Tname);
    end
    
    % fill out table/array ready for writing trc file
    row1to5 = cell(5,size(trcdata,2)+size(trchdata,2));
    row1to5(1,1:4) = {'PathFileType','4','(X/Y/Z)',takename};
    row1to5(2,1:8) = {'DataRate','CameraRate','NumFrames','NumMarkers','Units','OrigDataRate','OrigDataStartFrame','OrigNumFrames'};
    row1to5(3,1:8) = {framerate,framerate,length(trcdata),length(markernames),'m',framerate,'0',frames};
    row1to5(4,1:2) = [{'Frame#'} {'Time'}];
    for i=1:length(markernames)
        row1to5(4,i*3) = markernames(i);
    end
    X = cell(length(markernames),1); Y = X; Z = X;
    for i=1:length(X)
        X{i} = ['X' num2str(i)];
        Y{i} = ['Y' num2str(i)];
        Z{i} = ['Z' num2str(i)];
    end
    row1to5(5,3:end) = reshape([X Y Z]',1,[]);
    % convert to cell arrays for exporting to .trc file
    trcout = [row1to5; num2cell(trchdata) num2cell(trcdata)];
    txtfilename = [csvfile(1:end-4) suffix];
    trcfilename = [txtfilename(1:end-4) '.trc'];
    writecell(trcout,[csvfile(1:end-4) suffix],'Delimiter','tab');
    movefile(txtfilename,trcfilename);
end