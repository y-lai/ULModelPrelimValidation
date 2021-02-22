function filterEMGPlusForce(foldername,options)
% FILTEREMGPLUSFORCE Parses through a folder to find ROS bag files which
% has two topics for EMG data and force torque wrench data. Produces mat
% files for the EMG data and mot files for external loads for OpenSim.
%
%     filterEMGPlusForce(fname)
%
%     Arguments:
%     fname is the name of the folder for the data to process (force torque
%     data from the bag file is optional)
% 
%     Example:
%     filterEMGPlusForce(... , options) specifies other parameters.
%     Parameters are:
%     'emgtopicname'        - the ROS topic name of the EMG data.
%                             (Default is '/labjack/channelstream')
%     'numchannels'         - number of EMG channels (Default is 8)
%     'filterwindow'        - the window size (in seconds) used for EMG
%                             signal filtering. (Default is 0.06)
%     'butterworthfilter'   - boolean variable to decide whether to use a
%                             butterworth filter or a moving RMS filter. 
%                             (Default is 1 indicating butterworth filter)
%     'forcetorquetopicname'- the ROS topic name of the force torque data.
%                             Only force is used. (Default is '/netft_data')
% 
%     NOTE: This function requires the ROS toolbox in MATLAB to process the bag files
%     NOTE2: the layout of the emg data is based on the labjack T7 DAQ. A
%     ROS driver can be found <a href="https://github.com/UTS-CAS/labjack_ros_driver">here</a>.
% 
arguments
    foldername {mustBeText}
    options.emgtopicname {mustBeText}='/labjack/channelstream'
    options.numchannels {mustBeNumeric,mustBeInteger,mustBeNonnegative}=8
    options.filterwindow {mustBeNumeric,mustBeNonnegative}=0.06
    options.butterworthfilter {mustBeNumeric,mustBeInteger,mustBeNonnegative}=1
    options.forcetorquetopicname {mustBeText}='/netft_data'
end
addpath('Utilities')
suffix = '_force.txt';

pf = dir(fullfile(pwd,foldername));
pf = pf([pf.isdir]==1); pf = pf(3:end);
for i=1:numel(pf)
    % no need to grab emg or force files from tpose (only use markerdata
    % for scaling opensim model)
    emgfoldername = fullfile(pwd,[foldername '_emg'],pf(i).name);
    mkdir(emgfoldername);
    bagdir = dir(fullfile(pf(i).folder,pf(i).name,'*.bag'));
    
    for j=1:numel(bagdir)
        disp(['Subfolder ' num2str(i) ' - bag number: ' num2str(j)]);
        prefix = bagdir(j).name(1:end-4);
        bag = rosbag(fullfile(bagdir(j).folder,bagdir(j).name));
        
        sel = select(bag,'Topic',options.emgtopicname);
        [emg,emgtime] = extractAndFilterEMG(sel,options);
        save(fullfile(emgfoldername,[prefix '_emg.mat']),'emg','emgtime');
        
        ftsel = select(bag,'Topic',options.forcetorquetopicname);
        errcode = extractAndFilterForce(ftsel,bagdir(j).folder,[prefix suffix]);
        if(errcode~=0)
            disp(['Error occurred at subfolder ' num2str(i) ' - bag number: '...
                num2str(j) ' - errcode: ' num2str(errcode)]);
        end
    end
end
end

function [emgout,emgtime] = extractAndFilterEMG(sel,options)
% obtain EMG from rosbag selection
msg = readMessages(sel,'DataFormat','struct');
temp = cell(length(msg),1);
time = temp;
for msgidx = 1:length(msg)
    temp{msgidx} = reshape(msg{msgidx}.StreamData.Data,options.numchannels,[])';
    oldtime = double(msg{msgidx}.OldHeader.Stamp.Sec) + double(msg{msgidx}.OldHeader.Stamp.Nsec)/1e9;
    newtime = double(msg{msgidx}.Header.Stamp.Sec) + double(msg{msgidx}.Header.Stamp.Nsec)/1e9;
    time{msgidx} = cumsum(repmat(((newtime-oldtime)/length(temp{msgidx})),length(temp{msgidx}),1)) + oldtime;
end
emgdata = cell2mat(temp); emgtime = cell2mat(time);
% NOTE: currently working based off the notion that EMG readings are mV

% filter EMG
fs = length(emgtime)/(emgtime(end)-emgtime(1));
emgdata = abs(emgdata);
% option for RMS window or butterworth filter 6Hz
if(options.butterworthfilter)
    d = fdesign.lowpass('Fp,Fst,Ap,Ast',3,5,1,30,fs);
    Hdlp = design(d,'butter');
    emgdata = filtfilthd(Hdlp,emgdata);
else
    windowsamples = ceil(options.filterwindow*fs);
    mrmshandle = dsp.MovingRMS(windowsamples);
    emgdata = mrmshandle(emgdata);        
end
emgout = emgdata;
end

function err = extractAndFilterForce(ftsel,foldername,filename)
err = 0;
% obtain force torque data
ftmsg = readMessages(ftsel,'DataFormat','struct');
if(isempty(ftmsg))
    err = -1;
    return;
end
fttime = zeros(length(ftmsg),1); ftdata = repmat(fttime,1,3);
for ftmsgidx = 1:length(ftmsg)
    fttime(ftmsgidx) = double(ftmsg{ftmsgidx}.Header.Stamp.Sec)+double(ftmsg{ftmsgidx}.Header.Stamp.Nsec)/1e9;
    ftdata(ftmsgidx,:) = [ftmsg{ftmsgidx}.Wrench.Force.X ftmsg{ftmsgidx}.Wrench.Force.Y ftmsg{ftmsgidx}.Wrench.Force.Z];
end
fttime = fttime - fttime(1);
% filter ft data
ftfilt = smoothdata(ftdata,'gaussian');
ftpoint = zeros(size(ftfilt));
rows1to7 = cell(7,7);
rows1to7{1,1} = [filename(1:end-4) '.mot'];
rows1to7{2,1} = 'version=1';
rows1to7(3,1:2) = [{'nColumns'} {'7'}];
rows1to7(4,1:2) = [{'datarows'} {num2str(length(ftfilt))}];
rows1to7(5,1:3) = [{'range'} {num2str(fttime(1))} {num2str(fttime(end))}];
rows1to7(6,1) = {'endheader'};
rows1to7(7,:) = [{'time'} {'force_x'} {'force_y'} {'force_z'} ...
    {'point_x'} {'point_y'} {'point_z'}];
ftout = [rows1to7; num2cell(fttime) num2cell(ftfilt) num2cell(ftpoint)];
ftfile = fullfile(foldername,filename);
ftmotfile = [ftfile(1:end-4) '.mot'];
writecell(ftout,ftfile,'Delimiter','tab');
movefile(ftfile,ftmotfile);
end