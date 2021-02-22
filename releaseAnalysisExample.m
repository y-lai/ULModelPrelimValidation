% releaseAnalysisExample.m
% Assumes that releaseExampleScript.m has been conducted. This should
% result in a folder with _emg suffix indicating the real-world sEMG data.
% Similarly, subfolders for the results from OpenSim operations are assumed
% to exist.
clear
close all
addpath('Utilities');

datafoldername = 'markerdata';
savefile = 0; % set to 1 to save the figure as a pdf

% an index of the relevant muscles which have sEMG readings
% BIC Long,BIC Short,TRI Long,TRI Lateral,DELT Medius,DELT Posterior,DELT Anterior
muscidx = [20 21 16 17 2 1 3];
optimizationdt = 0.025; % time difference between frames from SO (in seconds)

% for playing around with figure position inside the figure window
figpos = [0.17 0.1 0.82 0.77];
% for playing around with position of legend in figure
legoffset = [0 0.08 0 0];
% fontsizes for various aspects of figure
legfontsize = 15; tickfontsize = 25; titlefontsize = 30;

% find index of all results in folder (needs to be in order dictated by
% runOpenSim.m)
folder = dir(fullfile(pwd,datafoldername)); folder = folder([folder.isdir]==1);
folder = folder(3:end);

% the number of spaces will need to be modified depending on the screen
% resolution and size (MATLAB figures are resolution based thus will change
% from screen to screen)
channelnames = [{'      BIC'} {'      TRI\newline     Long'}  ...
    {'      TRI\newline  Lateral'} {'     DELT\newline   Medius'} ...
    {'     DELT\newlinePosterior'} {'     DELT\newline Anterior'}];
bicl = 624.3; bics = 435.56;

% for each participant
for i=1:length(folder)
    trcdir = dir(fullfile(folder(i).folder,folder(i).name));
    trcdir = trcdir([trcdir.isdir]==1); trcdir = trcdir(3:end);
    % for each trial folder
    for j=1:length(trcdir)
        trialprefix = trcdir(j).name(1:end-7);
        flvmatname = fullfile(pwd,datafoldername,folder(i).name,trcdir(j).name,[trcdir(j).name '_SOflvMuscleRes.mat']);
        trialmatname = fullfile(pwd,datafoldername,folder(i).name,trcdir(j).name,[trcdir(j).name '_SOidealMuscleRes.mat']);
        emgmatname = [fullfile(pwd,[datafoldername '_emg'],folder(i).name,trialprefix) '_emg.mat'];
        
        % data for muscles with FLV constraints
        b = load(flvmatname);
        flvtime = (1:length(b.actRes))'*optimizationdt;
        flvres = b.actRes(:,muscidx);
        
        % need to merge biceps long head and short head (take max reading)
        % since real world emg data is only single electrode. Assume they
        % work constructively. Use max isometric force ratio for end
        % activation (624.30N:435.56N)
        flvresults = [(bicl/(bicl+bics))*flvres(:,1)+(bics/(bicl+bics))*flvres(:,2) flvres(:,3:end)];
        
        % RMS window emg output (50-100ms window - Konrad 2005)
        windowsamples = floor(0.08/optimizationdt); % using 80ms
        mrmshandle = dsp.MovingRMS(windowsamples);
        flvpres = mrmshandle(flvresults);

        % data for muscles with ideal force constraints
        a = load(trialmatname);
        trialtime = (1:length(a.actRes))'*optimizationdt;
        res = a.actRes(:,muscidx);
        results = [(bicl/(bicl+bics))*res(:,1)+(bics/(bicl+bics))*res(:,2) res(:,3:end)];
        lpres = mrmshandle(results);        
        
        % data from real-world sEMG readings
        load(emgmatname);
        emgtime = emgtime-emgtime(1);
        emg = emg(:,[1 2 3 4 5 7]); % using only 6 channels from the 8-channel DAQ
        
        % downsample emg readings (required since 12kHz is not necessary for comparisons
        ox = linspace(0,1,length(emgtime))';
        nx = linspace(0,1,ceil(length(emgtime)/120))'; % downsample to 1kHz
        emgdownsampled = interp1(ox,emg,nx);
        emgtimedownsampled = interp1(ox,emgtime,nx);
        
        % normalize emg data to 0-1 estimates.
        % Custom range used to indicate 0-1. Assume max mV 
        % readings at 0.8 (since we're checking trends only)
        maxmvc = max(emgdownsampled)/0.8;
        emgdownsampled = emgdownsampled./repmat(maxmvc,length(emgdownsampled),1);

        % remove first and last 100ms
        emgdownsampled = emgdownsampled((emgtimedownsampled>0.1) ...
            &(emgtimedownsampled<emgtimedownsampled(end)-0.1),:);
        emgtimedownsampled = emgtimedownsampled((emgtimedownsampled>0.1) ...
            &(emgtimedownsampled<emgtimedownsampled(end)-0.1));
        
        lpres = lpres((trialtime>0.1)&(trialtime<trialtime(end)-0.1),:);
        trialtime = trialtime((trialtime>0.1)&(trialtime<trialtime(end)-0.1));
        
        flvpres = flvpres((flvtime>0.1)&(flvtime<flvtime(end)-0.1),:);
        flvtime = flvtime((flvtime>0.1)&(flvtime<flvtime(end)-0.1));
        
        % re orient time to start from 0
        emgtimedownsampled = emgtimedownsampled - emgtimedownsampled(1);
        flvtime = flvtime - flvtime(1);
        trialtime = trialtime - trialtime(1);
        
        % colours for the line plots
        ctemp = [0 0.447 0.741; 0.85 0.325 0.098];
        h = figure(j);
        h = plotMultichannel(emgtimedownsampled,emgdownsampled,...
            'ylabels',channelnames,'plothandle',h,'color',ctemp(1,:),...
            'fontsize',tickfontsize,'linewidth',1.5);
        h = plotMultichannel(flvtime,flvpres, ...
            'ylabels',channelnames,'plothandle',h,'color',ctemp(2,:),...
            'fontsize',tickfontsize,'linewidth',1.5);
        [h,ph] = plotMultichannel(trialtime,lpres, ...
            'ylabels',channelnames,'plothandle',h,'color',ctemp(1,:),...
            'fontsize',tickfontsize,'linewidth',1.5);
        for idx=1:length(ph)
            set(ph{idx},'LineStyle','--');
        end
        
        % set the figure window's size on the screen
        set(h,'Units','Normalized','Position',[0.1 0.1 0.7 0.7]);
        set(gca,'Position',figpos);
        xl = xlabel(gca,'Time (s)','FontSize',tickfontsize);
        xl.Position(2) = xl.Position(2)+0.1;
        leg = legend([{'EMG Data'} {'FLV-Con Muscle'} {'Ideal Muscle'}],'FontSize',legfontsize);
        p = get(leg,'Position');
        % amending position of the legend - can be configured in legoffset
        set(leg,'Position',p+legoffset);
        title([{'EMG data and FLV Constrained      '} ...
            {'Muscle Activations       '}],'FontSize',titlefontsize);
        
        if(savefile==1)
            set(h,'Units','Inches');
            screenposition = get(h,'Position');
            set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',screenposition(3:4));
            print(h,['Analysis_test1' num2str(j)],'-dpdf','-r300');            
        end
    end
end