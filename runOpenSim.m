function [nump,numt] = runOpenSim(trialfoldername,modelname,options)
% RUNOPENSIM Runs Opensim model through Inverse Kinematics,
% Kinematic Analysis, Inverse Dynamics, and static optimization
% (native MATLAB fmincon) to obtain muscle activations
% 
%     [numparticipants,numdatatrials] = runOpenSim(X,N)
% 
%     Arguments:
%     X is the name of the folder where all trials MUST BE arranged into
%     the following structure (--> indicate folder)
%         --> X
%           <participant01prefix>_tpose_marker.trc
%           <participant02prefix>_tpose_marker.trc
%           <participant03prefix>_tpose_marker.trc
%           <participant04prefix>_tpose_marker.trc
%           --> <participant01prefix>
%              <prefix_trialx>.trc
%              <prefix_trialx>_force.mot
%           --> <participant02prefix>
%              <prefix_trialx>.trc
%              <prefix_trialx>_force.mot
%           --> <participant03prefix>
%              <prefix_trialx>.trc
%              <prefix_trialx>_force.mot
%           --> <participant04prefix>
%              <prefix_trialx>.trc
%              <prefix_trialx>_force.mot
% 
% 	  NOTE: a folder will be created for each trial in the respective
% 	  participant's folder which contains all the outputs.
%     NOTE2: Scaling model can be done using opensim GUI, and runOpenSim
%     can be performed using the scaled model (osim file).
% 
%     N is the filename of the model (must be an opensim .osim file format in the folder where you run this function)
% 
%     [nump,~] = runOpenSim(... , options) specifies other parameters for
%     opensim operation. Parameters are:
%     'statoptmaxiterations' - the maximum number of iterations for
%                              static optimization (default 20)
%     'lowpassfreq'          - frequency of the 4th order lowpass filter
%                              (default 10)
%     'removeforce'          - option to remove unecessary forces. 1 for true,
%                              0 for false. Requires your own
%                              removeUnnecessaryForces function (default 1)
%     'optimsampledt'        - the interval between frames when performing
%                              fmincon in static optimization. A value of 
%                              2 will mean that every second frame is 
%                              calculated. (default 1)
%     'momentarmmethod'      - method to calculate moment arms. 1 uses
%                              default opensim computeMomentArm for each
%                              muscle, 2 uses generalized forces method.
%                              (default 1)
%     'forcegenmethod'       - force generator type. 'ideal' assume ideal
%                              generator with only max isometric force. 
%                              'flv-curve' uses force length velocity curve
%                              constraints when calculating force output.
%                              (default 'ideal')
%     'forcebodyname'        - the body name which the force is exerted on
%                              (default 'lunate')
%     'forceexpressbody'     - applied forces expressed in which body frame
%                              (default 'lunate')
%     'pointexpressbody'     - applied point expressed in which body frame
%                              (default 'lunate')
%     'forceidentifier'      - prefix for forces in force mot file
%                              (default 'force_')
%     'pointidentifier'      - prefix for points in force mot file
%                              (default 'point_')
%     'torqueidentifier'     - prefix for torques in force mot file
%                              (default '')
% 
%     Example:
%         % Example for forces applied in different body and point of force
%         is expressed in a different reference frame
%         nump =
%         runOpenSimCMC('trialdata', 'opensimmodel.osim', 'forcebodyname', ...
%                       'thorax', 'forceexpressbody', 'ground', ...
%                       'pointexpressbody','ground')
% 
%     For more information about opensim, refer to <a
%     href="https://simtk-confluence.stanford.edu:8443/display/OpenSim/OpenSim+Documentation">opensim wiki.</a>
%     For more information about OpenSim API, refer to <a
%     href="https://simtk.org/api_docs/opensim/api_docs/md_doc_APIGuide.html">opensim API guide.</a>
% 
arguments
    trialfoldername {mustBeText}
    modelname {mustBeText}
    options.statoptmaxiterations {mustBeNumeric,mustBeNonnegative,mustBeNonzero}=20
    options.lowpassfreq          {mustBeNumeric,mustBeNonnegative}=10
    options.removeforce          {mustBeNumeric,mustBeNonnegative}=1
    options.optimsampledt        {mustBeNumeric,mustBeNonnegative,mustBeNonNan,mustBeNonzero}=1
    options.momentarmmethod      {mustBeNumeric,mustBeNonnegative,mustBeNonNan,mustBeNonzero}=1
    options.forcegenmethod       {mustBeText}='ideal'
    options.forcebodyname        {mustBeText}='lunate'
    options.forceexpressbody     {mustBeText}='lunate'
    options.pointexpressbody     {mustBeText}='lunate'
    options.forceidentifier      {mustBeText}='force_'
    options.pointidentifier      {mustBeText}='point_'
    options.torqueidentifier     {mustBeText}=''
end
import org.opensim.modeling.*
addpath('Utilities')

% checking arguments are sane/valid
if(~strcmp(options.forcegenmethod,'ideal')&&~strcmp(options.forcegenmethod,'flv-curve'))
    disp('Incorrect parameter for forcegenmethod. Default value used instead.');
    options.forcegenmethod = 'ideal';
end
if( options.removeforce~=1 && options.removeforce~=0)
    disp('Incorrect parameter for removeforce. Default value used instead.');
    options.removeforce = 1;
end
if( options.momentarmmethod~=1 && options.momentarmmethod~=2)
    disp('Incorrect parameter for momentarmmethod. Default value used instead.');
    options.momentarmmethod = 1;
end

% find participant subfolders for trial optitrack + force data
numt = 0;
pfold = dir(fullfile(pwd,trialfoldername));
pfold = pfold([pfold.isdir]==1);
pfold = pfold(3:end); nump = numel(pfold);

for i=1:numel(pfold) % for each participant
    modelfile = fullfile(pwd,modelname);
    model = Model(modelfile);
    
    % precursor calcs for use throughout participant which are static
    muscset = model.getMuscles();
    msize = muscset.getSize();
    muscnames = cell(msize,1);
    maxforces = zeros(msize,1);
    for idx = 1:msize
        temp = Millard2012EquilibriumMuscle.safeDownCast(muscset.get(idx-1));
        maxforces(idx) = temp.getMaxIsometricForce();
        muscnames{idx} = temp.toString();
    end
    coordset = model.getCoordinateSet();
    csize = coordset.getSize();
    cset = repmat(Coordinate(),csize,1);
    for ci=1:csize
        cset(ci) = coordset.get(ci-1);
    end
    
    % subdir for each trial
    trcdir = dir(fullfile(pfold(i).folder,pfold(i).name,'*.trc'));
    forcedir = dir(fullfile(pfold(i).folder,pfold(i).name,'*_force.mot'));
    if(length(trcdir) ~= length(forcedir))
        error(['Need the same number of external force files as trc files. Check participant ' num2str(i) '.']);
    end
    numt = numt + numel(trcdir);
    
    for j=1:numel(trcdir) % for each optitrack trial
        % filenames
        trcname = fullfile(trcdir(j).folder,trcdir(j).name);
        resfolder = fullfile(trcdir(j).folder,trcdir(j).name(1:end-4));
        resname = fullfile(resfolder,trcdir(j).name(1:end-4));
        mkdir(resfolder);
        iktoolname = [trcdir(j).name(1:end-4) '_iktool'];
        ikmotfile = [resname '_ikres.mot'];
        iksetupname = [resname '_IKsetup.xml'];
        kintoolname = trcdir(j).name(1:end-4);
        kinqfile = [resname '_Kinematics_q.sto'];
        kinufile = [resname '_Kinematics_u.sto'];
        kintoolsetupname = [resname '_kinematicssetup.xml'];
        extforcename = fullfile(forcedir(j).folder,forcedir(j).name);
        extloadname = [resname '_extload.xml'];
        idtoolname = [trcdir(j).name(1:end-4) '_idtool'];
        idstoname = [trcdir(j).name(1:end-4) '_idres.sto'];
        idstofile = [resname '_idres.sto'];
        idsetupname = [resname '_IDsetup.xml'];
        staticoptidealfile = [resname '_SOidealMuscleRes.mat'];
        staticoptflvfile = [resname '_SOflvMuscleRes.mat'];

        trcdata = MarkerData(trcname);
        initTime = trcdata.getStartFrameTime();
        endTime = trcdata.getLastFrameTime();

        % IK
        if(~isfile(ikmotfile))
            ikTool = InverseKinematicsTool();
            ikTool.setModel(model);
            ikTool.setName(iktoolname);
            ikTool.setMarkerDataFileName(trcname);
            ikTool.setStartTime(initTime);
            ikTool.setEndTime(endTime);
            ikTool.setOutputMotionFileName(ikmotfile);
            ikTool.setResultsDir(resfolder);
            ikTool.print(iksetupname);
            ikTool.run();            
        end
        
        % Kinematics analysis
        si = model.initSystem();
        if(strcmp(options.forcegenmethod,'flv-curve') && ~isfile(kinqfile))
            anTool = AnalyzeTool(model);
            anTool.setName(kintoolname);
            anTool.setCoordinatesFileName(ikmotfile);
            anTool.loadStatesFromFile(si);
            anTool.setStartTime(initTime);
            anTool.setFinalTime(endTime);
            anSet = anTool.getAnalysisSet();
            kinAnalysis = Kinematics();
            anSet.cloneAndAppend(kinAnalysis);
            anTool.addAnalysisSetToModel();
            anTool.setResultsDir(resfolder);
            anTool.print(kintoolsetupname);
            anTool.run();
        end
        
        % ID
        if(~isfile(idstofile))
            idTool = InverseDynamicsTool();
            idTool.setModel(model);
            a = ArrayStr(); a.append('Muscles');
            idTool.setExcludedForces(a);
            extForce = ExternalForce(Storage(extforcename));
            extForce.setName('externalforce');
            extForce.setAppliedToBodyName(options.forcebodyname);
            extForce.setForceExpressedInBodyName(options.forceexpressbody);
            extForce.setPointExpressedInBodyName(options.pointexpressbody);
            extForce.setForceIdentifier(options.forceidentifier);
            extForce.setPointIdentifier(options.pointidentifier);
            extForce.setTorqueIdentifier(options.torqueidentifier);
            extLoad = idTool.updExternalLoads;
            extLoad.cloneAndAppend(extForce);
            extLoad.setDataFileName(extforcename);
            extLoad.print(extloadname);

            idTool.setName(idtoolname);
            idTool.setStartTime(initTime);
            idTool.setEndTime(endTime);
            idTool.setCoordinatesFileName(ikmotfile);
            idTool.setOutputGenForceFileName(idstoname);
            idTool.setExternalLoadsFileName(extloadname);
            idTool.setResultsDir(resfolder);
            idTool.print(idsetupname);
            idTool.run();
        end

        % kinematics coordinates - kinqfile
        % kinematics velocities - kinufile
        % id forces - idstofile
        if(strcmp(options.forcegenmethod,'flv-curve'))
            [coords,coordnames,~] = loadFilterCropArray(kinqfile, ...
                options.lowpassfreq,[initTime endTime]);
            [speeds,~,~] = loadFilterCropArray(kinufile, ...
                options.lowpassfreq,[initTime endTime]);            
        else
            [coords,coordnames,~] = loadFilterCropArray(ikmotfile, ...
                options.lowpassfreq,[initTime endTime]);            
        end
        [forces,forcenames,~] = loadFilterCropArray(idstofile, ...
            options.lowpassfreq,[initTime endTime]);
        
        % realign column names to be aligned between force and coords
        force2coord = zeros(length(coordnames),1);
        for fidx=1:length(forcenames)
            forcename = forcenames{fidx};
            for fcoord=1:length(coordnames)
                coordname = coordnames{fcoord};
                if contains(forcename, '_moment')
                    forcename = forcename(1:end-7);
                elseif contains(forcename, '_force')
                    forcename = forcename(1:end-6);
                end
                if strcmp(forcename,coordname)
                    force2coord(fcoord) = fidx;
                end
            end
        end
        forces = forces(:,force2coord);
        forcenames = forcenames(force2coord);
        
        % remove any unnecessary forces
        if (options.removeforce==1)
            [nforce,~,~,unidx] = removeUnnecessaryForces(forces,forcenames,coordnames);
        else
            unidx = (1:length(coordnames))';
        end
        
        % disable activation dynamics if ideal force generators used
        if(strcmp(options.forcegenmethod,'ideal'))
            muscset = model.updMuscles();
            for midx = 1:msize
                temp = Millard2012EquilibriumMuscle.safeDownCast(muscset.get(midx-1));
                temp.setIgnoreActivationDynamics(si,true);
            end
            si = model.initSystem();
        end
        
        % fmincon to do stat opt (q, u, and forces at each q for each sample step)
        nframes = size(coords,1);
        coords = coords(1:options.optimsampledt:nframes,:);
        nforce = nforce(1:options.optimsampledt:nframes,:);
        nframes = size(coords,1);
        if(strcmp(options.forcegenmethod,'flv-curve'))
            speeds = speeds(1:options.optimsampledt:size(speeds,1),:);
        end
        
        % static opt options
        staticopt = optimoptions('fmincon','MaxIterations', ...
            options.statoptmaxiterations,'Display','none');
        costfcn = @(x) sum(x.^2); % cost function
        
        % init guess and bound arrays
        actRes = zeros(nframes,msize);
        initAct = zeros(msize,1)+0.1; % init guess of activation
        lowerbound = 0.05*ones(size(initAct));
        upperbound = 1*ones(size(initAct));
        
        for idx = 1:nframes
            tic;
            disp(['Optimizing time step ' num2str(idx) ' of ' num2str(nframes)]);
            % realize state
            if(strcmp(options.forcegenmethod,'flv-curve'))
                tmpQ = Vector();
                tmpQ.resize(csize);
                tmpU = Vector();
                tmpU.resize(coordset.getSize());
                for vidx = 1:csize
                    tmpQ.set(vidx-1,coords(idx,vidx));
                    tmpU.set(vidx-1,speeds(idx,vidx));
                end
                si.setQ(tmpQ);
                si.setU(tmpU);
                model.realizeDynamics(si);
            else
                tmpQ = Vector();
                tmpQ.resize(csize);
                for vidx = 1:csize
                    tmpQ.set(vidx-1,coords(idx,vidx));
                end
                si.setQ(tmpQ);
                model.realizePosition(si);
            end
            
            % (optional) setup inequality constraints
            A = []; B = [];
            
            if(strcmp(options.forcegenmethod,'flv-curve'))
                % take into account force-length-velocity properties
                dynForces = calcForcewDynamics(model,si,maxforces);
                muscForces = repmat(dynForces',size(nforce,2),1);
                % to investigate: PECM1-3 and LAT1 provide super high muscle forces [need to investigate with actRes output]
                % hotfix: if ratio of force vs max isometric force > 3, use max isometric force since it could be caused by singularity (infinite force generation)
            else
                % ideal muscle force generators
                muscForces = repmat(maxforces',size(nforce,2),1);                
            end
            
            % calc moment arms
            momentarms = calcMomentArms(model,si,cset,unidx,options.momentarmmethod);
            
            % setup equalities
            Aeq = muscForces.*momentarms;
            Beq = nforce(idx,:)';
            
            % call fmincon
            x = fmincon(costfcn,initAct,A,B,Aeq,Beq,lowerbound,upperbound,[],staticopt);
            
            % store solution - set guess for next sample
            actRes(idx,:) = x';
%             initAct = x; % current be next guess since assuming dynamics don't change as much from one frame to another
            disp(['Optimizing time step ' num2str(idx) ' time taken: ' num2str(toc) ' seconds.']);
        end
        
        % save activation results
        if(strcmp(options.forcegenmethod,'flv-curve'))
            save(staticoptflvfile,'actRes','coordnames','unidx');
        else
            save(staticoptidealfile,'actRes','coordnames','unidx');            
        end
    end
end
end