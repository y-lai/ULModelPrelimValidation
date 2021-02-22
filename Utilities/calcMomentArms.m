function L = calcMomentArms(model,state,coordset,index,method)
% CALCMOMENTARMS Calculates the moment arms for static optimization. This
% is the equivalent to calculating the Jacobian matrix between the muscle
% forces and the joint torques.
% 
%     L = calcMomentArms(model,state,coordset,index)
% 
%     Arguments:
%     model     - the opensim model object
%     state     - the opensim state object (moment arms are calculated
%                 based on the model in this state)
%                 Note: the mode must at least be realized at the position level.
%     coordset  - a (1 x D) cell array for the names of the coordinates from the
%                 model (refer to loadFilterCropArray.m)
%     index     - a (1 x M) array of coordinate sets to consider for calculating
%                 moment arms. (Default will be all coordinates)
%     method    - the method of calculating the moment arms. Method 1 will
%                 use OpenSim's computeMomentArm function for each muscle
%                 and coordinate. Method 2 will use the generalized forces
%                 approach to obtain the moment arm matrix. (Default is 1)
% 
%     NOTE: currently method 2 does not work (WIP). Setting method value
%     to 2 will use method 1.
% 
arguments
    model
    state
    coordset
    index       {mustBeNumeric,mustBeNonNan,mustBeNonnegative}=[]
    method      {mustBeNumeric,mustBeNonNan,mustBeNonnegative}=1
end
% check optional variables
if(isempty(index))
    index = 1:length(coordset);
end
if(~((method==1) || (method==2)) )
    disp('Invalid value for method is used. Setting to default value of 1.');
    method = 1;
end

muscset = model.getMuscles();
switch method
    case 1
        L = method1(muscset,state,coordset,index);
    case 2
% debug - unable to find projectU function when calculating C in matlab bindings
%         L = method2(muscset,state,coordset,index);
        L = method1(muscset,state,coordset,index);
    otherwise
        error('Error in method sent. Please use 1 for opensim computeMomentArms or 2 for generalized forces method');
end
end

% obtain moment arms using compute moment arms individually
function momentarms = method1(muscset,si,coordset,index)
msize = muscset.getSize();
momentarms = zeros(length(index),msize);
% index relevant coords and muscles
for midx = 1:msize
    mtemp = muscset.get(midx-1);
    mtemp.computeEquilibrium(si);
    c = zeros(size(index));
    for vidx = 1:length(c)
        c(vidx) = mtemp.computeMomentArm(si,coordset(index(vidx)));
    end
    momentarms(:,midx) = c(index);
end
end

% generalized force method for moment arms
function momentarms = method2(muscset,si,coordset,index)
msize = muscset.getSize();

% calc coupling matrix C
C = zeros(length(coordset));
model.realizeInstance(si);
for i=1:length(coordset)
    tmp = si.updU();
    for tidx = 1:tmp.size
        tmp.set(tidx-1,0);
    end
    coordset(i).setSpeedValue(si,1);
    model.realizeVelocity(si);
    model.projectU(si,1e-6);
    c = si.getU(); spval = coordset(i).getSpeedValue(si);
    for cidx = 1:length(index)
        C(i,cidx) = c.get(index(cidx)-1)/spval;
    end
end

% body spatial force relationships
F = zeros(length(coordset),msize);
for i=1:msize
    f0 = Vector(); f1 = Vector();
    m = muscset.get(i-1);
    m.setActivation(si,0);
    model.realizeVelocity(si);
    m.equilibrate(si);
    fa0 = m.getForce(si);
    model.realizeDynamics(si);
    model.multiplyBySystemJacobianTranspose(si,model.getRigidBodyForces(si,Stage(7)),f0);
    f0 = f0 + model.getMobilityForces(si,Stage(7));
    
    m.setActivation(si,1);
    m.equilibrate(si);
    fa1 = m.getForce(si);
    model.realizeDynamics(si);
    model.multiplyBySystemJacobianTranspose(si,model.getRigidBodyForces(si,Stage(7)),f1);
    f1 = f1 + model.getMobilityForces(si,Stage(7));
    
    f = f1 - f0;
    for fidx = 1:length(coordset)
        F(fidx,i) = f.get(index(fidx)-1)/(fa1-fa0);
    end
end
% C is (csize,csize), F is (csize,msize)
momentarms = -(C*F);
end