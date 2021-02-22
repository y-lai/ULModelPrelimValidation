function forces = calcForcewDynamics(model,state,maxforces,ratio)
% CALCFORCEWDYNAMICS Calculates the computed forces given a specific state.
% To determine whether force-length-velocity constraints are taken into
% consideration, user needs to set the muscles to incorporate muscle
% activation dynamics using OpenSim API prior to this function. This 
% function relies on the OpenSim model having Millard2012EquilibriumMuscle 
% type muscles.
% 
%     Arguments:
%     model     - the opensim model object
%     state     - the opensim state object (to equilibriate the muscles,
%                 the model MUST be realized to the Dynamics stage)
%     maxforces - a (1 x M) array of the maximum isometric forces from the
%                 M muscles in the opensim model
%     ratio     - in cases where calculated fiber forces are above max
%                 isometric forces, the ratio determines whether to use 
%                 those values or rely on max isometric forces.
%                 (Default ratio value is 3)
% 
arguments
    model
    state
    maxforces   {mustBeNumeric,mustBeNonNan,mustBeNonnegative}
    ratio       {mustBeNumeric,mustBeNonNan,mustBeNonnegative}=3
end
import org.opensim.modeling.*
msize = length(maxforces); 
forces = zeros(msize,1);
muscset = model.getMuscles();
for midx = 1:msize
    temp = Millard2012EquilibriumMuscle.safeDownCast(muscset.get(midx-1));
    temp.setActivation(state,1);
    temp.computeFiberEquilibrium(state);
    if (temp.getFiberForce(state)/maxforces(midx) > ratio)
        forces(midx) = maxforces(midx);
    else
        forces(midx) = temp.getFiberForce(state);        
    end
end
end