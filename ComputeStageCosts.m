function G = ComputeStageCosts( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, controlSpace, map, gate, mansion,
%   cameras) computes the stage costs for all states in the state space
%   for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       controlSpace:
%           A (L x 1)-matrix, where the l-th row represents the l-th
%           element of the control space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map.
%           Positive values indicate cells that are inaccessible (e.g.
%           trees, bushes or the mansion) and negative values indicate
%           ponds or pools.
%
%   	gate:
%          	A (1 x 2)-matrix describing the position of the gate.
%
%    	mansion:
%          	A (F x 2)-matrix indicating the position of the cells of the
%           mansion.
%
%    	cameras:
%          	A (H x 3)-matrix indicating the positions and quality of the 
%           cameras.
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.

% put your code here

% stage cost = ExpectedValue [P(i->t, 

c_gate = 6 ; 
c_terminal = 1 ; 
c_water = 4 ; 
c_land = 1 ; 
c_mansion = inf ;
c_camera = inf ;
c_wall = inf ;

% get transition probabilities
trans_probs = ComputeTransitionProbabilities( stateSpace, controlSpace, map, gate, mansion, cameras ) ;

% g(i,u) = P(i,u,terminal)*c_terminal + P(i,u,
%  for each state, determine the type of squares for n,s,w,e and current
%  square (should be land or water. if type of square is mansion, gate, or camera,
%  assign to land). then the cost is P_terminal*c_terminal +
%  P_i*c_current_type + P_j*c_jtype... 

% determine whether each state land or pond states.  
% cost_space assigns a cost to being in each square (either land cost or pond cost)
% the expected cost is P(being in state)*cost_of_being_in_state

for state = 1 : length(stateSpace)
    % check if the state is the gate, a mansion, or a camera:
    if map(stateSpace(state)) < 0 
        state_type(state) = c_water ;
    else 
        state_type(state) = c_land ;
    end
    
end

cost_space = zeros(length(stateSpace), length(controlSpace)) ;

[oooo,gate_state]=ismember(gate,stateSpace,'rows');

for state = 1 : length(cost_space) 
    
    %north
    [tf,next_ind]=ismember([stateSpace(state,1), stateSpace(state,2)+1], stateSpace, 'rows');
    if tf
        cost_space(state, 1) = state_type(next_ind)*p_not_caught + p_caught*c_gate ;
    else
        cost_space(state, 1) = inf;
    end
    %south
    [tf,next_ind]=ismember([stateSpace(state,1), stateSpace(state,2)-1], stateSpace, 'rows');
    if tf
        p_caught = trans_probs(stateSpace(state), stateSpace(gate_state), 1);
        p_not_caught = 1 - p_caught ; 
        cost_space(state, 3) = state_type(next_ind)*p_not_caught + p_caught*c_gate  ;
    else
        cost_space(state, 1) = inf;
    end
    %east
    [tf,next_ind]=ismember([stateSpace(state,1)+1, stateSpace(state,2)], stateSpace, 'rows');
    if tf
        p_caught = trans_probs(stateSpace(state), stateSpace(gate_state), 1);
        p_not_caught = 1 - p_caught ; 
        cost_space(state, 4) = state_type(next_ind)*p_not_caught + p_caught*c_gate  ;
    else
        cost_space(state, 1) = inf;
    end
    %west
    [tf,next_ind]=ismember([stateSpace(state,1)-1, stateSpace(state,2)], stateSpace, 'rows');
    if tf
        p_caught = trans_probs(stateSpace(state), stateSpace(gate_state), 1);
        p_not_caught = 1 - p_caught ; 
        cost_space(state, 2) = state_type(next_ind)*p_not_caught + p_caught*c_gate ;
    else
        cost_space(state, 1) = inf;
    end
    %click a pic
    p_caught = trans_probs(stateSpace(state), stateSpace(gate_state), 1);
    p_not_caught = 1 - p_caught ; 
    cost_space(state, 5) = state_type(next_ind)*p_not_caught + p_caught*c_gate ;
    
    % find all cells around the gate 
    if stateSpace(state) == gate
        p_not_caught = probabilityNotCaught(cameras, stateSpace, current_state) ;
        p_caught = 1 - p_not_caught ;
    else 
        p_caught = trans_probs(stateSpace(state), stateSpace(gate_state), 1);
        p_not_caught = 1 - p_caught ; 
    end 
    
end 



    
end



function P_nc = probabilityNotCaught(cameras, stateSpace, current_state) 
%{
    given the cameras, and the stateSpace (ie spaces cameras can see
    through) calculate the probability of not being caught in one timestep in a
    given state
%}

P_nc = 1;

current_coordinates = stateSpace(current_state,:);

% walk north until wall, check if a camera
walking_state = current_coordinates;
open = 1;
%while we can still walk...
while open
   % walking north
   walking_state = walking_state + [0,1];
   % check if we've hit a wall
   [open, c] = ismember(walking_state, stateSpace, 'rows');
end

[a, camera_row] = ismember(walking_state, cameras(:,1:2), 'rows');

if a
    cameracatch = cameras(camera_row, 3)/abs(current_coordinates(2) - walking_state(2));
    P_nc = P_nc * (1 - cameracatch);
end

% walk west until wall, check if a camera
walking_state = current_coordinates;
open = 1;
%while we can still walk...
while open
   % walking west
   walking_state = walking_state + [-1,0];
   % check if we've hit a wall
   [open, c] = ismember(walking_state, stateSpace, 'rows');
end

[a, camera_row] = ismember(walking_state, cameras(:,1:2), 'rows');

if a
    cameracatch = cameras(camera_row, 3)/abs(current_coordinates(1) - walking_state(1));
    P_nc = P_nc * (1 - cameracatch);
end

% walk south until wall, check if a camera
walking_state = current_coordinates;
open = 1;
%while we can still walk...
while open
   % walking south
   walking_state = walking_state + [0,-1];
   % check if we've hit a wall
   [open, c] = ismember(walking_state, stateSpace, 'rows');
end

[a, camera_row] = ismember(walking_state, cameras(:,1:2), 'rows');

if a
    cameracatch = cameras(camera_row, 3)/abs(current_coordinates(2) - walking_state(2));
    P_nc = P_nc * (1 - cameracatch);
end


% walk east until wall, check if a camera
walking_state = current_coordinates;
open = 1;
%while we can still walk...
while open
   % walking east
   walking_state = walking_state + [1,0];
   % check if we've hit a wall
   [open, c] = ismember(walking_state, stateSpace, 'rows');
end

[a, camera_row] = ismember(walking_state, cameras(:,1:2), 'rows');

if a
    cameracatch = cameras(camera_row, 3)/abs(current_coordinates(1) - walking_state(1));
    P_nc = P_nc * (1 - cameracatch);
end

end 



