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

state_type=zeros(length(stateSpace),1);

for state = 1 : length(stateSpace)
    % check if the state is the gate, a mansion, or a camera:
    if map(stateSpace(state, 2),stateSpace(state, 1)) < 0 
        state_type(state,1) = c_water ;
    else 
        state_type(state,1) = c_land ;
    end
    
end

cost_space = zeros(length(stateSpace), length(controlSpace)) ;

% find the index value of the gate, gate_state: 
[oooo,gate_state]=ismember(gate,stateSpace,'rows');

% find the costs for each action 
p_successful_picture = 0.5 ; 

for state = 1 : length(cost_space) 
    
    %north
    [tf,next_ind]=ismember([stateSpace(state,1), stateSpace(state,2)+1], stateSpace, 'rows');
    if tf
        p_caught = trans_probs(state, gate_state, 1);
        p_not_caught = 1 - p_caught ; 
        cost_space(state, 1) = state_type(next_ind,1)*p_not_caught + p_caught*c_gate ;
    else
        cost_space(state, 1) = inf;
    end
    %south
    [tf,next_ind]=ismember([stateSpace(state,1), stateSpace(state,2)-1], stateSpace, 'rows');
    if tf
        p_caught = trans_probs(state, gate_state, 3);
        p_not_caught = 1 - p_caught ; 
        cost_space(state, 3) = state_type(next_ind,1)*p_not_caught + p_caught*c_gate  ;
    else
        cost_space(state, 1) = inf;
    end
    %east
    [tf,next_ind]=ismember([stateSpace(state,1)+1, stateSpace(state,2)], stateSpace, 'rows');
    if tf
        p_caught = trans_probs(state, gate_state, 4);
        p_not_caught = 1 - p_caught ; 
        cost_space(state, 4) = state_type(next_ind,1)*p_not_caught + p_caught*c_gate  ;
    else
        cost_space(state, 1) = inf;
    end
    %west
    [tf,next_ind]=ismember([stateSpace(state,1)-1, stateSpace(state,2)], stateSpace, 'rows');
    if tf
        p_caught = trans_probs(state, gate_state, 2);
        p_not_caught = 1 - p_caught ; 
        cost_space(state, 2) = state_type(next_ind,1)*p_not_caught + p_caught*c_gate ;
    else
        cost_space(state, 1) = inf;
    end
    %click a pic
    p_caught = trans_probs(state, gate_state, 5);
    p_not_caught = 1 - p_caught ; 
    % this cost_space 
    cost_unsuccessful_pic = c_land*p_not_caught + p_caught*c_gate ;
    
    % cost of taking a pic
    p_successful_pic = 0.5/ distanceToMansion(mansion,stateSpace, current_state) ;
    
    cost_space(state, 5) = p_successful_pic*0 + (1 - p_successful_pic)*cost_unsuccessful_pic ;
        
    
    
end 

%cell to the north of gate
[tf,gate_n]=ismember([stateSpace(gate_state,1), stateSpace(gate_state,2)+1], stateSpace, 'rows');
if tf
    p_not_caught = probabilityNotCaught(cameras, stateSpace, gate_n) ;
    p_caught = 1 - p_not_caught ;
    cost_space(gate_n,3)=state_type(gate_state,1)*p_not_caught+p_caught*c_gate;
end
%cell to the south of gate
[tf,gate_s]=ismember([stateSpace(gate_state,1), stateSpace(gate_state,2)-1], stateSpace, 'rows');
if tf
    p_not_caught = probabilityNotCaught(cameras, stateSpace, gate_s) ;
    p_caught = 1 - p_not_caught ;
    cost_space(gate_s,1)=state_type(gate_state,1)*p_not_caught+p_caught*c_gate;
end
%cell to the west  of gate
[tf,gate_w]=ismember([stateSpace(gate_state,1)-1, stateSpace(gate_state,2)], stateSpace, 'rows');
if tf
    p_not_caught = probabilityNotCaught(cameras, stateSpace, gate_w) ;
    p_caught = 1 - p_not_caught ;
    cost_space(gate_w,4)=state_type(gate_state,1)*p_not_caught+p_caught*c_gate;
end
%cell to the east  of gate
[tf,gate_e]=ismember([stateSpace(gate_state,1)+1, stateSpace(gate_state,2)], stateSpace, 'rows');
if tf
    p_not_caught = probabilityNotCaught(cameras, stateSpace, gate_e) ;
    p_caught = 1 - p_not_caught ;
    cost_space(gate_e,2)=state_type(gate_state,1)*p_not_caught+p_caught*c_gate;
end
%stand at agte cell and click a pic

    p_not_caught = probabilityNotCaught(cameras, stateSpace, gate_state) ;
    p_caught = 1 - p_not_caught ;
    cost_space(gate_state,5)=c_land*p_not_caught+p_caught*c_gate;

% for cells around the gate, if we apply a control action that moves us
% into the gate cell, the cost should be 1 (not 6). 


G = cost_space ; 
    
end



function distance = distanceToMansion(mansion,stateSpace, current_state) 
%{
Calculates the distance to mansion from a state
%}
distance = inf;

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

[a, camera_row] = ismember(walking_state, mansion, 'rows');

if a
    distance = min(distance, abs(current_coordinates(2) - walking_state(2)));
end


% walk west until wall, check if a camera
walking_state = current_coordinates;
open = 1;
%while we can still walk...
while open
   % walking wests
   walking_state = walking_state + [-1,0];
   % check if we've hit a wall
   [open, c] = ismember(walking_state, stateSpace, 'rows');
end

[a, camera_row] = ismember(walking_state, mansion, 'rows');

if a
    distance = min(distance, abs(current_coordinates(1) - walking_state(1)));
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

[a, camera_row] = ismember(walking_state, mansion, 'rows');

if a
    distance = min(distance, abs(current_coordinates(2) - walking_state(2)));
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

[a, camera_row] = ismember(walking_state, mansion, 'rows');

if a
    distance = min(distance, abs(current_coordinates(1) - walking_state(1)));
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



