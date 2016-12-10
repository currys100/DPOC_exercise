function P = ComputeTransitionProbabilities( stateSpace, controlSpace, map, gate, mansion, cameras )
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace, map, gate,
%   mansion, cameras) computes the transition probabilities between all
%   states in the state space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability from
%           state i to state j if control input l is applied.

%{ 
Notes:

-- Pij depends on state x and control input u. 
-- Probabilities are static (for any given state i, the transitions to 
other squares or to the terminal state have the same probabilities); thus
this only needs to be calculated once. 

pseudo code: 

initialize Pij matrix using stateSpace and controlSpace (K x K x L)

for each state i: 
  determine if n, s, e, w squares are reachable
  -- for state i at (n,m), is (n+1, m) in statespace, etc. 
  if yes: 
     Pij = 1
  if no: 
     Pij = 0

calculate Pit (transition to terminal state):

determine if LineOfSight to mansion:
  if yes:
    determine distance to mansion
    P(good picture | line of sight to mansion) = 0.5 / distance to mansion
  if no:
    Pit = 0.001

calculate Pig (transition to gate):
-- determine p(capture) based on each camera.

%}

% initialize (K x K x L) matrix with zeros for transition probabilities
pij = zeros( length(stateSpace), length(stateSpace), length(controlSpace) );

% for each move action, determine if square is accessible. 

%The state of the the gate
[oooo,gate_state]=ismember(gate,stateSpace,'rows');

for i = 1 : length(stateSpace)
    % for each row in stateSpace, determine coordinates of 4 possible moves.
    k = stateSpace(i,:) ; % get n,m map values for a given state
    
    % determine if adjacent squares are in the stateSpace. 
    
    %n_ind is the next state if you choose north
    [move_north, n_ind] = ismember([k(1), k(2)+1], stateSpace, 'rows');
    if ~move_north   
        n_ind = i;
    end 
    
    P_nc = probabilityNotCaught(cameras, stateSpace, n_ind);
%    [lake,n_ind]=ismember(,'rows');
    pij(i,n_ind,1)=P_nc;
    
    if (map(stateSpace(n_ind))<0)
        pij(i,n_ind,1) = pij(i,n_ind,1)^4 ; 
    end
    
    pij(i,gate_state,1)=pij(i,gate_state,1)+1-pij(i,n_ind,1);
    
    %-----------
    
    [move_south, s_ind] = ismember([k(1), k(2)-1], stateSpace, 'rows');
    if ~move_south  
        s_ind = i;
    end
    
    P_nc = probabilityNotCaught(cameras, stateSpace, s_ind);
%    [lake,n_ind]=ismember(,'rows');
    pij(i,s_ind,3)=P_nc;
    
    if (map(stateSpace(s_ind))<0)
        pij(i,s_ind,3) = pij(i,s_ind,3)^4 ; 
    end
    
    pij(i,gate_state,3)=pij(i,gate_state,3)+1-pij(i,s_ind,3);
    
    %------------
    
    [move_east, e_ind] = ismember([k(1)+1, k(2)], stateSpace, 'rows');
    if ~move_east  
        e_ind = i;
    end
    
    P_nc = probabilityNotCaught(cameras, stateSpace, e_ind);
%    [lake,n_ind]=ismember(,'rows');
    pij(i,e_ind,4)=P_nc;
    
    if (map(stateSpace(e_ind))<0)
        pij(i,e_ind,4) = pij(i,e_ind,4)^4 ; 
    end
    
    pij(i,gate_state,4)=pij(i,gate_state,4)+1-pij(i,e_ind,4) ;
    
    %------------
    
    [move_west, w_ind] = ismember([k(1)-1, k(2)], stateSpace, 'rows');
    if ~move_west
        w_ind = i;
    end
    
    P_nc = probabilityNotCaught(cameras, stateSpace, w_ind);
%    [lake,n_ind]=ismember(,'rows');
    pij(i,w_ind,2)=P_nc;
    
    if (map(stateSpace(w_ind))<0)
        pij(i,w_ind,2) = pij(i,w_ind,2)^4 ; 
    end
    
    pij(i,gate_state,2)=pij(i,gate_state,2)+1-pij(i,w_ind,2);
    
    pij(i, i, 5) = 1; % if we take a picture, we stay at the same state i.
    
    %----------- Camera action ---------  
    distanceToMansion = distanceToMansion(mansion,stateSpace,i);
    P_success = max(0.001, 0.5/distanceToMansion);
    
    P_nc = probabilityNotCaught(cameras, stateSpace, i);
    
    pij(i,i,5) = (1-P_success)*P_nc;
    pij(i,gate_state,5) = pij(i,gate_state,5)+(1-P_success)*(1-P_nc);
    
end 

P = pij;

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





