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

for i = 1 : length(stateSpace)
    % for each row in stateSpace, determine coordinates of 4 possible moves.
    k = stateSpace(i,:) ; % get n,m map values for a given state
    
    
    
    % determine if adjacent squares are in the stateSpace. 
    
    %n_ind is the next state if you choose north
    [move_north, n_ind] = ismember([k(1), k(2)+1], stateSpace, 'rows');
    if move_north   
        pij(i, n_ind, 1) = 1 ;
    else
        n_ind = i;
        pij(i,n_ind,1) = 1 ;
    end 
    
    
    
    [move_south, s_ind] = ismember([k(1), k(2)-1], stateSpace, 'rows');
    if move_south  
        pij(i, s_ind, 3) = 1 ;
    else
        pij(i,i,3) = 1 ;
    end
    
    [move_east, e_ind] = ismember([k(1)+1, k(2)], stateSpace, 'rows');
    if move_east  
        pij(i, e_ind, 4) = 1 ;
    else
        pij(i,i,4) = 1 ;
    end
    
    [move_west, w_ind] = ismember([k(1)-1, k(2)], stateSpace, 'rows');
    if move_west
        pij(i, w_ind, 2) = 1 ;
    else
        pij(i,i,2) = 1 ;
    end
    
    pij(i, i, 5) = 1; % if we take a picture, we stay at the same state i. 
    
    % calculate p(caught) for each possible movement
    
end 

end

function los = lineOfSight(cameras, current_state) 
%{
lineOfSight accepts cameras matrix and outputs a binary matrix that
indicates which cameras are in line of sight of each of the 4 squares (n,
s, e, w) around the current state. 

returns a 4x2 matrix
 
%}



end

function p_caught = probabilityCaught(cameras, stateSpace, current_state) 
%{
    given the cameras, and the stateSpace (ie spaces cameras can see
    through) calculate the probability of being caught in one timestep in a
    given state
%}

P_nc = 1;

current_coordinates = stateSpace(current_state,:);

% walk north until wall, check if a camera
walking_state = current_coordinates
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
walking_state = current_coordinates
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
walking_state = current_coordinates
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
walking_state = current_coordinates
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



p_caught = 1-P_nc ;

end 





