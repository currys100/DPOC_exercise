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
    [move_north, n_ind] = ismember([k(1)-1, k(2)], stateSpace, 'rows');
    if move_north   
        pij(i, stateSpace(n_ind), 1) = 1 ;
    end 
    
    [move_south, s_ind] = ismember([k(1)+1, k(2)], stateSpace, 'rows');
    if move_south  
        pij(i, s_ind, 3) = 1 ;
    end
    
    [move_east, e_ind] = ismember([k(1), k(2)+1], stateSpace, 'rows');
    if move_east  
        pij(i, e_ind, 4) = 1 ;
    end
    
    [move_west, w_ind] = ismember([k(1), k(2)-1], stateSpace, 'rows');
    if move_west
        pij(i, w_ind, 2) = 1 ;
    end
    
    
end 

end




