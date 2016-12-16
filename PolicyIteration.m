function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% put your code here


policy_iteration = 1;
[m,n] = size(G);
J_old = zeros(m);
u_opt_ind = ones(m,1)*5; %literally just go north for everything

while policy_iteration < 1000
    
    
    u_old = u_opt_ind;
    
    for i = 1:m
        for j = 1:m
            pij(i,j) = P(i,j, u_opt_ind(i));
        end
        g(i,1) = G(i,u_opt_ind(i));
    end
    
    J = inv(eye(m) - pij)*g;
    
    
    for i = 1:m
        temp_Ji = zeros(1,5);
        for u = 1:n
            sum = 0;
            for j = 1:m
                sum = sum + P(i,j,u)*J(j);
            end
            temp_Ji(u) = G(i,u) + sum;
        end
        [J(i), u_opt_ind(i)] = min(temp_Ji);
    end

    if isequal(u_old, u_opt_ind)
        break;
    end
    
    policy_iteration = policy_iteration+1
end

J_opt = J;


end

