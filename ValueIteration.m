function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
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

iteration = 1;
[m,n] = size(G);
J_old = zeros(m);

while iteration < 1000
    J = ones(m,1)*inf;
    for i = 1:m
        for u = 1:n
            sum = 0;
            for j = 1:m
                sum = sum + P(i,j,u)*J_old(j);
            end
            J(i) = min(J(i), G(i,u) + sum);
        end
    end
    J_old = J;
    iteration = iteration+1
    J(1)
end

u_opt_ind = zeros(m,1);
J = ones(m,1)*inf;
for i = 1:m
    for u = 1:n
        sum = 0;
        for j = 1:m
            sum = sum + P(i,j,u)*J_old(j);
        end
        if ( G(i,u) + sum) < J(i)
           u_opt_ind(i) = u;
        end
        J(i) = min(J(i), G(i,u) + sum);
    end
end

J_opt = J;

end