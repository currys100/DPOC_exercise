function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

[m,n] = size(G);
A = zeros(n*m,m);
I = [eye(m); eye(m); eye(m); eye(m); eye(m)];
for u = 1:5
    for i = 1:m
        if G(i,u) == inf
            g(m*(u-1) + i,1) = 1;
            I(m*(u-1)+i, :) = 0;
                A( m*(u-1) + i, :) = 0;
        else
            for j = 1:m
                A( m*(u-1) + i, j) = P(i,j,u);
            end
            g(m*(u-1) + i,1) = G(i,u);
        end
    end
end
    
A= [I-A];
c = -1*ones(m,1);

[J,val,a]= linprog(c, A, g);

sum(g<0);

J_opt = J;

u_opt_ind = 5*ones(1,m);

for i=1:m
    slack= zeros(n,1);
    for u= 1:n
       slack(u) = g(m*(u-1) + i) - A(m*(u-1) + i,:)*J;
    end
    [temp, opt_action] = min(slack);
    u_opt_ind(i) = opt_action;
end


end

