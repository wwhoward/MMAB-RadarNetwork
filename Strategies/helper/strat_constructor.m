function [strat] = strat_constructor(strat, n_actions, ID, n_players, varargin)
%STRAT_CONSTRUCTOR Summary of this function goes here
%   Detailed explanation goes here
strat.n_actions = n_actions;
strat.ID = ID;
strat.n_players = n_players;

if any(strcmp(varargin, 'delay'))
    delay = varargin(find([varargin{:}] == 'delay')+1);
    strat.delay = delay{1};
else
    strat.delay = 0; 
end

strat.Actions = 1:n_actions;
strat.lastAction = randi(n_actions);
strat.lastCollision = 0;
strat.Means = zeros([n_actions, 1]);
strat.n_pulls = zeros([n_actions, 1]);
strat.Rews = zeros(n_actions, 5000); 

end

