classdef eDecaying < handle
    %MC_TOPM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Acts = [] % Action history
        Cols = [] % Collision history
        Rews = [] % Reward history
        Means = [] % Current means
        vector_rewards = []
        
        Actions = [] % Set of actions
        n_pulls = [] % Set of selections per action
        n_actions = [] % Measure of actions
        ID % Unique (to the network) identifier for this node
        
        n_players % Total players
        
        ActionHistory = []
        RewardHistory = []
        CollisionHistory = []
        CumCols = 0
        
        lastAction = []
        lastCollision = []
        
        delay
        T = 1
        
        % Unique
        epsilon_f = @(x) 1./(x.^0.8)
    end
    
    methods
        function obj = eDecaying(n_actions, ID, n_players, varargin)
            %MC_TOPM Construct an instance of this class
            %   Detailed explanation goes here
            obj = strat_constructor(obj, n_actions, ID, n_players, varargin); 
            if any(strcmp(varargin{1}{1}, 'epsilon_f'))
                obj.epsilon_f = varargin{1}{1}{find(strcmp(varargin{1}{1}, 'epsilon_f')==1)+1};
            end
        end
        
        function action = play(obj)
            epsilon = obj.epsilon_f(obj.T); 
            if rand > epsilon
                [~, action] = max(obj.Means); 
            else
                action = randi(obj.n_actions); 
            end
            obj.Acts(end+1) = action; 
            obj.lastAction = action; 
        end
        
        function [] = update(obj, obs, rew)
            obj = strat_updater(obj, obs, rew);           
        end
    end
end

