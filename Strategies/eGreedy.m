classdef eGreedy < handle
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
        epsilon = 0.1
    end
    
    methods
        function obj = eGreedy(n_actions, ID, n_players, varargin)
            %MC_TOPM Construct an instance of this class
            %   Detailed explanation goes here
            obj = strat_constructor(obj, n_actions, ID, n_players, varargin); 
            if any(strcmp(varargin, 'epsilon'))
                obj.epsilon = varargin{find(strcmp(varargin, 'epsilon')==1)+1};
            end
        end
        
        function action = play(obj)
            if rand > obj.epsilon
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

