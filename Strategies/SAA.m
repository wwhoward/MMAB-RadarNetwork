classdef SAA < handle
    %MC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Acts = [] % Action history
        Cols = [] % Collision history
        Rews = [] % Reward history
        vector_rewards = [] % Simplified reward representation
        Means = [] % Current means
        
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
        
        % Strategy Specific
        
    end
    
    methods
        function obj = SAA(n_actions, ID, n_players, varargin)
            %MC Construct an instance of this class
            %   Detailed explanation goes here
            obj = strat_constructor(obj, n_actions, ID, n_players, varargin); 
        end
        
        function action = play(obj)
            if obj.lastCollision(1)
                action = randsample(setdiff(1:obj.n_actions,obj.lastAction), 1); %randi(obj.n_actions); 
            else
                action = obj.lastAction; 
            end
            
            obj.Acts(end+1) = action; 
            obj.lastAction = action; 
        end
        
        function [] = update(obj, obs, rews)
            obj = strat_updater(obj, obs, rews); 
            obj.lastCollision = obs; 
            
            

        end
    end
end

