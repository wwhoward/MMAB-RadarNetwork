classdef MC < handle
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
        T0
        phase
        fixed
        bestM
        delta = 0.1
        B
        t
        M % estimate of n_players
        
    end
    
    methods
        function obj = MC(n_actions, ID, n_players, varargin)
            %MC Construct an instance of this class
            %   Detailed explanation goes here
            obj = strat_constructor(obj, n_actions, ID, n_players, varargin); 
            
            obj.t = 0; 
            obj.B = inf*ones([n_actions, 1]); 
            obj.M = 1; 
            obj.T0 = 1000; 
            obj.phase = "exploration"; 
            obj.fixed = -1; 
            obj.bestM = []; 
        end
        
        function action = play(obj)
            if obj.phase == "exploration"
                action = randi(obj.n_actions); 
            elseif obj.phase == "fixation"
                action = randsample(obj.bestM, 1); 
            else 
                action = obj.fixed; 
            end
            
            obj.Acts(end+1) = action; 
            obj.lastAction = action; 
        end
        
        function [] = update(obj, obs, rews)
            obj = strat_updater(obj, obs, rews); 
            
            obj.t = obj.t + 1; 
            if obj.phase == "exploration"
                if obj.t >= obj.T0
                    obj.phase = "fixation"; 
                    obj.M = round(log((obj.t - obj.CumCols)/obj.t)/(log(1-1/obj.n_actions))) + 1; 
                    [~, obj.bestM] = mink(-obj.Means, obj.M); 
                end
            elseif obj.phase == "fixation"
                if not(obj.lastCollision)
                    obj.phase = "exploitation"; 
                    obj.fixed = obj.lastAction; 
                end
            end
        end
    end
end

