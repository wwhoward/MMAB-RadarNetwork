classdef MC_TopM < handle
    %MC_TOPM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Acts = zeros(1,5000) % Action history
        Cols = zeros(1,5000) % Collision history
        Rews = [] % Reward history
        vector_rewards = [] % Simplified reward representation
        Means = zeros(1,5000) % Current means
        
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
        T = 1; 
        
        % Strategy Specific
        bestM
        s = false
        C = false
        b
        B
        previous_b
        T_ = 1000
        
    end
    
    methods
        function obj = MC_TopM(n_actions, ID, n_players, varargin)
            %MC_TOPM Construct an instance of this class
            %   Detailed explanation goes here
            obj = strat_constructor(obj, n_actions, ID, n_players, varargin); 
            
            obj.bestM = 1:n_actions; 
            obj.B = inf*ones([n_actions, 1]); 
            obj.b = inf*ones([n_actions, 1]); 
            obj.previous_b = inf*ones([n_actions, 1]); 
            
            if any(strcmp(varargin, 'T_'))
                T_ = varargin(find([varargin{:}] == 'T_')+1); 
                obj.T_ = T_{1};
            end
        end
        
        function action = play(obj)
            if not(any(obj.bestM == obj.lastAction))
                population = intersect(obj.bestM, find(obj.previous_b <= obj.previous_b(obj.lastAction))); 
                action = population(randsample(length(population), 1)); 
                obj.s = false; 
            elseif (obj.C && not(obj.s))
                action = randsample(obj.bestM, 1); 
                obj.s = true; 
            else
                action = obj.lastAction; 
                obj.s = true; 
            end
            
            obj.Acts(obj.T) = action; 
            obj.lastAction = action; 
        end
        
        function [] = update(obj, obs, rew)
            obj = strat_updater(obj, obs, rew); 
            
            obj.B(obj.lastAction) = sqrt(log(obj.T_)/(2*obj.n_pulls(obj.lastAction))); 
            obj.previous_b = obj.b; 
            obj.b = obj.Means + obj.B; 
            [~, obj.bestM] = mink(-obj.b, obj.n_players);             
        end
    end
end

