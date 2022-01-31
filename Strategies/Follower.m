classdef Follower < handle
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
        Followee
        myNode
        
    end
    
    methods
        function obj = Follower(n_actions, ID, n_players, varargin)
            %MC Construct an instance of this class
            %   Detailed explanation goes here
            obj = strat_constructor(obj, n_actions, ID, n_players, varargin); 
            
%             if any(strcmp(varargin{1}, 'Node'))
%                 Node = varargin{1}(find([varargin{1}{:}] == 'Node')+1); 
%                 obj.Followee = Node{1};
%             end
            obj.Followee = varargin{1}{2}; 
        end
        
        function action = play(obj, varargin)
            action = 0; 
            if obj.T > 1
                action = obj.Followee.Strategy{1}.Acts(obj.T-1); 
                % obj.myNode.Strategy{2}{action}.whatToPlay = obj.Followee.Strategy{2}{action}.Acts(end-1); 
                obj.myNode.Strategy{2}{action}.whatToPlay = obj.Followee.Acts(end,2); 
            end
            
            
            obj.T = obj.T+1;
            obj.Acts(end+1) = action; 
            obj.lastAction = action; 
        end
        
        function [] = update(obj, obs, rews)
            obj = strat_updater(obj, obs, rews); 
            
            
        end
    end
end

