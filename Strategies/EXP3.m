classdef EXP3 < handle
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
        w
        gainEXP3
        gamma = 0.1
        pAct
        gainRWD
        p
    end
    
    methods
        function obj = EXP3(n_actions, ID, n_players, varargin)
            %MC_TOPM Construct an instance of this class
            %   Detailed explanation goes here
            obj = strat_constructor(obj, n_actions, ID, n_players, varargin); 
            
            obj.w = ones(1,obj.n_actions);
            obj.gainEXP3 = [];
            obj.gainRWD = zeros(1, 5000); 
            obj.pAct = ones(1,bands);
            if any(strcmp(varargin, 'gamma'))
                obj.gamma = varargin{find(strcmp(varargin, 'gamma')==1)+1};
            end
        end
        
        function action = play(obj)                
            [action, obj.p] = EXP3_RecommendArm(obj.w, obj.gamma, obj.pAct, obj.Actions); 
            if isempty(action)
                action = randi(obj.n_actions); 
            end
            
            

            obj.Acts(end+1) = action; 
            obj.lastAction = action; 
        end
        
        function [] = update(obj, obs, rew)
            if obj.T > length(obj.length(obj.gainRWD))
                obj.gainRWD = [obj.gainRWD, zeros(1, 5000)]; 
            end
            obj = strat_updater(obj, obs, rew);      
            [obj.w, obj.gainEXP3] = EXP3_RecieveReward(obj.w, obj.p, rew, obj.n_arms, obj.gamma, obj.lastAction, obj.gainEXP3); 
            if obj.T > 1
                obj.gainRWD(obj.T) = obj.gainRWD(obj.T-1)+obj.gainEXP3(obj.T); 
            else
                obj.gainRWD(obj.T) = obj.gainEXP3(obj.T); 
            end
        end

        % Unique to this strat
        function [ArmToPlay, p] = EXP3_RecommendArm(w, gamma, pAct, ActionsList)
            K = length(w); 
            p = (1-gamma)*w/sum(w)+gamma/K; 
            flag = ones(1,K); 
            for i = 1:K
                Ed = expDist(ActionsList(i,:),pAct); 
                if Ed > 2
                    flag(i) = 0; 
                end
            end
            Proba = cumsum(p).*flag;
            ArmToPlay = Proba > rand();
            ArmToPlay = find(ArmToPlay == 1,1,'first');
        end

        function [w, gainEXP3] = EXP3_ReceiveReward(w, p,reward, K, gamma, ArmChosen, gainEXP3)
            if((reward <0) + (reward >1) ~= 0)
                error('Reward must be between 0 and 1');
            end
            gainEXP3 = [gainEXP3 reward];
            
            reward = reward/p(ArmChosen);
            
            w(ArmChosen) = w(ArmChosen)*exp(gamma*reward/K);
            w = w/sum(w);
        end


    end
end

