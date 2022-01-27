classdef C_and_P < handle
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
        
        % Algo Specific
        
        % Timing
        nBlocks=50 % Total blocks in T
        nSub_block=10 % Total sub-blocks per block
        nSteps=5 % Total steps per sub-block
        
        step % Current step in sub-block
        sub_block % Current sub-block in block
        block % Current block
        
        % Coordinator
        % eta = 1 % 
        eta = 0.0259
        MetaArms % Set of meta-arms
        nMetaArms % Number of meta-arms
        L % meta-arm loss
        l_hat
        l_tilde
        loss % Loss for each arm in each time step (there will be 0's)
        P % meta-arm probability
        
        current_meta % current meta-arm
        MetaHistory % History of meta-arms. Only stores index. 
        
        block_loss 
        
        sub_block_flag = 0 
        
        %Follower
        fixed_action % Arm sent from coordinator to this follower
        
        
        
        
    end
    
    methods
        function obj = C_and_P(n_actions, ID, n_players, varargin)
            %MC_TOPM Construct an instance of this class
            %   Detailed explanation goes here
            obj = strat_constructor(obj, n_actions, ID, n_players, varargin); 
            
            % obj.nSub_block = 50*(n_players-1);
            
            obj.step = 1; 
            obj.sub_block = 2; 
            obj.block = 1; 
            
            if obj.ID==1
                obj.MetaArms = nchoosek(obj.Actions, obj.n_players);
                obj.nMetaArms = size(obj.MetaArms, 1); 
                
                obj.block_loss = zeros(obj.n_actions, obj.nSub_block*obj.nSteps); 
            end
            
        end
        
        function action = play(obj)
            if obj.ID == 1 % Coordinator algorithm
                % If first step of block, select a meta-arm
                if (obj.sub_block == 2) && (obj.step == 1)
                    obj.updateLoss; 
                    obj.updateProb; 
                    obj.SampleMetaArm; 
                end
                               
                
                if obj.sub_block <= obj.n_players
                    if (obj.lastCollision == 0 && obj.sub_block_flag == 0) || obj.step == 1
                        action = obj.current_meta(obj.sub_block); 
                    else
                        obj.sub_block_flag = 1; 
                        action = obj.current_meta(1); 
                    end
                else
                    action = obj.current_meta(1); 
                end
            else % Follower algorithm
%                 if obj.ID==3
%                     display([obj.sub_block, obj.step, obj.T])
%                 end
                if obj.lastCollision == 1
                    obj.fixed_action = obj.lastAction;
                end
                if obj.sub_block < obj.ID % Not yet our sub-block
                    action = 0; 
                elseif obj.sub_block == obj.ID % OK this is it let's go
                    if obj.lastCollision == 1 || ~isempty(obj.fixed_action)
                        obj.fixed_action = obj.lastAction; 
                        action = obj.lastAction; 
                    else
                        action = mod(obj.step, obj.n_actions)+1; 
                    end
                elseif (obj.sub_block > obj.ID) && (obj.sub_block <= obj.n_players) % Our time passed; be patient, it will come again
                    action = 0;                     
                else % Time to send it
                    if isempty(obj.fixed_action)
                        % return
                        % continue
                    end
                    if obj.lastCollision == 1
                        obj.fixed_action = obj.lastAction; 
                    end
                    action = obj.fixed_action; 
                end
            end
            
            
            
            obj.Acts(end+1) = action; 
            obj.lastAction = action; 
        end
        
        function [] = update(obj, obs, rew)
            obj = strat_updater(obj, obs, rew); 
            
            if obj.ID==1
                obj.block_loss(obj.lastAction, obj.nSteps*(obj.sub_block-2) + obj.step) = 1-rew; 
            end
            % Update block, sub_block, step_in_sub_block
            obj.step = obj.step + 1; 
            if obj.step > obj.nSteps
                obj.step = 1; 
                obj.sub_block = obj.sub_block + 1; 
                obj.sub_block_flag = 0; 
                if obj.sub_block > obj.nSub_block+1 % +1 to account for starting at 2
                    obj.sub_block = 2; % Sub-blocks start at 2
                    obj.block = obj.block + 1; 
                    obj.fixed_action = []; 
                    
                    % Update loss information for meta-player, if this is
                    % the coordinator
                    if obj.ID == 1
                        obj.l_hat = [obj.l_hat, sum(obj.block_loss,2)]; 
                        % obj.l_tilde = [obj.l_tilde, obj.n_players*(obj.l_hat(:, end)/(obj.nSub_block*obj.nSteps))]; % TODO: this might yet be wrong
                        obj.l_tilde = [obj.l_tilde, zeros(obj.n_actions, 1)]; % TODO: this might yet be wrong
                        obj.l_tilde(obj.current_meta(1), end) = obj.n_players*(obj.l_hat(obj.current_meta(1), end)/(obj.nSub_block*obj.nSteps*obj.GetSumProb()));
                    end
                end
            end            
        end
        
        % For just this algo
        function [MetaArms] = GetMetaArms(obj)
            MetaArms = nchoosek(obj.Actions, obj.n_players); 
            
            for i = 1:size(MetaArms,2)
                idx = find(sort(MetaArms,1)==sort(MetaArms(i,:),1)) \ i; 
                MetaArms(idx) = []; 
            end
            
        end
        
        function updateLoss(obj)
            obj.L = [obj.L; zeros(1, obj.nMetaArms)]; 
            
            if isempty(obj.l_tilde)
                return
            else
                for i = 1:obj.nMetaArms
                    obj.L(end, i) = obj.L(end, i) + sum(sum(obj.l_tilde(obj.MetaArms(i,:),:))); 
                end
            end
        end
        
        function updateProb(obj)
            expL = exp(-obj.eta * obj.L(end,:)); 
            sum_exp = sum(expL); 
            
            obj.P = expL ./ sum_exp;             
        end
        
        function SampleMetaArm(obj)
            sampled_arm = randsample(1:obj.nMetaArms, 1, true, obj.P); 
            obj.MetaHistory = [obj.MetaHistory, sampled_arm]; 
            rand_idx = randperm(obj.n_players); 
            obj.current_meta = obj.MetaArms(sampled_arm, rand_idx); 
        end
        
        function SumProb = GetSumProb(obj) % Returns sum of probabilities of all MetaArms containing this player's fixed arm. 
            arm = obj.current_meta(obj.ID); % Current fixed arm
            idx = []; 
            for i = 1:obj.nMetaArms
                if any(obj.MetaArms(i,:)==arm)
                    idx = [idx, i]; 
                end
            end
            SumProb = sum(obj.P(idx)); 
        end
    end
end

