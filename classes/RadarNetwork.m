classdef RadarNetwork < handle
    %RADARNETWORK Holds information on a radar network
    
    properties
        % Inputs
        nNodes % Number of nodes in the network. Size [1]
        Nodes = {} % holds the nodes. Size [1,nNodes]
        Positions % Positions of each node. Size [2, nNodes]
        Actions % Array of indicies of each action in nBands x nWaveforms. Size [2, nBands, nWaveforms]
        nBands % How many bands? Size [1]
        nWaveforms % How many waveforms? Size [1]
        Strategy % Tuple, [BandStrat, WaveStrat]. Selects an element of the space nBands x nWaves. 
        n_pri % How many PRIs per CPI
        pri = 0.1024 % how many seconds per PRI
        n_cpi % How many CPIs to simulate
        Scene % Scene object for broadcast
        
        
        % Filtering
        EKF
        states
        cstates
        mean_cstates
        
        
        % Private
        ActionHistory = {} % History of actions, accumulated over time
        CollisionHistory = [] % History of collisions
        ObservedCollisionHistory = [] % History of observed collisions
        RewardHistory = [] % History of rewards as observed by the network
        CumulativeReward = [] % Accumulated reward of each node at each time step
        
        RegretHistory = [] % History of regret as observed by the network
        CumulativeRegret = [] % Accumulated regret of each node at each time step
        
        
        
        
    end
    
    methods
        function obj = RadarNetwork(nNodes, positions, strategy, nActions, varargin)
            %RADARNETWORK Construct an instance of this class

            obj.nNodes = nNodes; 
            obj.Positions = positions; 
            
            if any(strcmp(varargin, 'n_pri'))
                obj.n_pri = varargin{find(strcmp(varargin, 'n_pri')==1)+1};
            end
            if any(strcmp(varargin, 'n_cpi'))
                obj.n_cpi = varargin{find(strcmp(varargin, 'n_cpi')==1)+1};
            end            
            obj.Strategy = strat_assign(strategy, nActions, nNodes, varargin); %'n_pri', n_pri, 'n_cpi', n_cpi); 
            
            obj.Actions = zeros([nActions(1), nActions(2), 2]); 
            for b = 1:nActions(1)
                for w = 1:nActions(2)
                    obj.Actions(b, w, :) = [b,w]; 
                end
            end
            
            obj.ActionHistory = zeros([nNodes,0]);
            
            for n = 1:nNodes
                obj.Nodes{n} = node({obj.Strategy{:,n}}, nNodes, positions(n,:), nActions, obj.n_pri, obj.pri);                 
            end
            
            
            % Filtering stuff
            % cstates = zeros(0, obj.nNodes, 4);
            obj.initialize_kf(); 
            
        end
        
        % PRI Functions
        function CurrentActs = pri_transmit(obj, target)
            CurrentActs = zeros(obj.nNodes, 2); 
            for n = 1:obj.nNodes
                CurrentActs(n,:) = obj.Nodes{n}.singlePRI(target); 
            end            
            obj.ActionHistory{end+1} = CurrentActs; 
        end
        
        function pri_receive(obj, rews)            
            % Calculate cols and rews
            utilization = obj.Scene.Utilization{end}; 
            
            
            cols = zeros(1,obj.nNodes); 
            hist = obj.ActionHistory{end}; 
            for n = 1:obj.nNodes
                if hist(n,1) ~= 0 && hist(n,2) ~= 0
                    if utilization(hist(n,1), hist(n,2)) > 1
                        cols(n) = 1; 
                    end
                end
            end
            obj.CollisionHistory = [obj.CollisionHistory, cols'];             
            
            % Update nodes
            for n = 1:obj.nNodes
                obj.Nodes{n}.update(cols(n), rews(n)); 
            end
        end
        
        % CPI Functions
        function doCoherentProcessing(obj, maxRew)
            tmp_pos_hat = zeros(obj.nNodes, 2); 
            tmp_vel_hat = zeros(obj.nNodes, 2); 
            tmp_cstates = zeros(obj.nNodes, 4); 
            for i=1:obj.nNodes
                [tmp_pos_hat(i,:), tmp_vel_hat(i,:), tmp_cstates(i,:)] = obj.Nodes{i}.doCoherentProcessing(maxRew); 
            end
            
            obj.cstates(end+1,:,:) = tmp_cstates; 
            obj.mean_cstates(end+1,:) = mean(obj.cstates(end,:,:),2); % This feels wrong
            
            % Try collective filtering - still haven't figured out how to
            % do multi-sensor filtering
            % pstates = predict(obj.EKF, obj.n_pri*obj.pri); 
            % obj.cstates(end+1, :, :) = correct(obj.EKF, tmp_pos_hat); 
        end
        
        function initialize_kf(obj)
            mmodel = ones(obj.nNodes, 1) * [1, 0, 1, 0]; 
            % obj.EKF = trackingKF(@constvel, 'MeasurementModel', mmodel); 
            obj.EKF = trackingKF('MotionModel', '2D Constant Velocity', 'State', [0, 0, 0, 0], ...
                'EnableSmoothing', true, 'MaxNumSmoothingSteps', 1e7, 'MeasurementModel', mmodel);
        end
        
        function [smoothstates] = smoothKF(obj)
            for n = 1:obj.nNodes
                [smoothstates(n,:,:), ~] =  smooth(obj.Nodes{n}.KF); 
            end
        end
        
    end
end

