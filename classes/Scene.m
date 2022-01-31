classdef Scene < handle
    %SCENE Creates a scene containing radars, targets, and emitters
    % All objects (RadarNetworks, nodes, my_platforms) interact within the scene. 
    % In each of many time steps, all emitters (includes nodes &
    % plats.isEmitter=1) can choose one action A in [Bands]x[Waveforms]. To
    % avoid waveform selection, make [Waveforms] = 1. 
    
    % The scene determines collisions (instances of two emitters selecting
    % the same A in [Bands]x[Waveforms]. 
    
    % All nodes in a RadarNetwork can execute some collaborative strategy. 
    
    % Plat emitters can also leverage strategies to increase the regret
    % observed by emitters. 
    
    % Plats can be emitters, targets, or both. 
    properties
        Networks = [] % Index of present network objects
        Nodes = [] % Node objects in each network
        Platforms = [] % List of objects in the environment and indicators of radar/passive targeting and emissions
        PlatEmitters = {}
        PlatTargets = {}
        nPlats
        nEmitters % Cumulative number of emitters
        
        Actions = {} % Cell array with action histories for Networks as well as Platforms
        Collisions = {} % Array counting # collisions per band in each time step
        Rewards = {} % Cell array keeping a record of /network/ rewards. Shape [nPRI, nNets, nNodes]
        Utilization = {} % 3D array counting # uses per band/waveform per time step
        
        nActions % [nBands, nWaveforms]
        
        nPRI
        intervalPRI
        
        MeanRewards
        RewardType = "Gaussian"
        % RewardType = "Adversarial50"; 
        
        int_cols = 0 % Do interferers cause collisions
        
        % Analysis
        c_error
        regret
        
    end
    
    methods
        function obj = Scene(Networks, Platforms, nActions)
            %SCENE Construct an instance of this class
            %   Detailed explanation goes here
            obj.Networks = Networks; 
            obj.Platforms = Platforms; 
            for p = 1:length(Platforms)
                if Platforms{p}.isEmitter
                    obj.PlatEmitters{end+1} = Platforms{p}; 
                end
                if Platforms{p}.isTarget
                    obj.PlatTargets{end+1} = Platforms{p}; 
                end
            end
            
            obj.nActions = nActions; 
            obj.nPlats = length(Platforms); 
            
            for n = 1:length(Networks)
                obj.Networks.Scene = obj; 
            end
            
            obj.intervalPRI = obj.Networks(1).pri; 
            obj.nPRI = obj.Networks(1).n_pri; 
            
            % obj.MeanRewards = linspace(0.9, 0.5, nActions(1))' .* ...
            %     (linspace(0.9, 0.1, nActions(2)) .* ones(nActions(1), nActions(2))); 
            obj.MeanRewards = [0.95, 1, 0.9, 0.4, 0.4, 0.01]' .* ...
                (linspace(0.9, 0.1, nActions(2)) .* ones(nActions(1), nActions(2))); 
        end
        
        function simulate_cpi(obj)
            % The smallest resonable stretch of time to run a network is
            % one CPI
            for pri = 1:obj.nPRI
                % obj.simulate_pri(obj.PlatTargets); 
                obj.simulate_pri(); % TODO: clean this up ^
                obj.update_targets(obj.intervalPRI); 
            end            
            for i = 1:length(obj.Networks)
                obj.Networks(i).doCoherentProcessing(obj.getMaxRewards()); 
            end
            
            % Analysis
            obj.c_error(end+1) = norm(obj.Networks(1).mean_cstates(end,[1,3]) - obj.PlatTargets{1}.currentPosition); 
            
            
        end
        
        function simulate_pri(obj)
            % Called from main. Collects actions from all emitters in the
            % environment, determines collisions which are then
            % broadcasted to the emitters. 
            
            % Note on actions: they are denoted as a tuple (b^(e)_i, w^(e),i) of
            % the band and waveform selected by emitter e in time step i. 
            
            
            current_actions = {}; 
            
            for n = 1:length(obj.Networks)
                acts = obj.Networks(n).pri_transmit(obj.PlatTargets); 
                for a = 1:size(acts,1)
                    current_actions{end+1} = acts(a,:); 
                end
            end
            for e =1:length(obj.PlatEmitters)
                current_actions{end+1} = obj.PlatEmitters{e}.singlePRI(); 
            end
            
            [cols, rews, mean_rews, utilization] = obj.getFeedback(current_actions); 
            
            obj.Collisions{end+1} = cols; 
            obj.Rewards{end+1} = rews; 
            obj.Utilization{end+1} = utilization; 
            
            for n = 1:length(obj.Networks)
                nNodes = obj.Networks(n).nNodes; % Nodes in this network
                network_rews = rews(1:nNodes); % Rewards for this network TODO: Change to be correct
                obj.Networks(n).pri_receive(network_rews); 
            end
%             for e = 1:length(obj.PlatEmitters)
%                 obj.PlatEmitters.pri_receive(); 
%             end
            
            
            
            if isempty(obj.regret)
                obj.regret(1) = obj.Networks(1).nNodes*obj.getMaxRewards() - sum(mean_rews); 
            else
                obj.regret(end+1) = (obj.Networks(1).nNodes*obj.getMaxRewards() - sum(mean_rews)) + obj.regret(end); 
            end
        end
         
        function update_targets(obj, t_interval)
            for i = 1:length(obj.PlatTargets)
                obj.PlatTargets{i}.move(t_interval);
            end
        end  
        
        function cols = current_cols(obj)
            cols = obj.Collisions(end,:); 
        end
        
        function [cols, rews, mean_rews, utilization] = getFeedback(obj, current_actions)
            % Only track rews for nodes. Not platemitters. 
            % TODO: Cols are only between nodes. If we want jammer cols to
            % count, change this. 
            cols = zeros(obj.nActions(1), obj.nActions(2)); 
            rews = zeros(1, size(current_actions,2)-length(obj.PlatEmitters)); 
            mean_rews = zeros(1, size(current_actions,2)-length(obj.PlatEmitters)); 
            utilization = zeros(obj.nActions(1), obj.nActions(2)); 
            
            if obj.int_cols == 1
                len_current_acts = length(current_actions); 
            else
                len_current_acts = length(current_actions)-length(obj.PlatEmitters); 
            end
            
            for i = 1:len_current_acts
                if current_actions{i}(1) ~= 0 && current_actions{i}(2) ~= 0
                    utilization(current_actions{i}(1), current_actions{i}(2)) = utilization(current_actions{i}(1), current_actions{i}(2)) + 1;                   
                end
            end
            cols(utilization>1) = 1; 
            
            meanRewards = obj.MeanRewards; 
            for i = obj.Networks(1).nNodes+1:length(current_actions) % This sets rewards to 0 if an interferer plays & if int_cols is false.  
                tmp = current_actions{i};
                if tmp(1) ~= 0 && obj.int_cols==0
                    meanRewards(tmp(1), 1) = 0; 
                end
                % meanRewards(current_actions(i){1}) = 0; 
            end
            
            switch obj.RewardType
                case "Gaussian"
                    variance = 0.01; 
                    for n = 1:length(rews)
                        if current_actions{n}(1) ~= 0 && current_actions{n}(1) ~= 0
                            rews(n) = (variance*randn())+meanRewards(current_actions{n}(1), current_actions{n}(2)); 
                            rews(n) = rews(n) * (1-cols(current_actions{n}(1), current_actions{n}(2))); 
                            if rews(n) < 0
                                rews(n) = 0; 
                            end
                            if rews(n) > 1
                                rews(n) = 1; 
                            end
                            
                            mean_rews(n) = meanRewards(current_actions{n}(1), current_actions{n}(2)); 
                            mean_rews(n) = mean_rews(n) * (1-cols(current_actions{n}(1), current_actions{n}(2)));
                        else
                            rews(n) = 0; 
                        end
                    end
                case "Simple"
                    for n = 1:length(rews)
                        rews(n) = 1; 
                    end
                    
                    
            end
        end
        
        function [maxR] = getMaxRewards(obj)
            maxR = sum(maxk(obj.MeanRewards(:,1), obj.Networks(1).nNodes))/obj.Networks(1).nNodes; 
        end
        
        function [meanR] = getCurrentMeanReward(obj, current_actions)
            meanR = zeros(1, obj.Networks(1).nNodes); 
            for n = 1:obj.Networks(1).nNodes
                meanR(n) = obj.MeanRewards(current_actions{n}(1), current_actions{n}(2));      
            end
        end
        
        function [] = toggle_int_cols(obj)
            obj.int_cols = 1 - obj.int_cols; % flip dat bit
        end
        
    end
end

