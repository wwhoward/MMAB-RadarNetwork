classdef plat < handle
    %PLATFORM Object which can act as a radar or passive target as well as
    %an emitter
    %   Detailed explanation goes here
    
    properties
        isTarget % Is this a radar target?
        isEmitter % does this transmit
        Strategy % Strategy for if this guy emits
        Label % Name for this platform
        Acts = []
        
        % Radar target specific
        RCS % Radar cross section
        
        % Emitter specific
        tx_power % In dB.
        tx_beamwidth % in rads
        tx_gain % in dB
        strategy
        
        
        initialPosition
        currentPosition
        positionHistory
        
        initialVelocity
        currentVelocity
        
        acceleration
        
    end
    
    methods
        function obj = plat(X, bools, varargin)
            %PLATFORM Construct an instance of this class
            % If is target, varargin = {RCS}
            % If is emitter, varargin = {tx_power, tx_beamwidth, strategy}
            % If both, first comes target.
            % bools = [isTarget, isEmitter]
            
            % Select motion model
            if size(X,1) == 1 % Constant position
                obj.initialPosition = X(1,:);
                obj.currentPosition = X(1,:);
                
                obj.initialVelocity = [0,0];
                obj.acceleration = [0,0];
            elseif size(X,1) == 2 % Constant velocity
                obj.initialPosition = X(1,:);
                obj.currentPosition = X(1,:);
                obj.initialVelocity = X(2,:);
                obj.currentVelocity = X(2,:);
                
                obj.acceleration = 0;
            else
                obj.initialPosition = X(1,:);
                obj.currentPosition = X(1,:);
                obj.initialVelocity = X(2,:);
                obj.currentVelocity = X(2,:);
                obj.acceleration = X(3,:);
            end
            
            % Parse inputs
            if any(strcmp(varargin, 'Strategy'))
                strat = varargin(find(strcmp(varargin, 'Strategy')==1)+1); 
                obj.Strategy = strat{1};
            end
            if any(strcmp(varargin, 'Label'))
                label = varargin(find(strcmp(varargin, 'Label')==1)+1); 
                obj.Label = label{1}; 
            end
            obj.isTarget = bools(1);
            obj.isEmitter = bools(2);
            if bools(1) && not(bools(2))
                obj.RCS = varargin{1};
            elseif not(bools(1)) && bools(2)
%                 obj.tx_power = varargin{1};
%                 obj.tx_beamwidth = varargin{2};
%                 obj.strategy = varargin{3};
            elseif bools(1) && bools(2)
                obj.RCS = varargin{1};
                obj.tx_power = varargin{1};
                obj.tx_beamwidth = varargin{2};
                obj.strategy = varargin{3};
            end
            
            obj.positionHistory(end+1, :) = obj.currentPosition;
        end
        
        function currentPosition = move(obj, t_interval)
            obj.currentPosition = obj.currentPosition + obj.currentVelocity*t_interval + 1/2*obj.acceleration*t_interval;
            obj.currentVelocity = obj.currentVelocity + obj.acceleration*t_interval;
            currentPosition = obj.getCurrentPosition();
            
            obj.positionHistory(end+1, :) = obj.currentPosition;
        end
        
        function currentAction = act(obj, varargin)
            if ~obj.isEmitter
                currentAction = [];
                return
            end
            [currentAction, ~] = obj.strategy.play(varargin); % TODO: can modify to include additional output from strat
        end
        
        function [] = update(obj, varargin)
            
        end
        
        function currentPosition = getCurrentPosition(obj)
            currentPosition = obj.currentPosition;
        end
        
        function [act] = singlePRI(obj)
            % Get true & estimated target information
            
            act = zeros(1,2);
            act(1) = obj.Strategy{1}.play();
            if act(1) ~= 0
                act(2) = obj.Strategy{2}{act(1)}.play();
            else
                act(2) = 0;
            end
            
            obj.Acts(end+1) = act(1); 
        end
    end
end

