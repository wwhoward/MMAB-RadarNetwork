classdef node < handle
    %NODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Strategy % Strategy object used by this player
        n_nodes % How many total nodes in the network?
        Bands % Which bands are available
        Waveforms % Which waveforms are available
        ID % Unique identifier
        Acts
        
        initialPosition
        currentPosition
        pos % Position history
        
        initialVelocity
        currentVelocity
        vel % Velocity history
        
        acceleration
        
        % Bandit stuff
        % BandActions = [] % Which bands are selected, a history
        nPRI
        intervalPRI % How long a PRI lasts
        
        % Target info
        t_pos % True target positions, per PRI
        t_vel % True target velocities, per PRI
        
        t_range % True target range, avereged per CPI
        t_vt % True target velocity, avereged per CPI
        t_theta % True target angle, avereged per CPI
        
        t_range_hat % Estimated target range, per CPI
        t_vr_hat % Estimated target radial velocity, per CPI
        t_theta_hat % Estimated angle to target, per CPI
        
        
        % Radar parameters
        T = 1 % PRI index, resets with each CPI, start at 1 for indexing
        
        % Tracking filters
        KF
        pstates
        cstates
        
        pos_hat % [x, x', y, y']
        
        currentEstimate
        
        
        % Coherent processing constants
        Ntx = 1024; % Tx samples per PRI
        Fs = 100e6; % Sampling rate
        LO = 2.5e9; % Carrier freq
        npower = 10^(-4); % Noise power
        X = 1:8/200:9; 
        x1 = 1:9; 
        Ro = 400; 
        
        CMRmap = [0.00 0.00 0.00;
            0.15 0.15 0.50;
            0.30 0.15 0.75;
            0.60 0.20 0.50;
            1.00 0.25 0.15;
            0.90 0.50 0.00;
            0.90 0.75 0.10;
            0.90 0.90 0.50;
            1.00 1.00 1.00];
        
        % Coherent processing to be filled in later
        tau
        PRI
        PRF
        pad_R
        df
        dR
        Velocities
        SNR
        
        
        
    end
    
    methods
        function obj = node(Strategy, n_nodes, X, nActions, nPRI, intervalPRI, varargin)
            %NODE Construct an instance of this class
            
            obj.Strategy = Strategy; 
            obj.n_nodes = n_nodes; 
            obj.Bands = 1:nActions(1); 
            obj.Waveforms = 1:nActions(2); 
            obj.nPRI = nPRI; 
            obj.ID = Strategy{1}.ID; 
            obj.intervalPRI = intervalPRI; 
                     
            
            % Target parameters
            obj.t_pos = zeros(nPRI, 2); 
            obj.t_vel = zeros(nPRI, 2); 
            
            
            % Select motion model
            if size(X,1) == 1 % Constant position
                obj.initialPosition = X(1,:); 
                
                obj.initialVelocity = [0,0]; 
                obj.acceleration = [0,0]; 
            elseif size(X,1) == 2 % Constant velocity
                obj.initialPosition = X(1,:); 
                obj.initialVelocity = X(2,:); 
                
                obj.acceleration = 0; 
            else
                obj.initialPosition = X(1,:); 
                obj.initialVelocity = X(2,:); 
                obj.acceleration = X(3,:);               
            end
            obj.currentPosition = obj.initialPosition; 
            % TODO: Varargin stuff
            obj.coherentInitializing(); 
            
            % Initialize tracker
            obj.KF = trackingKF('MotionModel', '2D Constant Velocity', 'State', [0, 0, 0, 0], 'EnableSmoothing', true, 'MaxNumSmoothingSteps', 1e3); 
        end
        
        
        
        function currentPosition = getCurrentPosition(obj)
            currentPosition = obj.currentPosition;
        end
        
        function currentPosition = move(obj, t_interval)
            obj.currentPosition = obj.currentPosition + obj.curentVelocity*t_interval + 1/2*obj.acceleration*t_interval;
            obj.currentVelocity = obj.currentVelocity + obj.acceleration*t_interval;
            currentPosition = obj.getCurrentPosition();
        end
        
        function [act] = singlePRI(obj, target)
            % Get true & estimated target information
            % TODO: Only works with first target for now
            obj.t_pos(obj.T, :) = target{1}.currentPosition; 
            obj.t_vel(obj.T, :) = target{1}.currentVelocity; 
                        
            act = zeros(1,2); 
            act(1) = obj.Strategy{1}.play();             
            if act(1) ~= 0
                act(2) = obj.Strategy{2}{act(1)}.play(); 
            else
                act(2) = 0; 
            end
            obj.T = obj.T + 1; 
            obj.Acts(end+1,:) = act; 
        end
        
        function update(obj, cols, rews)
            obj.Strategy{1}.update(cols, rews);
            if obj.Strategy{1}.lastAction ~= 0
                strat_updater(obj.Strategy{2}{obj.Strategy{1}.lastAction}, cols, rews); 
            end
            % bandCols = zeros(1, obj.
            
        end
            
        function [position_hat, velocity_hat, cstate] = doCoherentProcessing(obj, maxRew)
            acts = obj.Strategy{1}.Acts(obj.Strategy{1}.T-obj.nPRI: obj.Strategy{1}.T-1); 
            cols = obj.Strategy{1}.Cols(obj.Strategy{1}.T-obj.nPRI: obj.Strategy{1}.T-1); 
            rews = obj.Strategy{1}.vector_rewards(obj.Strategy{1}.T-obj.nPRI: obj.Strategy{1}.T-1); 
            
            % mean_rew = mean(rews); 
            mean_rew = max(sum(rews)/sum(rews ~= 0), 0); 
            
            
            [trueR, theta, trueVr] = obj.globalToLocal(obj.t_pos, obj.t_vel); 
            r = trueR(end); 
            v = trueVr(end); 
            
            npri = sum(acts~=0); 
            pad_D = 2^(nextpow2(2*npri)); 
            
            % Simulate CPI
            [BW, CF] = obj.getDDS(acts, length(obj.Bands)); 
            colRate = sum(cols)/npri;  % Collision rate to inform SINR
            
            chirps = zeros(obj.Ntx, npri); 
            t = linspace(0, obj.tau-1/obj.Fs, obj.Ntx); 
            
            for pulse = 1:sum(acts ~= 0)
                f1 = CF(pulse) - BW(pulse)/2; 
                f2 = CF(pulse) + BW(pulse)/2; 
                k = (f2 - f1)/obj.tau; 
                
                % chirps(:, pulse) = 1/obj.n_nodes * exp(1i*(2*pi*f1*t + pi*k*t.^2)); 
                chirps(:, pulse) = exp(1i*(2*pi*f1*t + pi*k*t.^2)); 
            end
            
            cn = obj.npower * randn(1, npri); 
            target_range = trueR(acts~=0)' + cn; 
            target_range_bins = target_range(1)*ones(1, npri)*2/3e8*obj.Fs; 
            chirps_rx = zeros(size(chirps)); 
            
            for pulse = 1:npri
                chirps_rx(:, pulse) = circshift(chirps(:, pulse), -floor(target_range_bins(pulse)-1)); 
            end
            
            % Add doppler with narrowband assumption
            chirps_rx = 0.07*chirps_rx .* repmat(exp(1i*4*pi*target_range.*(obj.LO)/3e8), obj.Ntx, 1); 
            
            % Just a single target, so
            chirps_rx = chirps_rx + 1e-9*(randn(obj.Ntx, npri) + 1i*randn(obj.Ntx, npri)); 
            
            
            % np = 7.1e-4 + (colRate*2e-2) + real(sqrt(1-(mean_rew/maxRew)^0.5)*6e-3); 
            % np = 0.6e-5 + (colRate*2e-2) + real(sqrt(1-(mean_rew/maxRew)^0.7)*1e-5); 
            % np = 0.6e-5 + (colRate*0.5e-4) + real(sqrt(1-(mean_rew/maxRew)^0.7)*1e-5); 
            % np = 0.6e-5 + real(sqrt(1-(mean_rew/maxRew)^0.7)*1e-5); % Double counting collisions with above method
            
            % np = 0.6e-4 + real(sqrt(1-(mean_rew/maxRew)^0.7)*0.1e-5); % Double counting collisions with above method
            % np = 0; 
            
            % np = 4e-6 * (1 + 0.5 *real(sqrt(1-(mean_rew/maxRew)^0.7))); % Double counting collisions with above method
            np = 6e-6 * (1 + 0.5 *real(sqrt(1-(mean_rew/maxRew)))); % Double counting collisions with above method
            
            nois = np*log(randn(size(chirps))) + 1i*np*log(randn(size(chirps))); 
            
            Rxx = (conj(fft(chirps, obj.pad_R)).*fft(chirps_rx, obj.pad_R)/obj.Ntx/obj.Ntx); 
            Rxx = Rxx + nois; 
            
            % Windows
            w_doppler = transpose(repmat((kaiser(npri, 8)), 1, obj.Ntx)); 
            
            % Range window must adjust to pulse CF and BW
            w_range = zeros(obj.Ntx, npri);
            Sts = floor((CF-BW/1.8)./obj.df + obj.pad_R/2);
            Eds = floor((CF+BW/1.8)./obj.df + obj.pad_R/2);
            Lens = Eds-Sts;
            
            for pulse = 1:npri
                w_range(:,pulse) = fftshift(circshift([kaiser(Lens(pulse),5); 1e-6*ones(obj.pad_R-Lens(pulse),1)], Sts(pulse), 1));
            end
            
            % Doppler processing
            RD = fftshift((fft2(w_range.*w_doppler.*(Rxx),obj.pad_R, pad_D)), 2)/npri;
            RD(:,obj.Ntx/2) = 0;
            
            ranges = (1:obj.dR:(obj.dR*obj.Ntx));
            
            dbRD = db(RD); 
            
            RD2 = dbRD(100:end-100, 100:end-100);
            
            maxValue = max(RD2(:)); 
            
            % Find all locations where maxValue exists
            [rows, columns] = find(dbRD == maxValue); 
            rhat = ranges(rows); 
            vhat = obj.Velocities(columns); 
            
            range_hat = rhat; 
            vr_hat = vhat; 
            
            [position_hat, velocity_hat] = obj.localToGlobal(range_hat, theta(end)+0.01*(1-(mean_rew/maxRew))*randn, vr_hat); 
            % [position_hat, velocity_hat] = obj.localToGlobal(range_hat, theta(end), vr_hat); %Use if we want true thetas 
            
            % Update filters and histories
            obj.pos_hat(end+1,:) = [position_hat(1), velocity_hat(1), position_hat(2), velocity_hat(2)]; 
            obj.updateKF()
            cstate = obj.cstates(end,:); 
            
            
            obj.T = 1; % Reset PRI counter
        end
        
        function [range, theta, RV] = globalToLocal(obj, pos, vel)
            % TODO: Doesn't account for moving receiver
            npri = obj.nPRI; 
            
            range = zeros(npri, 1); 
            theta = zeros(npri, 1); 
            RV = zeros(npri, 1); 
            for n = 1:npri
                range(n) = abs(sqrt((obj.currentPosition(1)-pos(n,1))^2 + (obj.currentPosition(2)-pos(n,2))^2)); 
                theta(n) = atan2((pos(n,2)-obj.currentPosition(2)),(pos(n,1)-obj.currentPosition(1))); 
                RV(n) = -dot((obj.currentPosition-pos(n,:)), vel(n,:)) / norm(obj.currentPosition-pos(n,:));
            end
        end
        
        function [pos, vel] = localToGlobal(obj, range, theta, RV)
            N = length(range); 
            
            pos = zeros(N, 2); 
            vel = zeros(N, 2); 
            
            theta = -theta; 
            
            for n = 1:N
                R = [cos(theta(n)), -sin(theta(n)); sin(theta(n)), cos(theta(n))]; 
                pos(n,:) = [range(n),0] * R + obj.currentPosition; 
                vel(n,:) = RV * (pos(n,:)-obj.currentPosition)/norm(pos(n,:)-obj.currentPosition);                 
            end
        end
        
        function coherentInitializing(obj)
            obj.tau = obj.Ntx/obj.Fs; 
            obj.PRI = (10*obj.Ntx/obj.Fs); % How long is a PRI 
            obj.PRF = 1./obj.PRI; % How often is a PRI
            obj.pad_R = obj.Ntx; 
            obj.df = obj.Fs / obj.pad_R; 
            obj.dR = (3e8/(2*obj.Fs)); 
            
            Vres = (3e8*obj.PRF)/(2*obj.LO*obj.Ntx); 
            maxV = Vres * (obj.Ntx/2); 
            vScaling = maxV/(obj.Ntx/2); 
            
            obj.Velocities = vScaling .* ((-obj.Ntx/2)+1:obj.Ntx/2); 
            
            obj.SNR = 12;             
        end
        
        function updateKF(obj)
            obj.pstates(end+1, :) = predict(obj.KF, obj.nPRI*obj.intervalPRI); 
            obj.cstates(end+1, :) = correct(obj.KF, obj.pos_hat(end,[1,3])); 
        end
    end
    
    
    
    methods (Static)
        function [bw, cf] = getDDS(radarCPI, nSB)
            nPulses = length(radarCPI); 
            sbSize = 90e6/nSB; 
            
            bw = zeros(nPulses, 1); 
            cf = zeros(nPulses, 1); 
            
            for kk = 1:nPulses
                util = 1; % Util always seems to be 1? 
                % util = find(radarCPI(:,kk)); 
                chirpStart  = (min(util)-1)*sbSize; 
                chirpEnd    =  max(util)*sbSize; 
                bw(kk)      =  chirpEnd - chirpStart; 
                cf(kk)      = median([chirpStart, chirpEnd]); 
                cf(kk)      = cf(kk) - 45e6; % something something baseband? 
            end
        end
        
        
    end
end

