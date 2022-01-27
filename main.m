clc;clear;close all
addpath('classes', 'Strategies', 'Strategies/helper'); 

runs_to_average = 2; 
saving = 0; 


n_nodes = 3; 
positions = 0.5*[[0,1000]; [500, -200]; [1000, 1000]]; 
nActions = [5, 3]; % [nBands, nWaveforms]  
n_pri = 50; % Value indicated by algo paper
n_cpi = 3; 
nCPI = 1; % CPI per block
nBlocks = 100; 
nStrats = 2; % How many different nets

% Add strategies as desired, but update nSims above
Strategies = {  ["SAA", "SAA"]
                ["SAA", "eGreedy"]  };  

errors = zeros(nStrats, runs_to_average, nBlocks); 
regret = zeros(nStrats, runs_to_average, nBlocks*n_pri); 

for run = 1:runs_to_average
    for strat = 1:nStrats
        target = plat([[0,0];[0.00,0.00]], [1,0], 1, 'Label', 'Target'); 
        net = RadarNetwork(n_nodes, positions, Strategies{strat}, nActions, 'n_pri', n_pri, 'n_cpi', n_cpi);
        scene = Scene(net, {target}, nActions);   
        scene.toggle_int_cols(); 
        
        for n = 1:nBlocks*nCPI
            tic
            scene.simulate_cpi(); 

            clc; 
            display('Run ' + string(run) + ', Net ' + string(strat) + ', CPI #' + string(n) + ' out of ' + string(nBlocks*nCPI) + ', CPI Duration: ' + toc); 
        end
            
        errors(strat, run, :) = scene.c_error; 
        regret(strat, run, :) = scene.regret;  
    end
end

%%
mean_err = zeros(nStrats, nBlocks); 
mean_regret = zeros(nStrats, n_pri*nBlocks); 
for strat = 1:nStrats
    mean_err(strat, :) = mean(squeeze(errors(strat, :, :)), 1); 
    mean_regret(strat, :) = mean(squeeze(regret(strat, :, :)), 1); 
end 

%% %%%%%%%%%%%%%%%
% % Make Plots % %
%%%%%%%%%%%%%%% %%

f3 = figure; hold on; 
for strat = 1:nStrats
    semilogy(movmean(mean_err(strat,:),10), 'linewidth', 2); 
end
title('Position Tracking Error', 'interpreter', 'latex', 'fontsize', 16)
legend('SAA / SAA', 'SAA / $\epsilon$-Greedy', 'interpreter', 'latex', 'fontsize', 12)
xlabel('Coherent Pulse Interval', 'interpreter', 'latex', 'fontsize', 12)
ylabel('Error (m)', 'interpreter', 'latex', 'fontsize', 12)
% ylim([0, 100])

f2 = figure; hold on
for strat = 1:nStrats
plot(1/3*mean_regret(strat, :), 'linewidth', 2); 
end
title('Cumulative Regret', 'interpreter', 'latex', 'fontsize', 16)
legend('SAA / SAA', 'SAA / $\epsilon$-Greedy', 'interpreter', 'latex', 'fontsize', 12, 'location', 'northwest')
xlabel('Pulse Repetition Interval', 'interpreter', 'latex', 'fontsize', 12)
ylabel('Cumulative Regret', 'interpreter', 'latex', 'fontsize', 12)

figname1 = "combo_comparison_regret";
figname2 = "combo_comparison_error"; 
figname3 = "combo_comparison_averegret";

if saving
    saveas(f2, figname1+'.eps')
    saveas(f2, figname1+'.png')
    savefig(f2, figname1+'.fig')
    saveas(f3, figname2+'.eps')
    saveas(f3, figname2+'.png')
    savefig(f3, figname2+'.fig')
end







