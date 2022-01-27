function strat = strat_updater(strat, col, rew)
%STRAT_UPDATER Can push updates to strat objects

if strat.T>length(strat.Rews)
    strat.Rews = [strat.Rews, zeros(strat.n_actions, 5000)]; 
end
if strat.T > length(strat.Cols)
    strat.Cols = [strat.Cols, zeros(1, 5000)]; 
end
if strat.T > length(strat.vector_rewards)
    strat.vector_rewards = [strat.vector_rewards, zeros(1,5000)]; 
end


strat.CumCols = strat.CumCols + col; 
strat.Cols(strat.T) = col; 
if strat.lastAction ~= 0
    strat.Means(strat.lastAction) = (strat.n_pulls(strat.lastAction)*strat.Means(strat.lastAction) + rew)/(strat.n_pulls(strat.lastAction)+1); 
    strat.n_pulls(strat.lastAction) = strat.n_pulls(strat.lastAction) + 1; 
    strat.Rews(strat.lastAction, strat.T) = rew; 
    strat.vector_rewards(strat.T) = rew; 
end



if length(strat.Cols) <= strat.delay
    strat.lastCollision = 0; 
else
    strat.lastCollision = strat.Cols(strat.T - strat.delay); 
end

strat.T = strat.T+1; 

end

