function Strats = strat_assign(strategies, nActions, n_players, varargin)
%STRAT_ASSIGN Summary of this function goes here
%   Detailed explanation goes here
Strats = cell(2, n_players);

for player = 1:n_players
    switch strategies{1}
        case "MC_TopM"
            Strats{1, player} = MC_TopM(nActions(1), player, n_players, varargin);
        case "MC"
            Strats{1, player} = MC(nActions(1), player, n_players, varargin); 
        case "C_and_P"
            Strats{1, player} = C_and_P(nActions(1), player, n_players, varargin); 
        case "orthog"
            Strats{1, player} = orthog(nActions(1), player, n_players, varargin); 
        case "SAA"
            Strats{1, player} = SAA(nActions(1), player, n_players, varargin); 
        % Now for "bad guy" strategies
        case "Follower"
            Strats{1, player} = Follower(nActions(1), player, n_players, varargin); 
        case "NoOp"
            Strats{1, player} = NoOp(nActions(1), player, n_players, varargin); 
        
    end
    
    switch strategies{2}
        case "eGreedy"
            s = cell(1,nActions(1)); 
            for i = 1:nActions(1)
                s{i} = eGreedy(nActions(2), i, n_players, varargin); 
            end
            Strats{2, player} = s;
        case "eDecaying"
            s = cell(1,nActions(1)); 
            for i = 1:nActions(1)
                s{i} = eDecaying(nActions(2), i, n_players, varargin); 
            end
            Strats{2, player} = s; 
        case "NoOp"
            s = cell(1,nActions(1)); 
            for i = 1:nActions(1)
                s{i} = NoOp(nActions(2), i, n_players, varargin); 
            end
            Strats{2, player} = s; 
        case "SAA"
            s = cell(1,nActions(1)); 
            for i = 1:nActions(1)
                s{i} = SAA(nActions(2), i, n_players, varargin); 
            end
            Strats{2, player} = s; 
        case "subFollower"
            s = cell(1,nActions(1)); 
            for i = 1:nActions(1)
                s{i} = subFollower(nActions(2), i, n_players, varargin); 
            end
            Strats{2, player} = s; 
    end
end

end

