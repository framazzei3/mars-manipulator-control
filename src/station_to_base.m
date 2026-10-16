function P_B = station_to_base(P_S, p)
%STATION_TO_BASE Positions from the station frame {S} to the base frame {B}.
%   P_B = STATION_TO_BASE(P_S, p) for P_S of size 3xN.

P_B = p.R_BS * P_S + p.p_BS;
end
