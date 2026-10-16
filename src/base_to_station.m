function P_S = base_to_station(P_B, p)
%BASE_TO_STATION Positions from the base frame {B} to the station frame {S}.
%   P_S = BASE_TO_STATION(P_B, p) for P_B of size 3xN.

P_S = p.R_BS.' * (P_B - p.p_BS);
end
