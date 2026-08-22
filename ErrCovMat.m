function P_prdt = ErrCovMat(P_old,F,Q_old)

% declare return variable
P_prdt = zeros(4,4);

% calculate return value
P_prdt = F*P_old*F' + Q_old;


end
