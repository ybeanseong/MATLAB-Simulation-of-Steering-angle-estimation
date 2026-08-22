function P_ret = PEstimn(H,Kg,P)

P_ret = (eye(4) - Kg*H)*P;

end
