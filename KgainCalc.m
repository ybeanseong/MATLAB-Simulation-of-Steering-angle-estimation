function K_gain_ret = KgainCalc(Pkknext, H, R)

K_gain_ret= Pkknext *(H')*inv(H*Pkknext*H' + R);
end
