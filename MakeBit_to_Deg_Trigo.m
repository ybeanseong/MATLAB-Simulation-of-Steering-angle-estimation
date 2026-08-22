function [A1_Deg_x,A1_Deg_y,T1_Deg_x,T1_Deg_y,Diff_Deg_x,Diff_Deg_y,verdet] = MakeBit_to_Deg_Trigo(A1RawData,T1RawData)

[t_r,~] = size(A1RawData);

A1_Deg_x = zeros(t_r,1);
A1_Deg_y = zeros(t_r,1);
T1_Deg_x = zeros(t_r,1);
T1_Deg_y = zeros(t_r,1);
Diff_Deg_x = zeros(t_r,1);
verdet = zeros(t_r,1);

for i = 1:t_r

    A1RawData_Temp = bitshift(bitand(A1RawData(i),268369920),-16);
    T1RawData_Temp = bitshift(bitand(T1RawData(i),268369920),-16);
    Diff_Temp = bitand(T1RawData(i),4095);

    strack= 7585*(A1RawData_Temp-1)/256;
    ptrack = 4*(T1RawData_Temp-2);
    
    if ptrack < 0 
       ptrack = ptrack + 16368
    elseif ptrack > 16368
       ptrack = ptrack - 16368
    end
    Diff_Temp = (6.5-(13*(Diff_Temp-8)/4079))*(1480)/(13);
    
    A1_Deg = strack*(1/409.2);
    T1_Deg = ptrack*(1/409.2);
    ver_diff_temp = (A1_Deg)-(T1_Deg);
    ver_diff_temp = ver_diff_temp;
    ver_idx = mod((15/8)*(ver_diff_temp),37);

    verdet(i) = abs((8*(ver_idx)+4)-ver_diff_temp); 


    
    A1_Deg_x(i) = cos((2*pi/296)*A1_Deg);
    A1_Deg_y(i) = sin((2*pi/296)*A1_Deg);
    T1_Deg_x(i) = cos((2*pi/40)*T1_Deg);
    T1_Deg_y(i) = sin((2*pi/40)*T1_Deg);
   
    Diff_Deg_x(i) = cos((2*pi/1480)*Diff_Temp);
    Diff_Deg_y(i) = sin((2*pi/1480)*Diff_Temp);
end
end
