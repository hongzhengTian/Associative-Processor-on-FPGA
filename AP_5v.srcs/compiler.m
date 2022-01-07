fid = fopen('ISA.txt','r');
fid_copy = fopen('ISA.txt','r');
lines = 0;
while ~feof(fid_copy)
fgetl(fid_copy);
lines = lines + 1;
end
fid_2 = fopen('ISA_Bin.txt','w');

for i = 1 : lines
tline = fgetl(fid);
newStr = strrep(tline, '0x','');
newStr = strrep(newStr, ';','');
newStr = strrep(newStr, '0','0000');
newStr = strrep(newStr, '1','0001');

newStr = strrep(newStr, 'RESET','000100000000000000000000000000');
newStr = strrep(newStr, 'RET','001000000000000000000000000000');
newStr = strrep(newStr, 'LOADRBR','0100');
newStr = strrep(newStr, 'LOADCBC','0101');
newStr = strrep(newStr, 'STORERBR','0110');
newStr = strrep(newStr, 'STORECBC','0111');
newStr = strrep(newStr, 'COPY','1000');
newStr = strrep(newStr, 'PRINT','11010000000000');
newStr = strrep(newStr, 'ADD','100100000000000000000000000000');
newStr = strrep(newStr, 'SUB','101000000000000000000000000000');
newStr = strrep(newStr, 'TSC','101100000000000000000000000000');
newStr = strrep(newStr, 'ABS','110000000000000000000000000000');
newStr = strrep(newStr, 'M_A M_B','00000001100000000000000000');
newStr = strrep(newStr, 'M_A M_R','00000001110000000000000000');
newStr = strrep(newStr, 'M_B M_A','00000010010000000000000000');
newStr = strrep(newStr, 'M_B M_R','00000010110000000000000000');
newStr = strrep(newStr, 'M_R M_A','00000011010000000000000000');
newStr = strrep(newStr, 'M_R M_B','00000011100000000000000000');
newStr = strrep(newStr, 'M_A','01');
newStr = strrep(newStr, 'M_B','10');
newStr = strrep(newStr, 'M_R','11');

newStr = strrep(newStr, '2','0010');
newStr = strrep(newStr, '3','0011');
newStr = strrep(newStr, '4','0100');
newStr = strrep(newStr, '5','0101');
newStr = strrep(newStr, '6','0110');
newStr = strrep(newStr, '7','0111');
newStr = strrep(newStr, '8','1000');
newStr = strrep(newStr, '9','1001');
newStr = strrep(newStr, 'A','1010');
newStr = strrep(newStr, 'B','1011');
newStr = strrep(newStr, 'C','1100');
newStr = strrep(newStr, 'D','1101');
newStr = strrep(newStr, 'E','1110');
newStr = strrep(newStr, 'F','1111');
newStr = strrep(newStr, ' ','');

fprintf(fid_2, '%s\n', newStr);
end
fclose(fid);
fclose(fid_2);