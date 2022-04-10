data = readtable("HW4-1.xls", "ReadVariableNames", false);
inputs = data{:, 1:2};
output = data{:, 3};

coef = inv(inputs' * inputs) * inputs' * output;
disp(coef)