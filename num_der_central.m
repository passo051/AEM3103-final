function [fp_num] = num_der_central(x, y)
drl = length(x);
fp_num = nan(1, drl); % NaN - means Not a Number (An array of size 1 by drl)

for i = 2:drl-1
    fp_num(i) = (y(i+1) - y(i-1))/(x(i+1) - x(i-1));

end

% Handle last element
fp_num(drl) = (y(drl) - y(drl-1))/(x(drl) - x(drl-1));
% Backward Difference
fp_num(i) = (y(drl) - y(drl-1))/(x(drl) - x(drl-1));

end