clc; clear all;
a = importdata('projector-values', '\t', 1);
heightdiff = [];

for index = 0:max(a.data(:,2))
    cpoint = a.data(a.data(:,2)==index,:);

    for i = 1:(size(cpoint,1)-1)
        dh = cpoint(i+1,1) - cpoint(i,1);
        dd = norm(cpoint(i+1,5:6)) - norm(cpoint(i,5:6));
        tth = (dd/dh);

        difh =  norm(cpoint(i,5:6))/tth - cpoint(i,1);
        heightdiff = [heightdiff, difh];
    end
end

heightdiff = heightdiff(~(isinf(heightdiff)|isnan(heightdiff)));
hcor = mean(heightdiff)
hist(heightdiff)

a.data(:,1) = a.data(:,1) + hcor;


for i = 1:size(a.data,1)
    a.data(i,logical([1,0,0,0,1,1]))
end
