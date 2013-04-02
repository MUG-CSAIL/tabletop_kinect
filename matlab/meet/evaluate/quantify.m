function count = quantify(y, ystar, cat)
%
% Args:
% - y: true label.
% - ystar: estimated label.
% - cat: categories.
count = zeros(1, 4); % [tp fp tn fn]
if ismember(ystar, cat) 
  if ismember(y, cat) 
    count(1)  = 1;
  else
    count(2) = 1;
  end
else
  if ~ismember(y, cat) 
    count(3) = 1;
  else
    count(4) = 1;
  end
end
end


