% This is calling the methods doing a string comparision.  This does codegen, but obviously string comparisions
% are a costly operation.  This was included for comparision to other callback mechanisms.  Not really advised
% to do it this way.

% This is the entry point method for codegen
function [y] = entry_point_as_struct_str(scenario) %#codegen
  y = NaN(scenario.iterations, numel(scenario.commands));
  for i = 1:scenario.iterations
    for j = 1 : numel(scenario.commands)
      % A bunch of codegen compliant code is here
    
      % This is the codegen statement under test
      y(i, j) = lookup(scenario.commands(j).func_as_str, scenario.commands(j).arg);
    
      % A bunch of codegen compliant code is here
    end
  end
end

function y = lookup(str, varargin) %#codegen
  y = nan;
  if isequal(str, func2str(@computeSquare))
    y = computeSquare(varargin{:});
  elseif isequal(str, func2str(@computeCube))
    y = computeCube(varargin{:});
  elseif isequal(str, func2str(@computeSquareRoot))
    y = computeSquareRoot(varargin{:});
  elseif isequal(str, func2str(@addValues))
    y = addValues(varargin{:});
  end
end


