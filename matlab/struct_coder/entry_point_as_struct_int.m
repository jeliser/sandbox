% Please let us not have to get to this: https://www.mathworks.com/help/simulink/slref/coder.ceval.html

% This is the entry point method for codegen
function [y] = entry_point_as_struct_int(scenario, iter, num_cmds) %#codegen
  y = NaN(iter, num_cmds);
  for i = 1:scenario.iterations
    for j = 1 : num_cmds
      % A bunch of codegen compliant code is here
    
      % This is the codegen statement under test
      y(i, j) = lookup(scenario.commands(j).func_as_int, scenario.commands(j).arg);
    
      % A bunch of codegen compliant code is here
    end
  end
end

% This will codegen, and obviously it really sucks to have to hand define all this stuff, but function_handles as
% entry point arguements OR dynamic function handle references do not work in codegen.  Only statically defined
% function handles will work (eg: compile time substitutions)
%
% NOTE: This function handler would likely be a part of your simulation framework initialization.  There is no
% reason for a developer to have to maintain this listing.  This should be generated from a directory listing
% of functions that you can call.
function y = lookup(i, varargin) %#codegen
  % Preallocate the return value
  y = nan; % Need to talk about what function return values are ... codegen doesn't support dynamic return values

  % This comparision lookup cannot be done using an array indexing.  The codegen process requires statically defined
  % code paths, so you must define it as an if/elseif sequence.
  % TODO: Are there codegen alternatives to this that might be more efficent?
  if i == 1
    y = computeSquare(varargin{:});
  elseif i == 2
    y = computeCube(varargin{:});
  elseif i == 3
    y = computeSquareRoot(varargin{:});
  elseif i == 4
    y = addValues(varargin{:});
  end
end

