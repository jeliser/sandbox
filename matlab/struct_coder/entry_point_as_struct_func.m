% This is the close to the current implementation of function_handles as entry point arguements.
% Dynamic function handle references (eg: cell arrays of function handles) are also in the code base
% and do not work in codegen.

% This is the entry point method for codegen
function [y] = entry_point_as_struct_func(scenario) %#codegen
  y = NaN(scenario.iterations, numel(scenario.commands));
  for i = 1:scenario.iterations
    for j = 1 : numel(scenario.commands)
      % A bunch of codegen compliant code is here
    
      % This is the codegen statement under test
      y(i, j) = scenario.commands(j).func_as_func(scenario.commands(j).arg);
    
      % A bunch of codegen compliant code is here
    end
  end
end

