% This is the close to the current implementation of function_handles as entry point arguements.
% Dynamic function handle references (eg: cell arrays of function handles) are also in the code base
% and do not work in codegen.

% This is the entry point method for codegen
function [y] = entry_point_as_struct_func(scenario) %#codegen
  y = NaN(size(scenario));
  for i = 1 : numel(scenario)
    % A bunch of codegen compliant code is here
  
    % This is the codegen statement under test
    y(i) = scenario(i).as_func(scenario(i).arg);
  
    % A bunch of codegen compliant code is here
  end
end

