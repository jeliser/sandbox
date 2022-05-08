% This is the close to the current implementation of function_handles as entry point arguements.
% Dynamic function handle references (eg: cell arrays of function handles) are also in the code base
% and do not work in codegen.

% This is the entry point method for codegen
function y = entry_point_as_func(func, x) %#codegen
  % A bunch of codegen compliant code is here

  % This is the codegen statement under test
  y = func(x);

  % A bunch of codegen compliant code is here
end

