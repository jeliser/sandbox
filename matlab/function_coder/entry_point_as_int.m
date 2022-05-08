% Please let us not have to get to this: https://www.mathworks.com/help/simulink/slref/coder.ceval.html

% This is the entry point method for codegen
function y = entry_point_as_int(i, x) %#codegen
  % A bunch of codegen compliant code is here

  % This is the codegen statement under test
  y = lookup(i, x);

  % A bunch of codegen compliant code is here
end

% This will codegen, and obviously it really sucks to have to hand define all this stuff, but function_handles as
% entry point arguements OR dynamic function handle references do not work in codegen.  Only statically defined
% function handles will work (eg: compile time substitutions)
%
% NOTE: This function handler would likely be a part of your simulation framework initialization.  There is no
% reason for a developer to have to maintain this listing.  This should be generated from a directory listing
% of functions that you can call.
function y = lookup(i, x) %#codegen
  y = nan; % Need to talk about what function return values are ... codegen doesn't support dynamic return values

  % This comparision lookup cannot be done using an array indexing.  The codegen process requires statically defined
  % code paths, so you must define it as an if/elseif sequence.
  % TODO: Are there codegen alternatives to this the might be more efficent?
  if i == 0
    y = computeSquare(x);
  elseif i == 1
    y = computeCube(x);
  elseif i == 2
    y = computeSquareRoot(x);
  end
end

