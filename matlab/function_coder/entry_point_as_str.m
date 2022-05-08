% This is calling the methods doing a string comparision.  This does codegen, but obviously string comparisions
% are a costly operation.  This was included for comparision to other callback mechanisms.  Not really advised
% to do it this way.

% This is the entry point method for codegen
function y = entry_point_as_str(str, x) %#codegen
  % A bunch of codegen compliant code is here

  % This is the codegen statement under test  
  y = lookup(str, x);

  % A bunch of codegen compliant code is here
end

function y = lookup(str, x) %#codegen
  y = nan;
  if isequal(str, func2str(@computeSquare))
    y = computeSquare(x);
  elseif isequal(str, func2str(@computeCube))
    y = computeCube(x);
  elseif isequal(str, func2str(@computeSquareRoot))
    y = computeSquareRoot(x);
  end
end


