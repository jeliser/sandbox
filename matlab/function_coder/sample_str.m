% Please let us not have to get to this: https://www.mathworks.com/help/simulink/slref/coder.ceval.html

% sample is the entry point function
function y = sample_str(str, x) %#codegen
  y = lookup(str, x);
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


