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

%% Sample function callbacks

function y = computeSquare(x)
  y = x.^2;
end

function y = computeCube(x)
  y = x.^3;
end

function y = computeSquareRoot(x)
  y = sqrt(x);
end

