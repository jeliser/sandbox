% Please let us not have to get to this: https://www.mathworks.com/help/simulink/slref/coder.ceval.html

% sample is the entry point function
function y = sample_int(i, x) %#codegen
  y = lookup(i, x);
end

function y = lookup(i, x) %#codegen
  y = nan;
  if i == 0
    y = computeSquare(x);
  elseif i == 1
    y = computeCube(x);
  elseif i == 2
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

