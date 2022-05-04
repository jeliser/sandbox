% Please let us not have to get to this: https://www.mathworks.com/help/simulink/slref/coder.ceval.html

function y = sample(i, x) %#codegen
  fx = lookup(i)
  y = fx(x);
end

function func = lookup(i)
  funcs = {
    @computeSquare
    @computeCube
    @computeSquareRoot
  };
  
  func = funcs{i};
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

