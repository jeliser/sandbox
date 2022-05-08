% Please let us not have to get to this: https://www.mathworks.com/help/simulink/slref/coder.ceval.html

% sample is the entry point function
function y = sample_func(func, x) %#codegen
  f = func;
  y = f(x);
end

