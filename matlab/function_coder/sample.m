% Please let us not have to get to this: https://www.mathworks.com/help/simulink/slref/coder.ceval.html

% sample is the entry point function
function y = sample(i, x) %#codegen
  y = callback_matlab(i, x);

%   if coder.target('MATLAB')
%     y = callback_matlab(i, x);
%   else
%     y = 0;
%     % Executing in generated code, call C function foo
%     coder.updateBuildInfo('addSourceFiles','computeSquare.c');
%     %coder.cinclude('foo.h');
%     funcs = {
%       'computeSquare'
%     };
%     y = coder.ceval(i, x); %#codegen
%   end
% 
%   %fx = lookup(i)
%   %y = fx(x);
end

function y = callback_matlab(i, x) %#codegen
  % Executing in MATLAB
  y = lookup(i, x);
end

function y = lookup(i, x) %#codegen
  y = nan;
  if i == 1
    f = @computeSquare;
    y = f(x);
  elseif i == 2
    f = @computeCube;
    y = f(x);
  elseif i == 3
    f = @computeSquareRoot;
    y = f(x);
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

