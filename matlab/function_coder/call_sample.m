
% The predefined list of functions
funcs = {@computeSquare; @computeCube; @computeSquareRoot};

% Expand and iterate over all the functions and call them in different ways and time them
for j = 10:25:100
%for j = 2
  % Let's build a entry point calling structure
  s = struct();
  s.iterations = 1000000;
  
  % Transform the function handles supporting arrays
  s.funcs_as_funcs = cell(size(funcs));
  s.funcs_as_str = cell(size(funcs));
  for i = 1 : numel(funcs)
    s.funcs_as_funcs{i} = funcs{i};
    s.funcs_as_str{i} = func2str(funcs{i});
  end
  s.funcs_as_funcs = repmat(s.funcs_as_funcs, j, 1);
  s.funcs_as_str = repmat(s.funcs_as_str, j, 1);

  fprintf('\nFunction lookup structure is %d elements long (%d iterations)\n', numel(s.funcs_as_funcs), s.iterations)

  % Call the functions using integer indexing
  tic
  for i = 1:iterations
    sample_int(mod(i, numel(s.funcs_as_str))+1, 5);
  end
  t = toc;
  fprintf('  Calling as int: %0.3f seconds\n', t)
  
  % Call the functions using function callbacks
  tic
  for i = 1:iterations
    sample_func(s.funcs_as_funcs{mod(i, numel(s.funcs_as_funcs))+1}, 5);
  end
  t = toc;
  fprintf('  Calling as func: %0.3f seconds\n', t)

  % Call the functions using string indexing
  tic
  for i = 1:iterations
    sample_str(s.funcs_as_str{mod(i, numel(s.funcs_as_str))+1}, 5);
  end
  t = toc;
  fprintf('  Calling as string: %0.3f seconds\n', t)
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