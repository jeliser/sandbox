
funcs = {@computeSquare; @computeCube; @computeSquareRoot};
iterations = 1000000;

% Transform the function handles supporting arrays
funcs_as_funcs = cell(size(funcs));
funcs_as_str = cell(size(funcs));
for i = 1 : numel(funcs)
  funcs_as_funcs{i} = funcs{i};
  funcs_as_str{i} = func2str(funcs{i});
end


% Expand and iterate over all the functions and call them in different ways and time them
%for s = 10:25:100
for s = 1
  arr_funcs = repmat(funcs_as_funcs, s, 1);
  arr_strs = repmat(funcs_as_str, s, 1);
  fprintf('\nFunction lookup structure is %d elements long (%d iterations)\n', numel(arr_funcs), iterations)

  % Call the functions using integer indexing
  tic
  for i = 1:iterations
    sample_int(mod(i, numel(arr_strs))+1, 5);
  end
  t = toc;
  fprintf('  Calling as int: %0.3f seconds\n', t)
  
  % Call the functions using function callbacks
  tic
  for i = 1:iterations
    sample_func(arr_funcs{mod(i, numel(arr_funcs))+1}, 5);
  end
  t = toc;
  fprintf('  Calling as func: %0.3f seconds\n', t)

  % Call the functions using string indexing
  tic
  for i = 1:iterations
    sample_str(arr_strs{mod(i, numel(arr_strs))+1}, 5);
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