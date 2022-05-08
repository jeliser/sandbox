% Add the utility lookup path
addpath('utils');

% The predefined list of functions
funcs = {@computeSquare; @computeCube; @computeSquareRoot};
iterations = 1000000;

% Expand and iterate over all the functions and call them in different ways and time them
for j = 10:25:100
%for j = 2
  % Let's build configurations that we'll iterate through.
  s = struct([]);
  for i = numel(funcs):-1:1 % By iterating backwards, the struct array is preallocated
    % Transform the function handles supporting arrays
    s(i).funcs_as_func = funcs{i};
    s(i).funcs_as_str = func2str(funcs{i});
  end
  % Replicate the configurations to be a larger array
  s = repmat(s, 1, j);

  fprintf('\nFunction lookup structure is %d elements long (%d iterations)\n', numel(s), iterations)

  % Call the functions using integer indexing
  tic
  for i = 1:iterations
    sample_int(mod(i, numel(s))+1, 5);
  end
  t = toc;
  fprintf('  Calling as int: %0.3f seconds\n', t)
  
  % Call the functions using function callbacks
  tic
  for i = 1:iterations
    sample_func(s(mod(i, numel(s))+1).funcs_as_func, 5);
  end
  t = toc;
  fprintf('  Calling as func: %0.3f seconds\n', t)

  % Call the functions using string indexing
  tic
  for i = 1:iterations
    sample_str(s(mod(i, numel(s))+1).funcs_as_str, 5);
  end
  t = toc;
  fprintf('  Calling as string: %0.3f seconds\n', t)
end
