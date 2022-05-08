% Add the utility lookup path
addpath('utils');

% Get a listing of all the available functions that we can use as callbacks
files = dir('utils\**\*.m');
funcs = cell(numel(files), 1);
iterations = 1000000;

% Grab all of the available functions
for i = 1 : numel(files)
  funcs{i} = str2func(strrep(files(i).name, '.m', ''));
end

% Expand and iterate over all the functions and call them in different ways and time them
%for j = 10:25:100
for j = 2
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
    entry_point_as_struct_int(mod(i, numel(s))+1, 5);
  end
  t = toc;
  fprintf('  Calling as int: %0.3f seconds\n', t)
  
  % Call the functions using function callbacks
  tic
  for i = 1:iterations
    entry_point_as_struct_func(s(mod(i, numel(s))+1).funcs_as_func, 5);
  end
  t = toc;
  fprintf('  Calling as func: %0.3f seconds\n', t)

  % Call the functions using string indexing
  tic
  for i = 1:iterations
    entry_point_as_struct_str(s(mod(i, numel(s))+1).funcs_as_str, 5);
  end
  t = toc;
  fprintf('  Calling as string: %0.3f seconds\n', t)
end
