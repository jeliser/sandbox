% Structures fields are pre-defined lengths during codegen, so you can't change the size of the structures
% and you will need to have all possible fields loaded into the structure.

% Add the utility lookup path
addpath('utils');

% Get a listing of all the available functions that we can use as callbacks
files = dir('utils\**\*.m');
funcs = cell(numel(files), 1);
iterations = 10000;

% Grab all of the available functions
for i = 1 : numel(files)
  funcs{i} = str2func(strrep(files(i).name, '.m', ''));
end

% Expand and iterate over all the functions and call them in different ways and time them
%for j = 10:25:100
for j = 2

  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Let's build configurations that we'll iterate through.
  cmds = struct([]);
  for i = numel(funcs):-1:1 % By iterating backwards, the struct array is preallocated
    cmds(i).func_as_int = i;
    cmds(i).arg = randi([3 25]);
    for j = 1 : randi([1 5], 1)
      cmds(i).(char(floor(26*rand(1, randi([5 10], 1))) + 97)) = rand();
    end
  end
  % Replicate the configurations to be a larger array
  cmds = repmat(cmds, 1, j);
  scenario = build_configuration('Mission Configuration Integers', iterations, cmds);
  s_int = scenario; % Save this off for easier access to the codegen methods

  fprintf('\nFunction lookup structure is %d elements long (%d iterations)\n', numel(cmds), iterations)

  % Call the functions using integer indexing
  tic
  entry_point_as_struct_int(scenario);
  t = toc;
  fprintf('  Calling as int: %0.3f seconds\n', t)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Let's build configurations that we'll iterate through.
  cmds = struct([]);
  for i = numel(funcs):-1:1 % By iterating backwards, the struct array is preallocated
    cmds(i).func_as_func = funcs{i};
    cmds(i).arg = randi([3 25]);
    for j = 1 : randi([1 5], 1)
      cmds(i).(char(floor(26*rand(1, randi([5 10], 1))) + 97)) = rand();
    end
  end
  % Replicate the configurations to be a larger array
  cmds = repmat(cmds, 1, j);
  scenario = build_configuration('Mission Configuration Functions', iterations, cmds);
  s_func = scenario; % Save this off for easier access to the codegen methods

  % Call the functions using function callbacks
  tic
  entry_point_as_struct_func(scenario);
  t = toc;
  fprintf('  Calling as func: %0.3f seconds\n', t)

  %%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Let's build configurations that we'll iterate through.
  cmds = struct([]);
  for i = numel(funcs):-1:1 % By iterating backwards, the struct array is preallocated
    cmds(i).func_as_str = func2str(funcs{i});
    cmds(i).arg = randi([3 25]);
    for j = 1 : randi([1 5], 1)
      cmds(i).(char(floor(26*rand(1, randi([5 10], 1))) + 97)) = rand();
    end
  end
  % Replicate the configurations to be a larger array
  cmds = repmat(cmds, 1, j);
  scenario = build_configuration('Mission Configuration Strings', iterations, cmds);
  s_str = scenario; % Save this off for easier access to the codegen methods

  % Call the functions using string indexing
  tic
  entry_point_as_struct_str(scenario);
  t = toc;
  fprintf('  Calling as string: %0.3f seconds\n', t)
end

%%
function scenario = build_configuration(name, iterations, commands)
  scenario = struct();
  scenario.name = name;
  scenario.iterations = iterations;
  scenario.commands = commands;
end
