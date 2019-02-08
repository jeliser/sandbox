clear all;
close all;
format long;
filename = 'sample_short.raw';
fd = fopen(filename);

entities = [];
ids = [];

xmin = 999999999999999999999999;
xmax = 0;

%% Read in the binary data
while not(feof(fd))
  id = fread(fd, 1, 'uint32');
  if ~any(numel(id))
    break
  end
  
  if isempty(find(ids == id))
    ids = [ids, id];

    entity(1).id = id;
    entity(1).type = [fread(fd, 1, 'uint8')];
    entity(1).time = [fread(fd, 1, 'uint64') / 1.0E9];

    xmin = min(xmin, entity(1).time(end));
    xmax = max(xmax, entity(1).time(end));

    entities = [entities, entity];
  else
    idx = find(ids == id);
    entities(idx).type = [entities(idx).type, fread(fd, 1, 'uint8')];
    entities(idx).time = [entities(idx).time, fread(fd, 1, 'uint64') / 1.0E9];
        
    xmin = min(xmin, entities(idx).time(end));
    xmax = max(xmax, entities(idx).time(end));
  end
  
  fread(fd, 1, 'uint8');  % Throw away newline
end

fclose(fd);

% For now, filter out the timing measurements that are not periodic
for i = numel(entities):-1:1
  if numel(entities(i).time(entities(i).type == 2)) < 5
    entities(i) = [];
  end
end

%% Plot the loaded structures
figure('Name', sprintf('Timing data starting from %s', strftime('%Y-%m-%d %H:%M:%S', gmtime(round(xmin)))), 'NumberTitle', 'Off');
for i = 1:numel(entities)
  subplot(numel(entities), 1, i);
  stairs((entities(i).time - xmin), entities(i).type);
  ylim([0.5, 2.5]);
  xlim([0, xmax-xmin]);
  title(sprintf('Descriptor ID: %d', entities(i).id));
  xlabel('Elapsed time since start (secs)');
  
  %% Change the label to enumerations
  y_labels = {'STOP', 'START'};
  set(gca, 'Ytick', [1, 2],'YTickLabel', y_labels);
end
saveas(gcf, 'timing.png')

%% Generate some metrics from the performance data
if numel(entities) > 0
  disp(sprintf('Processing: %s -> Total Duration: %d (secs)', filename, xmax - xmin))
  disp('-----------------------------------------------')
end
for i = 1:numel(entities)
  starts = entities(i).time(entities(i).type == 2) * 1.0E3;
  stops  = entities(i).time(entities(i).type == 1) * 1.0E3;
  
  start_diff = diff(starts);
  execution_diff = stops - starts;
  disp(sprintf('  ID: %6d -> Number of samples: %d, Duration: %d (secs)', entities(i).id, numel(entities(i).time), entities(i).time(end) - entities(i).time(1)))
  disp(sprintf('    Timing Diff -> min: %0.3f (msec), max: %0.3f (msec), mean: %0.3f (msec), std: %0.3f (msec)', min(start_diff), max(start_diff), mean(start_diff), std(start_diff)))
  disp(sprintf('                -> max: %0.1f (Hz), min: %0.1f (Hz), mean: %0.1f (Hz)', 1000/min(start_diff), 1000/max(start_diff), 1000/mean(start_diff)))
  disp(sprintf('    Execution   -> min: %0.3f (msec), max: %0.3f (msec), mean: %0.3f (msec)', min(execution_diff), max(execution_diff), mean(execution_diff)))
  disp('-----------------------------------------------')
end