
path = 'C:\Users\elisertech\Desktop\combined';
dirs = dir(path);

for i = 3:length(dirs)
%for i = 3:5
    filename = sprintf('%s\\%s\\%s.mat', path, dirs(i).name, dirs(i).name);
    fprintf('Processing %s\n', filename);
    tmp = load(filename);
    quick_plot(dirs(i).name, tmp.(subsref(fieldnames(tmp),substruct('{}',{1}))));
end


