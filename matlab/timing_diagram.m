rows = csvread('hello.raw');

id_select = 0;
select_row = rows(rows(:,1) == id_select, :);

subplot(2, 1, 1);
plot(select(:,2))

subplot(2, 1, 2);
plot(select(:,3) - select(1, 3))
