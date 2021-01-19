
Fs = 50;                    % Hertz

[b,a] = butter(4, 0.025);

figure(1);
freqz(b,a);

x = [zeros(1,100), 50*ones(1,100), -20*ones(1,200), zeros(1,200), 50*ones(1,10), zeros(1,200), -20*ones(1,100), 50*ones(1,100), zeros(1,100)];
t = ( 0:(length(x)-1) )/Fs;

y = filter(b,a,x); % apply the recursive filter to x to produce y

figure(2);
plot(t,x);
hold on
plot(t,y);
hold off

%%

y2 = x;
for i = 2:length(y2)
    [ y2(i) ] = slewRateFilter( y2(i), y2(i-1) );
end

figure(3);
plot(t,x);
hold on
plot(t,y2);
hold off