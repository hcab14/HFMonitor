clear all; clc;

fs = 2000;
N = 3 * fs;
Ts = 1 / fs;

t = (0 : N - 1) * Ts;

f = 10;

%Pll characteristics
Wn = 2 * 2 * pi() * f;
lambda = 0.707;

%Transfer function constants
K = Wn / (2 * lambda);
Kf = 1 / (2 * Wn * lambda);

%Continuous
%Transfer
subplot(4, 2, 1)
[h, w] = freqs([K 0], [Kf 1], 100);
plot(w, abs(h)); title('PLL Transfer'); xlabel('Angular Frequency'); ylabel('Magnitude');

%Entire PLL
subplot(4, 2, 2)
[h, w] = freqs([0 0 K], [Kf 1 K], 100);
plot(w, abs(h)); title('Entire PLL'); xlabel('Angular Frequency'); ylabel('Magnitude');

%

input = sin(2 * pi() * f * t);
output = sin(2 * pi() * f * t + pi() / 16);

%Averager for getting rid of phase detector harmonic
pd_alpha = 1 / fs;
pd_y = 1;

%Phase integrator for output frequency generation
op = 0;

%Low pass filter
x = 0;
y = output(1);
xd = 0;
yd = sin(2 * pi() * f * (t(1) - 1/f) + pi() / 8);
%K = 4 * pi() * pi() / (T * T);
%Kf = exp(-4 * sqrt(2) * pi() * Ts / T);

pd_plot = [0];
pd_y_plot = [0];
x_plot = [0];
y_plot = [0];

for i = 1 : length(t) - 1
    pd = input(i) * output(i);
    
    pd_y = pd_y * (1 - pd_alpha) + pd_alpha * pd;
    
    %xd = 4 * pi() * pi() * x / (T * T);
    x = K * pd_y;
    y = x + yd;
    yd = Kf * y;
    
    y = 1 * pd_y;
    
    op = op + 2 * pi() * (9 + y) * Ts;
    output(i + 1) = sin(op);
    
    pd_plot = [pd_plot pd];
    pd_y_plot = [pd_y_plot pd_y];
    x_plot = [x_plot x];
    y_plot = [y_plot y];
end

subplot(4, 2, 3);
hold off;
plot(t, input);
hold on;
plot(t, output, 'r');
subplot(4, 2, 4);
hold off;
plot(t, pd_plot);
hold on;
plot(t, pd_y_plot, 'r');

subplot(4, 2, 5);
plot(t, x_plot);

subplot(4, 2, 6);
plot(t, y_plot);