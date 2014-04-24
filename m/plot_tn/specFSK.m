# -*- octave -*-

function [s,f]=specFSK
  N=1000;
  dt=0.01;
  f0=10;

  t=[0:N-1]*dt;
  b=1:N/4;
  x(b)=exp(i*2*pi * f0*t(b));
  b=N/4+1:N/2;
  x(b)=exp(i*2*pi * 1* f0*t(b));
  b=N/2+1:3*N/4;
  x(b)=exp(i*2*pi * 1* f0*t(b));
  b=3*N/4+1:N;
  x(b)=exp(i*2*pi * 1* f0*t(b));

  x=2*exp(i*2*pi * 1* f0*t);
  x+= 0*randn(size(x)) * .001;
  [s,f]=spec(x, 1, N, dt);
  semilogy(f, abs(s), '*-', 'markersize', 2);

  sum(abs(s)([101 201 301 401]))

endfunction

function [s,f]=spec(x, off, N, dt)
  f = (0:N-1)/dt/N;
  s = fft(x(off:off+N-1).*w(N)) / N;# sum(w(N)); 
#  s = fft(x(off:off+N-1));
endfunction

function y=w(N)
  y=0.54 - 0.46*cos(2.*pi*[0:N-1]/(N-1));
#  y=ones(1,N);
endfunction