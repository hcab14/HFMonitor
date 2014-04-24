# -*- octave -*-

function [aa,bb]=spec
  M=40;
  N=100;
  dt=0.01;
  f0=10.1;

  aa=[];
  for i=1:100
    [a,f_off]=sim_spec(M,N,dt,f0);    
    bb=a;
    aa(end+1,:)=mean(mod(diff(a),1));
  endfor
endfunction

function [a,f_off]=sim_spec(M,N,dt,f0)
  t=[0:M*N-1]*dt;
  x=exp(i*2*pi * f0*t);
  x+= randn(size(x)) * 0.1;
  a=[];
  for i=1:M
    [s,f]=spec(x, 1+(i-1)*N, N, dt);
#    semilogy(f, abs(s), '*-', 'markersize', 2);

    [m,im]=max(abs(s))
    im=11;
#    im=91;
    f_off=(f0-f(im))/(f(2)-f(1));
    a(end+1,:)=arg(s(im-2:im+2))/(2*pi);
  endfor
endfunction

function [s,f]=spec(x, off, N, dt)
  f = (0:N-1)/dt/N;
  s = fft(x(off:off+N-1).*w(N)); 
#  s = fft(x(off:off+N-1));
endfunction

function y=w(N)
  y=0.54 - 0.46*cos(2.*pi*[0:N-1]/(N-1));
endfunction