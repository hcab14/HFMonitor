# -*- octave -*-

function [in,in0,theta,f,e]=costas()
  N=10000;
  fs=500;
  fc=10;
  phase0 = 0.5;
  psig = (rand(1,N*fc/fs)>0.5)*pi;
  in0 = exp(i*2*pi*[0:N-1]*fc/fs + i*phase0);
  in = exp(i*2*pi*[0:N-1]*fc/fs + i*phase0 + i*pi*psig(1+floor([0:N-1]*fc/fs)));
  # add noise
  in = awgn(in, 20);
  
  alpha=0.1
  beta=alpha**2/4
  beta=0.01
  [theta,f,e]=costas(in,fc,fs,alpha,beta);
  
endfunction

function [theta,f,e]=costas(in,fc,fs,alpha,beta)  
  ##Initialization 
  theta(1)= .0;
  f(1) = fc;
  x=0;
  mu=.7;
  for I=2:length(in)    
    x=(1-mu)*x+ mu*imag(log( in(I-1) * exp(-i*theta(I-1)) ));
    e(I-1) = +x;

    theta(I) = theta(I-1) + alpha*e(I-1) + beta*f(I-1);
    f(I)     = f(I-1) + beta*e(I-1);
  endfor
endfunction