# -*- octave -*-

function [in,in0,theta,f,e]=costas_()
  N=5000;
  fs=500;
  fc=10;
  phase0 = 0.5;
  psig = (rand(1,N*fc/fs)>0.5)*pi;
  in0 = exp(i*2*pi*[0:N-1]*(fc+2)/fs + i*phase0);
#  in = in0.*exp( i*pi*psig(1+floor([0:N-1]*fc/fs)));
  in = in0 .* cos(2*pi*[0:N-1]*1/fs) + ...
      0.3* in0 .* cos(2*pi*[0:N-1]*1.5/fs);
  # add noise
  in = awgn(in, 20);
  
  alpha=.6;
  beta=alpha**2/4
#  beta=0.01;
  [theta,f,e]=costas_1(in,fc,fs,alpha,beta);
  
endfunction

function [theta,f,e]=costas_1(in,fc,fs,alpha,beta)  
  ##Initialization 
  f(1) = fc;
  theta(1)=0;
  x=0;
  mu=1;
  mup=.05;
  for I=2:length(in)    
    xerr=in(I-1)*exp(-1i*theta(I-1));
    x=(1-mu)*x+ mu*xerr;
    ## phase detector
    e(I) = (1-mup)*e(I-1) + mup * real(x) * imag(x);
    ## frequency update     
    f(I)     = f(I-1) + beta*e(I);
    ## phase update
    theta(I) = theta(I-1) + alpha*e(I) + f(I)/fs*2*pi;
  endfor
endfunction