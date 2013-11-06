# -*- octave -*-

function [in,in0,theta,f,e]=costas_()
  N=5000;
  fs=500;
  fc=10;
  phase0 = 0.5;
  n=4;
  psig = floor(rand(1,round(N*fc/fs))*n);
  s = exp(i*2*pi/n*psig(1+floor([0:N-1]*fc/fs)));

  in0 = exp(i*2*pi*[0:N-1]*(fc+2)/fs + i*phase0);
#  in = in0.*s;
  in = in0 .* cos(2*pi*[0:N-1]*1/fs) + ...
      0.7* in0 .* cos(2*pi*[0:N-1]*1.2/fs);
  # add noise
  in = awgn(in, 20);
  
  in *= .01;
  in = agc(in, fs, 1, 2);

  alpha=.6;
  beta=alpha**2/4
#  beta=0.01;
  [theta,f,e]=costas_1(2, in,fc,fs,alpha,beta);
  
endfunction

function z=agc(z, fs, tfall, trise) 
  f = 1;
  alpha = 1/(fs*tfall);
  beta = 1/(fs*trise);
  for n=1:length(z)
    if abs(z(n)) < f
      f = (1-alpha)* f + alpha*abs(z(n));
    else
      f = (1-beta)* f + beta*abs(z(n));
    endif
    z(n) *= .5/f;
  endfor
endfunction

function [theta,f,e]=costas_1(n, in,fc,fs,alpha,beta,nd)  
  ##Initialization 
  f(1) = fc;
  theta(1)=0;
  x=0;
  mu=.1;
  mup=.05;

#  nd=4;
  dl=zeros(1,nd);
  for I=2:length(in)    
    xerr=in(I-1)*exp(-1i*theta(I-1));
    x=(1-mu)*x+ mu*xerr;
    ## error estimate:
    if (n==2)
      err = real(x) * imag(x);
    elseif (n==4)
#      err = sign(real(x))*imag(x) - sign(imag(x))*real(x);
#      dl(1) = sign(real(x))*imag(x);
      dl(1) = sign(imag(x))*real(x);
      err = sign(real(x))*imag(x)- dl(end);
      dl(2:end) = dl(1:end-1);
    else
      error("n!=2 and n!=4")
    endif
    ## phase detector
    e(I) = (1-mup)*e(I-1) + mup * err;
    ## frequency update     
    f(I)     = f(I-1) + beta*e(I);
    ## phase update
    theta(I) = theta(I-1) + alpha*e(I) + f(I)/fs*2*pi;
  endfor
endfunction