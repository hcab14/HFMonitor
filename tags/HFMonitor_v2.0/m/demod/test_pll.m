# -*- octave -*-

function [in,theta,f,e,s]=test_pll
  N=10000;
  fs=5000;
  fc=512.3;

  phase0 =pi/2;

  ## AM
  s = cos(2*pi*[0:N-1]*3.5/fs) ...
  +sin(2*pi*[0:N-1]*4.5/fs) ...
  +cos(2*pi*[0:N-1]*5./fs) ...
  +sin(2*pi*[0:N-1]*7./fs) ...
  +cos(2*pi*[0:N-1]*8.1/fs) ...
  +sin(2*pi*[0:N-1]*9.7/fs) ;
  s *=10;
  ## BPSK
  n=4;
  psig = floor(rand(1,round(N*fc/fs))*n);
  s = exp(i*2*pi/n*psig(1+floor([0:N-1]*fc/fs)));

  in = s.*exp(i*2*pi*[0:N-1]*fc/fs + i*phase0);
  in = awgn(in,20);

#  osc = @(f, N) exp(2*pi*i*f/fs *[1:N]);
#  in .*= osc(-fc/2,length(in));

  alpha=0.1;
  beta=alpha**2/4*1.0;

 [theta,f,e]=costas(in,fc,fs,alpha,beta,"QPSK",n);
endfunction

