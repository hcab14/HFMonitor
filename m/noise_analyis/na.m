# -*- octave -*-

function [x,z,z0]=na(jj)
  addpath("../demod/");

  [y,fs,bits] = wavread("../../DataKRKIQ/DC_298000_DGPS/y2013-m11-d08_H12M50.wav");
  z = y(:,1)+1i*y(:,2);

  zz=z(1:5000);
  n=length(zz);
  f=[0:n/2 -n/2+1:-1]/n*2*fs/2;
  b=f>-100&f<100;
  semilogy(f(b),abs(fft(zz))(b), '-');

  f(b)(find(abs(fft(zz))(b)>1))
  ## -18 Hz

  osc = @(f,fs, N) exp(2*pi*i*f/fs *[0:N-1]');
  fc = 18+jj*50;
  z .*= osc(fc, fs, length(z));
  z0=z;
  
  bw=25;
  b= fir2(5000,[0 bw/fs 1.1*bw/fs 1], [1 1 0 0 ], 2, 20);
  z=fftfilt(b,z);

  z=z(1:5000*30);

  b=f>-100&f<100;
  semilogy(f(b),abs(fft(zz))(b), '-');

  decim=200;
  zd=z(1:decim:end);
  fs /= decim;
  n=length(zd);
  f=[0:n/2 -n/2+1:-1]/n*2*fs/2;
  b=f>-10 & f<10;
  semilogy(f(b),abs(fft(zd))(b));

  xx=diff(sort(f(find(abs(fft(zd))>0.07))));
  xx(xx>1)

  plot(real(zd(200:400)))
  x=zd(200:end);
  
  if 0
    f=[0:2500 -2499:-1];
    for i=1:300
      s(:,i)=abs(ifft(z(1+(i-1)*5000:i*5000)));
    endfor;
    plot(f,log10(mean(s')));

    z=z(1:5000*300);
    
  
  
    
    xi=1/sqrt(2);
    for j=1:30
      pll(j)  = pll_init(xi, .5, -18+(j-1)*50,      fs);
    endfor
    for n=1:5000*2
      for j=1:30
	pll(j)  = pll_1(z(n), pll(j));
	theta(j,n) = pll(j).theta;
	f1(j,n)    = pll(j).f1;
      endfor
    endfor
    plot(theta'+t' * (18-[0:29]*50)*2*pi)
  endif

endfunction