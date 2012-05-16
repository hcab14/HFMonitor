# -*- octave -*-

function [z,zs,fs,bits,bitsE,bitsO,btE,btO,xE,xO,ct,st,theta0,theta1,theta2]=demod_p(fn,off)
  [y,fs,bits]=wavread(fn);

  z=y(:,1)+1i*y(:,2);

  f0= 1e3*sscanf(fn(end-9:end-7), "%f")

  baud= 100;
  fc= 308e3;
  fc= 283.5e3;
  fc= 289.5e3;
  fc= 303.5e3;
  fc= 298.5e3;
#  fc= 19.6e3-25;
#  fc= 20.27e3;

  fc= 23.4e3;
  baud= 100;
  fc=20.9e3; # 100bd
  baud=200;
  fc=20.27e3;

  bw= 3*baud;

  osc = @(f,fs, N) exp(2*pi*i*f/fs *[0:N-1]');
  z .*= osc(f0-fc, fs, length(z));

  b= fir2(5000,[0 bw/fs bw/fs 1], [1 1 0 0 ], 2, 20);
  b=conv(b,b);
  z=fftfilt(b,z);
  zs=z;
  
  decim=80;
  z=z(1:decim:end);
  fs /= decim;

  z=z';

  [bits,bitsE,bitsO,btE,btO,xE,xO,ct,st,theta0,theta1,theta2]=demod_msk(z, baud, fs);

  bbits=bitxor(bitsE, bitsO);

  decode_rtcm(bbits);

endfunction

function [zf,z,theta,ud,uf,fs]=demodulate_sam(filename, f0, fc, bw)
  [y,fs,bits] = wavread(filename);

  y=y(40000:end,:);
  y=y(1:500000,:);
  z=y(:,1)+i*y(:,2);

  osc = @(f, N) exp(2*pi*i*f/fs *[0:N-1]');

  delta_f = fc-f0

  z .*=  osc(-delta_f,length(z));

  b=fir1(2000,bw/fs);
  zf=fftfilt(b,z);
  
  zf=zf(4000:end);

  decim = 8;
  zf = zf(1:decim:end);
  fs /= decim;
  zf /= mean(abs(zf));
  
  xi=2**-.5;
  dwl=100;
#  b=fir1(500,100./fs);
#  zf_=fftfilt(b,zf);

  [theta,ud,uf]=pll(zf, xi, dwl, fc, fs);

endfunction

function [zz,zf,theta,f,e,fs]=demodulate(filename, f0, f, mode, bw, n)
  [y,fs,bits] = wavread(filename);

  y=y(40000:end,:);
  y=y(1:100000,:);
  z=y(:,1)+i*y(:,2);

  osc = @(f, N) exp(2*pi*i*f/fs *[0:N-1]');
  make_spec = @(z,N) 20*log10(abs(fft(z(1:N))));

  N=10000;
  freq = 0*f0 + [0:N/2-1 -N/2:-1]*fs/N;

  delta_f = f-f0

  z .*=  osc(-delta_f,length(z));

#  b=abs(freq)<20000;
#  plot(freq(b)/1000, make_spec(z,N)(b));

  b=fir1(2000,bw/fs);
  zf=fftfilt(b,z);

  zf=zf(4000:end);

#  b=abs(freq)<10000;
#  plot(freq(b)/1000, make_spec(zf,N)(b));

  decim = 8;
#  zf .*=  osc(4000,length(z));
  zf = zf(1:decim:end);

  fs /= decim;

  zf /= mean(abs(zf));

  alpha=0.1;
  beta=alpha**2/4;

  tic;
  [theta,f,e,zf]= costas(zf,0,fs,alpha,beta,mode,n);
  toc;

  zz=zf.*exp(i*theta)';
endfunction

