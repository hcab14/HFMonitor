# -*- octave -*-

function [zf,z,theta,ud,uf,fs]=demod

  if (1)
    [zf,z,theta,ud,uf,fs]=demodulate_sam("wav/test_9800.wav", ...
				       9800e3, ...
				       9830e3-1000,
				       3000);
  else 
#    [zz,zf,theta,f,e,fs]=demodulate("wav/test_8470_8478.wav", ...
    [zf,z,theta,ud,uf,fs]=demodulate_sam("wav/test_8470_8478.wav", ...
				 8470e3, ...
				 8478e3-200, ...
				 2800);  
  endif
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

