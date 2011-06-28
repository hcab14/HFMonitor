# -*- octave -*-

function xx=down_conversion(fc)
  fns=glob("wav/ThuB*wav");
#  fns={"wav/ThuB_002_800_01.wav"};

  fns={"/Volumes/DATA1/PERSEUS/ExternalWAVE/ThuB_002.wav"};

  f0    = 800e3;
  decim = 50;      # 40 kHz i/q wav samples 
  bw    = 38e3;    # +- 19 kHz

  osc = @(f,fs, N) exp(2*pi*i*f/fs *[1:N]');

  x={};
  N=2*10^6;
  counter=1;
  for n=1:length(fns)
    printf("processing %s ...\n", fns{n});fflush(stdout);
    for j=1:60
      printf(" ... %3d ...\n", j);fflush(stdout);
      [y,fs,bits]=wavread(fns{n}, [N*(j-1)+1 N*j]);
      b = fir2(5000, [0 bw/fs bw/fs 1], [1 1 0 0], 2, 10);
      b=conv(b,b);
      z=fftfilt(b, (y(:,1)+1i*y(:,2)) .* osc(f0-fc,fs,size(y,1)));
      fs /= decim;
      x{counter} = z(1:decim:end);
      counter += 1;
    endfor
  endfor

  xx=cell2mat(x');
  wavwrite([real(xx) imag(xx)], fs, 16, sprintf("wav/iq_%.0fkHz.wav", fc/1000));
endfunction
