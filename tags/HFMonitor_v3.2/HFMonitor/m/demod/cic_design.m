# -*- octave -*-

function [f,b,bcic,bp,bt]=cic_design(N, ... # CIC order
				     M, ... # diff. delay
				     R, ... # reduction factor
				     fc)    # passband edge  
  L = 64; # fir filter order
  Fo = R*fc;
  p  = 2e3;
  s  = 0.25/p;
  fp = [0:s:Fo];
  fs = (Fo+s):s:0.5;
  f  = [fp fs]*2;

  Mf = [1 abs(M*R*sin(pi*fp(2:end)/R)./sin(pi*M*fp(2:end))).**N zeros(1,length(fs))];
  b  = fir2(L,f,Mf);
  b /= norm(b);  

  brec = ones(1,R*M);
  tmpb = brec;
  
  for k=1:N-1
    tmpb = conv(brec, tmpb);
  end;
  bcic = tmpb;
  bcic=bcic/norm(bcic);

  bp = upsample(b, R);
  bt = conv(bcic, bp);


endfunction