# -*- octave -*-

function [y,init]=lfsr

  init=uint32(sum(bitshift(1,[1 3 4])));
  init=uint32(1);
  
  x=init;
  y(1)=x;
  printf("%3d %03x %d\n", i, x, y(1));fflush(stdout);
  for i=2:2000
    x=_lfsr2(x);
    y(i) = x;
    printf("%3d %s %03x %d\n", i, dec2bin(double(x)), x, y(i));fflush(stdout);
    if x == init
      break
    endif
  endfor

endfunction

function x=_lfsr(x)
  bit=mod(sum(bitget(x,[1 3])),2);
  x=bitshift(x,-1);
  x=bitset(x,5,bit);
endfunction

function x=_lfsr2(x)
  bit=mod(sum(bitget(x,[1 5])),2);
  x=bitshift(x,-1);
  x=bitset(x,9,bit);
endfunction