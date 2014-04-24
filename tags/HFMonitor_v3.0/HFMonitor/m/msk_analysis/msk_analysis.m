# -*- octave -*-

function b=msk_analysis
  b1 = read_bits("../../DataKRK/MSK_DHO_bits/y2013-m11-d07.txt");
  ##  b2 = read_bits("../../DataKRK/MSK_NAA_bits/y2013-m11-d07.txt");
  N=100000;
  for i=1:N
    c(i) = corrcoef(b1(1:N), b1(1+i:N+i));
  endfor
  plot(c);
endfunction

function b = read_bits(fn)
  fid = fopen(fn);
  b=[];
  while 1
    line = fgetl(fid);
    if line == -1
      break;
    endif
    if line(1) != '#' 
      [v,c] = sscanf(line(28:end), "%02x", [1 Inf]);
      b(end+1:end+8*c) = reshape(dec2bin(v,8)', 1, 8*c)=="1";
    endif
  endwhile
  fclose(fid);
endfunction