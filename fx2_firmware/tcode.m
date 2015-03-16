# -*- octave -*-


function y = tcode(x)
  r6 = bitshift(x, -8);
  r7 = bitand(x, 0xFF);
  _r6 =r6; _r7 =r7;

  cy = 0;
  a = 0xAA;
  [a, cy] = subb(a, r7, cy);
  r7 = a;

  a=0x09;
  [a, cy] = subb(a, r6, cy);

  [a, r7] = swap(a, r7);
  [a, cy] = add(a, 2, 0);
  [a, r7] = swap(a, r7);

  [a, cy] = add(a, 0, cy);
  r6 = a;

#  assert(r6<256);
#  assert(r6>=0);
#  assert(r7<256);
#  assert(r7>=0);
  y = bitor(bitshift(r6,8), r7);
endfunction

function [xc, cy]=subb(a, b, cy)
  xa = bitget(a, 1:8);
  xb = bitget(b, 1:8);
  xc = zeros(1,8);
  for i=1:8
    xx = xa(i) - xb(i) - cy;
    xc(i) = mod(xx,2);
    cy = (xx < 0);
  endfor
  xc = sum(xc .* bitshift(1,[0:7]));
endfunction

function [xc, cy]=add(a, b, cy)
  xa = bitget(a, 1:8);
  xb = bitget(b, 1:8);
  xc = zeros(1,8);
  for i=1:8
    xx = xa(i) + xb(i) + cy;
    xc(i) = mod(xx,2);
    cy = (xx > 1);
  endfor
  xc = sum(xc .* bitshift(1,[0:7]));
endfunction



