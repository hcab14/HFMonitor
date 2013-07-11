# -*- octave -*-

function decode_rtcm(bbits)
  ## find preamble
  preamble = [0 1 1 0 0 1 1 0];
  idx = find(conv(bbits-.5, preamble-.5)==2);
  idx = [idx find(conv(bbits-.5, ~preamble-.5)==2)];
  idx = sort(idx);

  idx -= 8;

  ## search for valid frames
  ii=1
  while ii < length(idx)-1
    n=idx(ii);
    [D, success] = parity_check(bbits(n+1-30:n), ...
				bbits(n+1:n+30));
    i=0;
    if (success)
      i2=i3=0; next=0;
      success=true;
      i=0;
      while success
	i +=1;
	[D, success] = parity_check(bbits(n+1-30+30*i:n+30*i),
				    bbits(n+1+30*i:n+30+30*i));
#	fprintf("i=%d success=%d index=%d\n", i, success, n+1+30*i);fflush(stdout);
	
	if success
	  preamble = D( 1: 8)*bitshift(1,[ 7:-1:0]');
	  if preamble==102
	    p.preamble  = preamble;
	    p.msg_nr    = D( 9:14)*bitshift(1,[ 5:-1:0]');
	    p.refid     = D(15:24)*bitshift(1,[ 9:-1:0]');
	    p.frame_len = D(17:21)*bitshift(1,[ 4:-1:0]');
	    i2=1;
	  elseif i2
	    p.mod_z     = D( 1:13)*bitshift(1,[12:-1:0]');
	    p.seq_nr    = D(14:16)*bitshift(1,[ 2:-1:0]');
	    i3=1; i2=0;
	  elseif i3
	    p.prn       = D( 4: 8)*bitshift(1,[ 4:-1:0]');
	    p.prc       = D( 9:24)*bitshift(1,[15:-1:0]');
	    fprintf(stdout, 
		    "preamble=%d msg_nr=%d refid=%d frame_len=%2d mod_z=%d seq_nr=%d prn=%d prc=%d\n",
		    p.preamble, p.msg_nr, p.refid, p.frame_len, p.mod_z, p.seq_nr, p.prn, p.prc); 
				#	  fflush(stdout);
	    i3=0;
	  endif
	else 
	  break 
	endif
      endwhile
      ii +=1;
    else # no success
      n+1+30*i;
      ii=find(idx>n+1+30*i)(1);
    endif
  endwhile
endfunction

function [D,success]=parity_check(bits_old, bits)
  _xor=@(x) mod(sum(x),2);
  for i=0:0
    _bits=bits;
    if i 
      _bits(i) = bitxor(1, _bits(i)); 
    endif
    D(1:24) = bitxor(bits_old(30), _bits(1:24));
    D(25)   = _xor([bits_old(29) D([ 1  2  3  5  6 10 11 12 13 14 17 18 20 23    ]) ]);
    D(26)   = _xor([bits_old(30) D([ 2  3  4  6  7 11 12 13 14 15 18 19 21 24    ]) ]);
    D(27)   = _xor([bits_old(29) D([ 1  3  4  5  7  8 12 13 14 15 16 19 20 22    ]) ]);
    D(28)   = _xor([bits_old(30) D([ 2  4  5  6  8  9 13 14 15 16 17 20 21 23    ]) ]);
    D(29)   = _xor([bits_old(30) D([ 1  3  5  6  7  9 10 14 15 16 17 18 21 22 24 ]) ]);
    D(30)   = _xor([bits_old(29) D([ 3  5  6  8  9 10 11 13 15 19 22 23 24       ]) ]);
    
    success= length(find(_bits(25:30) == D(25:30))) == 6;
    if success break; endif
  endfor
endfunction