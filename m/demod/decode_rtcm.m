# -*- octave -*-

function decode_rtcm(bbits)
  ## find preamble
  preamble = [0 1 1 0 0 1 1 0];
  idx = find(conv(bbits-.5, preamble-.5)==2);
  idx = [idx find(conv(bbits-.5, ~preamble-.5)==2)];
  idx = sort(idx);

  idx -= 8;
  idx(idx==0)=[];
  idx(idx>length(bbits)-60)=[];
  idx
  length(bbits)
  ## search for valid frames
  ii=1;
  length(idx)-1
  while ii < (length(idx)-1)
    n=idx(ii);
    [D, success] = parity_check(bbits(n+1-30:n), ...
				bbits(n+1:n+30));
    i=0;
    if (success)
      i2=i3=i4=i5=i6=i7=i8=i9=i10=i11=0;
      next=0;
      success=true;
      i=0;
      p=[];
      while success
	i +=1;
	[D, success] = parity_check(bbits(n+1-30+30*i:n+30*i),
				    bbits(n+1+30*i:n+30+30*i));
	##fprintf("i=%d success=%d index=%d\n", i, success, n+1+30*i);fflush(stdout);
	
	if success
	  preamble = D( 1: 8)*bitshift(1,[ 7:-1:0]');
	  if preamble==102
	    p.preamble  = preamble;
	    p.msg_type  = D( 9:14)*bitshift(1,[ 5:-1:0]');
	    p.refid     = D(15:24)*bitshift(1,[ 9:-1:0]');
	    p.parity    = D(25:30)*bitshift(1,[ 5:-1:0]');
	    i1=0;i2=1;
	  elseif i2
	    p.mod_z     = D( 1:13)*bitshift(1,[12:-1:0]');
	    p.seq_nr    = D(14:16)*bitshift(1,[ 2:-1:0]');
	    p.n_data    = D(17:21)*bitshift(1,[ 4:-1:0]');
	    fprintf(stdout, 
		    "preamble=%03d msg_type=%02d refid=%03d parity=%2d mod_z=5%d seq_nr=%2d n_data=%2d\n",
		    p.preamble, p.msg_type, p.refid, p.parity, p.mod_z, p.seq_nr, p.n_data); 
				#	  fflush(stdout);
	    if p.msg_type == 9
	      i3=1;
	    endif
	    if p.msg_type == 3
	      i8=1;
	    endif
	    i2=0;
	  elseif i3
	    Xi3=1;
	    p.sat_id(1) = D( 4: 8)*bitshift(1,[ 4:-1:0]');
	    p.prc(1)    = D( 9:24)*bitshift(1,[15:-1:0]');
	    i3=0;i4=1;
	  elseif i4
	    Xi4=1;
	    p.rrc(1)    = D( 1: 8)*bitshift(1,[ 7:-1:0]');
	    p.iod(1)    = D( 9:16)*bitshift(1,[ 7:-1:0]');
	    p.sat_id(2) = D(20:24)*bitshift(1,[ 4:-1:0]');
	    i4=0;i5=1;
	  elseif i5;
	    Xi5=1;
	    p.prc(2)    = D( 1:16)*bitshift(1,[15:-1:0]');
	    p.rrc(2)    = D(17:24)*bitshift(1,[ 7:-1:0]');
	    i5=0;i6=1;
	  elseif i6;
	    Xi6=1;
	    p.iod(2)    = D( 1: 8)*bitshift(1,[ 7:-1:0]');
	    p.sat_id(3) = D(12:16)*bitshift(1,[ 4:-1:0]');
	    p.prc(3)    = D(17:24)*bitshift(1,[15:-1:8]');
	    i6=0;i7=1;
	  elseif i7;
	    Xi7=1;
	    p.prc(3)   += D( 1: 8)*bitshift(1,[ 7:-1:0]');
	    p.rrc(3)    = D( 9:16)*bitshift(1,[ 7:-1:0]');
	    p.iod(3)    = D(17:24)*bitshift(1,[ 7:-1:0]');
	    for j=1:3
	      fprintf(stdout, "\t sat_id=%02d prc=%6d rrc=%6d iod=%3d\n", p.sat_id(j), p.prc(j), p.rrc(j), p.iod(j));
	    endfor
	    i7=0;
	  elseif i8
	    Xi8=1
	    p.ref_x  = D( 1:24)*bitshift(1,[31:-1:8]');
	    i8=0;i9=1;
	  elseif i9
	    Xi9=1;
	    p.ref_x += D( 1: 8)*bitshift(1,[ 7:-1: 0]');
	    p.ref_y  = D( 9:24)*bitshift(1,[15:-1: 0]');
	    i9=0;i10=1;
	  elseif i10
	    Xi10=1;
	    p.ref_y += D( 1:16)*bitshift(1,[15:-1: 0]');
	    p.ref_z  = D(17:24)*bitshift(1,[31:-1:24]');
	    i10=0; i11=1;
	  elseif i11
	    Xi11=1;
	    p.ref_z += D( 1:24)*bitshift(1,[23:-1: 0]');	    
	    fprintf(stdout, "x=%d y=%d z=%d\n", p.ref_x, p.ref_y, p.ref_z);
	    i11=0;
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