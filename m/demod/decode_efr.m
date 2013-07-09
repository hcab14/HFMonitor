# -*- octave -*-

##------------------------------------------------------------------------------
## telegram stucture:
# H L L H [number | ...] A1 A2 ... Checksum E
# H=104, E=22
# L .. length of data 5:5+L-1
# checksum= sum(b(5:5+L-1) mod 256
##------------------------------------------------------------------------------

##------------------------------------------------------------------------------
## time telegrams:
# examples:
#  104    10    10   104   149     0     0     0    96    31    17   144    12    10   203    22
#  104    10    10   104   165     0     0     0   140    31    17   144    12    10     7    22
#  104    10    10   104   181     0     0     0   184    31    17   144    12    10    67    22
#  104    10    10   104   197     0     0     0   228    31    17   144    12    10   127    22
#
# *  9: bit1-6: second
# * 10: minute
# * 11: hour
# * 12: bit1-3: day in week, bit4-8: day in month
# * 13: month
# * 14: year
##------------------------------------------------------------------------------

function [bytes,parity,bbits,ns]=decode_efr(bits)
  ## search for 0.. start of message
  n=1;
  n_msg=0;
  byte_counter=1;
  state="Null";
  bytes={};
  bbits={};
  parity={};
  ns={};
  while n<length(bits)-11
    switch state
      case "Null"
	if bits(n) == 1
	  n += 1;
	else
	  state="Init";
	endif
      case "Init"
	if bits(n+11) == 0 && bits(n+10) == 1
	  state="Msg";
	  n_msg += 1;
	  ns{n_msg} = n;
	  byte_counter=1;
	else
	  state="Init";
	  n+=1;
	endif
      case "Msg"
	if bits(n) == 0 && bits(n+10) == 1
	  parity{n_msg}(byte_counter)=mod(sum(bits(n+1:n+8)),2) == bits(n+9);
	  bytes{n_msg}(byte_counter) = bits(n+1:n+8)'*bitshift(1,[0:1:7]');
	  bbits{n_msg}(byte_counter,:) = bits(n+1:n+8)';
	  n += 11;
	  byte_counter += 1;
	else
	  state="Null";
	  n+=1;
	endif
    endswitch
  endwhile
endfunction