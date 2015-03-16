# -*- octave -*-

function ps2png(plotFileName)
  system(["(ps2raster -A -Tg -E250 -P " plotFileName ... 
	  ";rm " plotFileName ")&"]);
endfunction