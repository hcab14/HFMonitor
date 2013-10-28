all: perseus.ihx perseus.disas perseus_orig.disas

# 
# 

# --code-size 0x3c00 
# --xram-size 0x0200 
# --xram-loc 0x3c00 
# --no-pack-iram 
# --code-size 0x3c00 --xram-size 0x0200 --xram-loc 0x3c00 
# 
# --xram-size 0x0200 --xram-loc 0x3c00 
# 
# --code-loc 0x0080 
# -Wl"-b DSCR_AREA=0x9000" 
# -Wl"-b INT2JT=0x8000"

perseus.ihx: dscr.rel perseus.rel
	sdcc -mmcs51 \
		--no-pack-iram \
		--no-xinit-opt \
		--stack-loc 0x004c --code-loc 0x0000 --xram-loc 0x80 --iram-size 0x80 \
		--code-size 0x3c00 --xram-size 0x0200 --xram-loc 0x3c00 \
		-Wl"-b DSCR_AREA=0x0900" -Wl"-b INT2JT=0x0800" -Wl"-b INT4AV=0x0800" \
		-o perseus.ihx \
		perseus.rel dscr.rel fx2.lib -L../../FX2/fx2lib/lib/

perseus.rel: perseus.c
	sdcc -mmcs51 \
		--no-pack-iram \
		--no-xinit-opt \
		--stack-loc 0x004c --code-loc 0x0000 --xram-loc 0x80 --iram-size 0x80 \
		--code-size 0x3c00 --xram-size 0x0200  --xram-loc 0x80 --xram-loc 0x3c00 \
		-Wl"-b DSCR_AREA=0x0900" -Wl"-b INT2JT=0x0800" -Wl"-b INT4AV=0x0800" \
		-c -I ../../FX2/fx2lib/include -I "" perseus.c -o perseus.rel

dscr.rel: dscr.a51
	sdas8051 -logs dscr.a51

perseus.disas: perseus.ihx regs.list
	~/bin/mcs51-disasm.pl -rj -fl \
		--const-area 0x0900 0x09AB \
		--const-area 0x0E43 0x0E4C \
		--name-list regs.list \
		perseus.ihx > perseus.disas

#		--const-area 0x0800 0x08B7
#		--const-area 0x0E6B 0x0E8E
#		--const-area 0x07FC 0x07FF
#		--const-area 0x0900 0x09AB
perseus_orig.disas:
	 ~/bin/mcs51-disasm.pl -v 10 -rj -fl \
		--const-area 0x0900 0x09AB \
		--const-area 0x0E43 0x0E4C \
		--name-list regs.list \
		../config/perseus/perseus.hex >  perseus_orig.disas


regs.list: 
	python2.7 proc_regs.py ../../FX2/fx2lib/include/fx2regs.h

clean: 
	rm -f perseus.{asm,ihx,lst,map,mem,rel,rst,sym,disas,lk} dscr.{sym,rel,lst}