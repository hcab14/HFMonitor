/**
 * Copyright (C) 2009 Ubixum, Inc. 
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 **/
#include <stdio.h>

#include <fx2ints.h>
#include <fx2regs.h>
#include <fx2macros.h>
#include <serial.h>
#include <delay.h>
#include <autovector.h>
#include <lights.h>
#include <setupdat.h>
#include <eputils.h>

#define SYNCDELAY SYNCDELAY3

__data __at 0x00 unsigned char r0; 
__data __at 0x01 unsigned char r1; 
__data __at 0x02 unsigned char r2; 
__data __at 0x03 unsigned char r3; 
__data __at 0x04 unsigned char r4; 
__data __at 0x05 unsigned char r5; 
__data __at 0x06 unsigned char r6; 
__data __at 0x07 unsigned char r7; 

__data __at 0x08 unsigned char r0_1; 
__data __at 0x09 unsigned char r1_1; 
__data __at 0x0A unsigned char r2_1; 
__data __at 0x0B unsigned char r3_1; 
__data __at 0x0C unsigned char r4_1; 
__data __at 0x0D unsigned char r5_1; 
__data __at 0x0E unsigned char r6_1; 
__data __at 0x0F unsigned char r7_1; 

__data __at 0x10 unsigned char r0_2; 
__data __at 0x11 unsigned char r1_2; 
__data __at 0x12 unsigned char r2_2; 
__data __at 0x13 unsigned char r3_2; 
__data __at 0x14 unsigned char r4_2; 
__data __at 0x15 unsigned char r5_2; 
__data __at 0x16 unsigned char r6_2; 
__data __at 0x17 unsigned char r7_2; 

__sbit __at 0x00 bit0x00;
__sbit __at 0x01 bit0x01;
__sbit __at 0x02 bit0x02;
__sbit __at 0x03 bit0x03;
__sbit __at 0x04 bit0x04;
__sbit __at 0x05 bit0x05;
/* __sbit __at 0x06 bit0x06; */
/* __sbit __at 0x07 bit0x07; */

/* ACC bits */
__sbit __at 0xE0 bit0xE0;
__sbit __at 0xE1 bit0xE1;
__sbit __at 0xE2 bit0xE2;
/* __sbit __at 0xE3 bit0xE3; */
__sbit __at 0xE4 bit0xE4;
/* __sbit __at 0xE5 bit0xE5; */
__sbit __at 0xE6 bit0xE6;
__sbit __at 0xE7 bit0xE7;
__sbit __at 0xE8 bit0xE8;

 /* 32bit number: var21,22,23,24 */
__data __at 0x21 unsigned char var0x21;
__data __at 0x22 unsigned char var0x22;
__data __at 0x23 unsigned char var0x23;
__data __at 0x24 unsigned char var0x24;

__data __at 0x25 unsigned char var0x25;
__data __at 0x26 unsigned char var0x26;
__data __at 0x27 unsigned char var0x27;
__data __at 0x28 unsigned char var0x28;

__data __at 0x29 unsigned char var0x29;
__data __at 0x2A unsigned char var0x2A;
__data __at 0x2B unsigned char var0x2B;
__data __at 0x2C unsigned char var0x2C;

__data __at 0x2D unsigned char var0x2D;
__data __at 0x2E unsigned char var0x2E;
__data __at 0x2F unsigned char var0x2F;
__data __at 0x30 unsigned char var0x30;

// sleep for var0x31,32 ms
__data __at 0x31 unsigned char var0x31; // H
__data __at 0x32 unsigned char var0x32; // L

__data __at 0x33 unsigned char var0x33; //
__data __at 0x34 unsigned char var0x34; //

__data __at 0x35 unsigned char var0x35; // FLAG
__data __at 0x36 unsigned char var0x36; // ADDRH
__data __at 0x37 unsigned char var0x37; // ADDRL

__data __at 0x38 unsigned char var0x38;
__data __at 0x39 unsigned char var0x39;
__data __at 0x3A unsigned char var0x3A;

__data __at 0x3B unsigned char var0x3B;
__data __at 0x3C unsigned char var0x3C;

__data __at 0x3D unsigned char var0x3D; // ADDRH
__data __at 0x3E unsigned char var0x3E; // ADDRL
__data __at 0x3F unsigned char var0x3F; //

__data __at 0x40 unsigned char var0x40;
__data __at 0x41 unsigned char var0x41;

__data __at 0x41 unsigned char var0x42;
__data __at 0x41 unsigned char var0x43;
__data __at 0x41 unsigned char var0x44;
__data __at 0x41 unsigned char var0x45;
__data __at 0x41 unsigned char var0x46;

__data __at 0x47 unsigned char var0x47;
__data __at 0x48 unsigned char var0x48;
__data __at 0x49 unsigned char var0x49;
__data __at 0x4A unsigned char var0x4A;
__data __at 0x4B unsigned char var0x4B;

void function_000();
void function_001();
void function_002();
void function_003();
void function_004();
void function_005();
void function_006();
void function_007();
void function_008();
void function_009();
void function_010();
void function_011();
void function_012();
void function_014();
void function_015();
void function_017();
void function_020();
void function_021();
void function_023();
void function_030();
void function_031();
void label_221();
void label_179();

void main() {
  ENABLE_RESUME();

   /* Label_092 */
  ACC = 0;
  var0x2C = 0;
  var0x2B = 0;
  var0x2A = 0;
  var0x29 = 0;
  r3 = 0;
  r0 = 0;
  r2 = 0;
  r1 = 0;

  function_002();

  r6   = 0x09;  r7   = 0x00;
  r2_1 = r6;    r3_1 = r7;
  r2_2 = 0x09;  r3_2 = 0x12;
  r0_1 = 0x09;  r1_1 = 0x1C;
  r0_2 = 0x09;  r1_2 = 0x43;
  r4_2 = 0x09;  r5_2 = 0x6A;

  ACC  = r6;
  ACC &= 0xC0;
  if (ACC != 0) {
  label_093:
    var0x2D = 0x00;
    var0x2E = 0x80;
    var0x2F = r6;
    var0x30 = r7;
     /* 16 bit address operation; r6r7 = 0x09AA - r6r7 + 2 */
    __asm__("clr  C");
    __asm__("mov  A, #0xAA");
    __asm__("subb A, R7");
    __asm__("mov  R7, A");
    __asm__("mov  A, #0x09");
    __asm__("subb A, R6");
    __asm__("xch  A, R7");
    __asm__("add  A, #0x02");
    __asm__("xch  A, R7");
    __asm__("addc A, #0x00");
    __asm__("mov  R6, A");
    __asm__("clr  A");

    var0x28 = r7;
    var0x27 = r6;
    var0x26 = ACC;
    var0x25 = ACC;
    var0x24 = ACC;
    var0x23 = ACC;
    var0x22 = ACC;
    var0x21 = ACC;
  label_094:
    r7 = var0x28;
    r6 = var0x27;
    r5 = var0x26;
    r4 = var0x25;
    r3 = var0x24;
    r2 = var0x23;
    r1 = var0x22;
    r0 = var0x21;
    CY = 0;
    function_010();
    if (CY) {
      /* DPL = var0x30 + var0x24; */
      __asm__("mov  A, 0x30");
      __asm__("add  A, 0x24");
      __asm__("mov  DPL, A");
      /* DPH = var0x2F+ var0x23; */
      __asm__("mov  A, 0x2F");
      __asm__("addc A, 0x23");
      __asm__("mov  DPH, A");
      __asm__("movx A, @DPTR");
      r7 = ACC;
 
      /* DPL = var0x2E+ var0x24; */
      __asm__("mov  A, 0x2E");
      __asm__("add  A, 0x24");
      __asm__("mov  DPL, A");
      /* DPH = var0x2D+ var0x23; */
      __asm__("mov  A, 0x2D");
      __asm__("addc A, 0x23");
      __asm__("mov  DPH, A");
      ACC = r7;
      __asm__("movx @DPTR, A");

      /* var0x24 += 1; */
      __asm__("mov A, 0x24");
      __asm__("add A, #0x01");
      __asm__("mov 0x24, A");
      __asm__("clr A");

       /* var0x23 += CY; */
      __asm__("addc A, 0x23");
      __asm__("mov  0x23, A");
      __asm__("clr  A");

       /* var0x22 += CY; */
      __asm__("addc A, 0x22");
      __asm__("mov  0x22, A");
      __asm__("clr  A");

      /* var0x21 += CY; */
      __asm__("addc A, 0x21");
      __asm__("mov  0x21, A");
      goto label_094;
    }
  label_095:
    r2_1  = var0x2D;
    r3_1  = var0x2E;
    __asm__("mov  A, #0x00");
    __asm__("add  A, #0x80");
    __asm__("mov  R7, A");
    __asm__("mov  A, #0x09");
    __asm__("addc A, #0xFF");
    __asm__("mov  R6, A"); // 0x08 CY=1
    __asm__("clr  C");
    /* // R6R7 = 0x0880 */

    /* // R2R3<#2> -= R6R7 */
    __asm__("mov  A, 0x13"); // 0x13 = R3<#2>
    __asm__("subb A, R7");
    __asm__("mov  0x13, A");
    __asm__("mov  A, 0x12"); // 0x12 = R2<#2>
    __asm__("subb A, R6");
    __asm__("mov  0x12, A");
    __asm__("clr  C");

    /* // R4R5<#1> -= R6R7 */
    __asm__("mov  A, 0x0D"); // 0x0D = R5<#1>
    __asm__("subb A, R7");
    __asm__("mov  0x0D, A");
    __asm__("mov  A, 0x0C"); // 0x0C = R4<#1>
    __asm__("subb A, R6");
    __asm__("mov 0x0C, A");
    __asm__("clr  C");

    /* // R6R7<#1> -= R6R7 */
    __asm__("mov  A, 0x0F"); // 0x0F = R7<#1>
    __asm__("subb A, R7");
    __asm__("mov  0x0F, A");
    __asm__("mov  A, 0x0E"); // 0x0E = R6<#1>
    __asm__("subb A, R6");
    __asm__("mov  0x0E, A");
    __asm__("clr C");

    /* // R0R1<#1> -= R6R7 */
    __asm__("mov  A, 0x09"); // 0x09 = R1<#1>
    __asm__("subb A, R7");
    __asm__("mov  0x09, A");
    __asm__("mov  A, 0x08"); // 0x00 = R0<#1>
    __asm__("subb A, R6");
    __asm__("mov  0x08, A");
    __asm__("clr C");

    /* //  R0R1<#2> -= R6R7 */
    __asm__("mov  A, 0x11"); // 0x11 = R1<#2>
    __asm__("subb A, R7");
    __asm__("mov  0x11, A");
    __asm__("mov  A, 0x10"); // 0x10 = R0<#2>
    __asm__("subb A, R6");
    __asm__("mov  0x10, A");
    __asm__("clr C");

    /* //  R4R5<#2> -= R6R7 */
    __asm__("mov  A, 0x15"); // 0x15 = R5<#2>
    __asm__("subb A, R7");
    __asm__("mov  0x15, A");
    __asm__("mov  A, 0x14"); // 0x14 = R4<#2>
    __asm__("subb A, R6");
    __asm__("mov  0x14, A");
  }

 label_096:
  {
    unsigned char dummy=INT2JT;
  }
  {
    unsigned char dummy=INT4JT;
  }
  EUSB      = 1;    // EIE.0
  ERESI     = 1;    // EICON    |= 0x20;
  INTSETUP |= 0x09; // bmAV2EN | bmAV4EN
  USBIE    |= 0x3D; // bmEP0ACK | bmHSGRANT | bmURES | bmSUSP | bmSUTOK | bmSUDAV
  EA        = 1;
  bit0x04   = 1;
  function_014();
  USBCS    &= 0xF7; // bmHSM | 0x70 | bmNOSYNSOF | bmRENUM | bmSIGRESUME
  CKCON    &= 0xF8; // timers T2,1,0 use CLKOUT/4
  bit0x03   = 0;
  
 label_097:
  function_000();
  if (bit0x01) {
    function_001();
    bit0x01 = 0;
  }
 label_098:
  if (bit0x03 == 0)
    goto label_097;
  function_005();
  if (CY == 0)
    goto label_097;
  bit0x03 = 0;
 label_099:
  if (bit0x00 == 0) {
    if (WAKEUPCS & 0x80) {
      if (WAKEUPCS & 0x02)
        goto label_099;
    }
    if (WAKEUPCS & 0x40)
      if (WAKEUPCS & 0x01)
        goto label_099;
  }
 label_101:
  function_015();
  function_031();
  goto label_097;
}

void function_000() {
  if ((EP1OUTCS & 0x02) == 0)
    return;
  // Label_003:
  var0x35 = 0x01;
  var0x36 = 0xE7; // 0xE780: EP1OUTBUF[0]
  var0x37 = 0x80;
  r3 = var0x35;
  r2 = var0x36;
  r1 = var0x37;
  function_007();
/*         cjne	A, #0x09, Label_004 // CY = (A < #0x09) */
/* Label_004: */
/*         jc	Label_005 */
/*         ljmp	Label_044 */
  if (ACC >= 0x09)
    goto label_044;
  // jumptable
  // 0x00: (fpga_config) Label_014                                           -> label_044
  // 0x01: (fpga_reset)  Label_015                                           -> label_044
  // 0x02: (fpga_check)  Label_016-18              -> Label_041-42           -> Label_044
  // 0x03: (fpga_sio)    Label_019-21 -> Label_027 -> Label_034 -> Label_042 -> Label_044
  // 0x04: (fx2_port)    Label_028                                           -> label_044
  // 0x05: (??)          Label_022-27              -> Label_034 -> Label_042 -> Label_044
  // 0x06: (eeprom_read) Label_029-34                           -> Label_042 -> Label_044
  // 0x07: (??)          Label_035-42                                        -> Label_044
  // 0x08: (shutdown)    Label_043                                           -> Label_044
  switch (ACC) {
  case 0x00:
    function_023();
    goto label_044;
    break;
  case 0x01:
  label_015:
    function_017();
    goto label_044;
    break;
  case 0x02: // fpga_check
    /* label_016: */
    var0x38 = 0x01; var0x39 = 0xE7; var0x3A = 0xC0; //var0x39,0x3A=EP1INBUF
    // var0x35,36,37
    r3     = var0x35;
    ++var0x37; ACC = var0x37;
    r2     = var0x36;
    if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
    function_007(); // gptr = [(r1r2),r3] -> ACC
    r7 = ACC;
    r3 = var0x38;
    ++var0x3A; ACC = var0x3A;
    r2 = var0x39;
    if (ACC == 0) ++var0x39; --ACC; r1 = ACC;
    ACC = r7;
    function_011();
    goto label_041;    
    break;
  case 0x03:
    /*   /\* label_019: *\/ */
    /*   var0x38 = 0x01; */
    /*   var0x39 = 0xE7; // var0x39,0x3A = EP1INBUF */
    /*   var0x3A = 0xC0; */
    /*   ACC = EP1OUTBC; */
    /*   var0x31 = 0x00; */
    /*   var0x32 = ACC; */
    /*   r3 = var0x35; */
    /*   ++var0x37; */
    /*   ACC = var0x37; */
    /*   r2 = var0x36; */
    /*   if (ACC == 0) */
    /* 	++var0x36; */
    /*   --ACC; */
    /*   r1 = ACC; */
    /*   function_007(); */
    /*   r7 = ACC; */
    /*   r3 = var0x38; */
    /*   ++var0x3A; */
    /*   ACC = var0x3A; */
    /*   r2 = var0x39; */
    /*   if (ACC == 0) */
    /* 	++var0x39; */
    /*   --ACC; */
    /*   r1 = ACC; */
    /*   ACC = r7; */
    /*   label_179(); */
    /*   ACC = var0x32; */
    /*   ACC += 0xFF; */
    /*   r7 = ACC; */
    /*   ACC = var0x31; */
    /*   __asm__("addc A, #0xFF"); */
    /*   r6 = ACC; */
    /*   var0x40 = var0x38; */
    /*   var0x41 = var0x39; */
    /*   var0x42 = var0x3A; */
    /*   r3 = var0x35; */
    /*   r2 = var0x36; */
    /*   r1 = var0x37; */
    /*   function_006(); */
    /*   goto label_027; */
    /* label_027: */
    /*   ACC = EP1OUTBC; */
    /*   goto label_034; */
    /* label_034: */
    /*   __asm__("mov DPTR, #0xE68F"); //EP1INBC */
    /*   goto label_042; */
    /* label_042: */
    /*   __asm__("movx @DPTR, A"); SYNCDELAY; */
    /*   goto label_044;       	   */
    break;
  case 0x04:
  label_028:
    r3 = var0x35;
    r2 = var0x36;
    r1 = var0x37;
    __asm__("mov DPTR, #0x0001");
    function_008();
    IOE = ACC;
    break;
  case 0x05:
  label_022:
    break;
  case 0x06:
  label_029:
    var0x38 = 0x01;
    var0x39 = 0xE7; // 0xE7C0 EP1INBUF[0]
    var0x3A = 0xC0;
    r3 = var0x35;
    ++var0x37;
    r2 = var0x36;
    if (ACC == 0)
      ++var0x36;
    --ACC;
    r1 = ACC;
    function_007(); // [r2r3] -> ACC
    r7 = ACC;
    r3 = var0x38;
    ++var0x3A;
    r2 = var0x39;
    if (ACC == 0)
      ++var0x39;
    --ACC;
    r1 = ACC;
    ACC = r7;
    label_179();
    r3 = var0x35;
    ++var0x37;
    ACC = var0x37;
    r2 = 0x36;
    if (ACC == 0)
      ++var0x36;
    --ACC;
    r1 = ACC;
    function_007(); // [r2r3] -> ACC
    var0x33 = 0x00;
    var0x34 = ACC;
    ++var0x37;
    r2 = var0x36;
    if (ACC == 0) ++ var0x36;
    --ACC;
    r1 = ACC;
    function_007();
    r6 = ACC;
    ACC = r6;
    var0x33 |= ACC;
    r2 = var0x36;
    r1 = var0x37;
    function_007(); // [r2r3] -> ACC
    var0x31 = 0x00;
    var0x32 = ACC;
    r3 = var0x38;
    r2 = var0x39;
    r1 = var0x3A;
    var0x40 = var0x31;
    var0x41 = ACC;
    r5 = var0x34;
    r4 = var0x33;
    function_021();
    ACC = var0x32;
    ACC += 0x02;
    goto label_034;
    break;
  case 0x07:
  label_035:
    break;
  case 0x08:
    break;
  }

 label_034:
  __asm__("mov DPTR, #0xE68F"); // DPTR = EP1INBC
  goto label_042;
  
 label_042:
  __asm__("movx @DPTR, A"); SYNCDELAY;
  goto label_044;
  
  label_043:
  function_017();
  function_030();

 label_044:
  __asm__("clr A");
  EP1OUTBC = ACC;
  return;
}
// 0x00: fpga_config
void function_023() {
  ACC = EP1OUTBC;
  --ACC;
  r7 = ACC; // length of data
  __asm__("mov DPTR, #0xE781"); // DPTR = #EP1OUTBUF[1]
  DPL1 = 0x84;
  __asm__("label_224:      \n           \
            movx A, @DPTR; \n		\
            mov P1, A;     // IOB \n	\
            mov P2, #0x84; // IOC \n	\
            inc DPTR;      \n		\
            dec R7;        \n		\
            mov A, R7;     \n		\
            jnz label_224;");
  __asm__("mov P2, #0x84");
}

// fpga_reset:
void function_017() {
  IFCONFIG = 0x40; // bm3048MHZ: select 48MHz clock
  ACC      = 0;
  IOB      = ACC;
  OEB      = 0xFF;
  IOC      = 0x80; // PC7
  IOC     &= 0x7F;
  r7       = ACC;
  r6       = ACC;
  // counter: r6r7; 256 * 128
 label_213:
  ++r7;
  if (r7 == 0) {
    ++r6;
  }
 label_214:
  ACC  = r7;
  ACC ^= 0x80;
  ACC |= r6;
  if (ACC != 0)
    goto label_213;
  IOC |= 0x80; // PC7
 label_215: // wait for PC2==1
  if ((IOC & 0x04) == 0) // PC2
    goto label_215;
}

void function_001() {}
void function_005() {}
void function_010() {}
void function_015() {}
void function_021() {
  r7 = r5;
  r6 = r4;
  var0x42 = r3; var0x41 = r2; var0x40 = r1;
  r1 += 0x01;
  ACC = 0;
  __asm__("addc A, R2");
  r2 = ACC;
  var0x4A = var0x40;
  var0x4B = var0x41;
  function_003();
  ACC   = 0;
  ACC <<= 1;
  r3 = var0x42; r2 = var0x43; r1 = var0x44;
  /* goto label_179; */
  switch (r3) {
  case 0x01:
    DPL = r1; DPH = r2;
    __asm__("movx @DPTR, A");
    break;
  case 0x00:
    __asm__("mov @R1, A");
    break;
  case 0xFE:
    __asm__("movx @R1, A");
    break;
  default:
    return;
  }
}
void function_030() {
  IOE = 0x3B;
}
void function_031() {}

void function_014() {
  if (bit0x04) {
    USBCS |= 0x0A; // bmDISCON | bmRENUM    
  } else {
    USBCS |= 0x08; // bmDISCON
  }
 label_206:
  r7 = 0xDC;
  r6 = 0x05;
  function_004();
  USBIRQ = 0xFF;
  EPIRQ  = 0xFF;
  EXIF  &= 0xEF;
  USBCS &= 0xF7;
}


// delay for r6r7 ms
void function_004() {
  // var0x31,0x32 = r6r7
  var0x31 = r6;
  var0x32 = r7;
  ACC  = CPUCS;
  ACC &= 0x18; // select bmCLKSPD
  if (ACC == 0) {
    // 12MHz  (var0x31,32) = (var0x31,32 + 1) / 2
    r7 = var0x32 + 1;
    ACC = 0;
    __asm__("addc A, 0x31"); // ACC += var0x31 + CY
    CY = 0;
    ACC >> 1;
    var0x31 = ACC;

    ACC = r7;
    ACC >> 1;
    var0x32 = ACC;
    goto label_160; 
  }
 label_159:
  CPUCS &= 0x18; // select bmCLKSPD
  r7 = ACC;
  if (r7 != 0x10) // != 48 MHz --> 24 MHz
    goto label_160;

  // 48 MHz: (var0x31,32) *= 2
  ACC = var0x32;
  ACC += ACC;
  var0x32 = ACC;

  ACC = var0x31;
  ACC << 1;
  var0x31 = ACC;

  // loop (var0x31,32) down until 0x0000
 label_160:
  ACC = var0x32;
  --var0x32;
  r6 = var0x31;
  if (ACC == 0)
    --var0x31;
  ACC |= r6;
  if (ACC == 0)
    return;
  function_020();
  goto label_160;
}

// 1.0078 ms per function execution for 24MHz clock; 4 clock cycles/instr. cycle
// instr. cycles: 4 (lcall) + 9 + 603*10 + 4 (ret)
void function_020() {
  __asm__("mov A, #0x00");      // 2 instr. cycles
  __asm__("mov 0x86, A");       // 2 instr. cycles DPS = ACC
  __asm__("mov DPTR, #0xFDA5"); // 3 instr. cycles
  __asm__("mov R4, #0x05");     // 2 instr. cycles
  // loop 603 times (0x10000 - 0xFDA5) until DPTR == 0x0000 (overflow)
  __asm__("label_220:\n			                \
     inc DPTR;                  // 3 instr. cycles \n	\
     mov A, DPL;                // 2 instr. cycles \n	\
     orl A, DPH;                // 2 instr. cycles \n	\
     jnz label_220;             // 3 instr. cycles ");
  return;                       // 4 instr. cycles
}

void function_002() {
  CPUCS        &= 0xE7; // CLKSPD(1:0) = 0
  CPUCS        |= 0x10; // bmCLKSPD1 = 48MHz clock
  CKCON         = 0;
  REVCTL        = 0x03; // bmNOAUTOARM | bmSKIPCOMMIT
  FIFORESET     = 0x80;  SYNCDELAY;
  FIFORESET     = 0x20;  SYNCDELAY;
  FIFORESET     = 0x40;  SYNCDELAY;
  FIFORESET     = 0x60;  SYNCDELAY;
  FIFORESET     = 0x80;  SYNCDELAY;
  FIFORESET     = 0x00;  SYNCDELAY;
  IFCONFIG      = 0x40; // bm3048MHZ -> 48 MHz IF clock (default)
  EP2FIFOCFG    = 0x01;  SYNCDELAY; // bmWORDWIDE: PORTD = FIFO_Data(15:8)
  EP1OUTCFG     = 0xA0;  SYNCDELAY; // valid | bulk
  EP1INCFG      = 0xA0;  SYNCDELAY; // valid | bulk
  EP2CFG        = 0xE0;  SYNCDELAY; // bmVALID | bmDIR = valid | in | bulk | 512
  EP4CFG        = 0x20;  SYNCDELAY; // invalid | double buffer
  EP6CFG        = 0x20;  SYNCDELAY; // invalid | double buffer
  EP8CFG        = 0x20;  SYNCDELAY; // invalid | double buffer
  PINFLAGSAB    = 0x8C;  SYNCDELAY; // EP2 FLAGB,A = FF, EF
  PINFLAGSCD    = 0xAE;  SYNCDELAY; // EP6 FLAGD,C = EF, FF
  FIFOPINPOLAR  = 0x00;  SYNCDELAY;
  EP2AUTOINLENH = FIFOPINPOLAR+1;  SYNCDELAY;
  EP2AUTOINLENL = 0xFE;  SYNCDELAY; // EP2AUTOINLEN = 0x1FE = 510
  EP1OUTBC      = 0x01;  SYNCDELAY;
  /* // port C */
  PORTCCFG      = 0x00;
  OEC           = 0x8B;
  IOC           = 0x80;
  IOC          &= 0x7F;
  /* // port A */
  PORTACFG      = bmFLAGD; //0x80;
  OEA           = 0x80;
  /* // port B,D | when IFCFG1==1: alternate */
  IOB           = 0;
  OEB           = 0xFF;
  /* // port E */
  PORTECFG      = 0x00;  
  OEE           = 0xFF;
  IOE           = 0x38;

  if (EP24FIFOFLGS & 0x02)
    goto label_102;
  
  INPKTEND = 0x82;  SYNCDELAY;  // skip | ep2
 label_102:
  IOC    |= 0x80; // PC7 = 1
  r3      = 0x00;
  r2      = 0x00;
  r1      = 0x31;
  var0x4A = 0;
  var0x4B = 0x02;
  ACC = 0;
  r7 = ACC;
  r6 = ACC;
  function_003();
}

void function_003() {
  r3 = var0x47;
  r2 = var0x48;
  r1 = var0x49;
  function_012();
  if (CY) {
    I2CS |= bmSTART; //0x80;
    I2DAT = 0xA3;
    bit0x05 = 1;
    label_221();
    if (CY) {
      ACC = I2DAT;
      r3  = var0x47;
      r2  = var0x48;
      r1  = var0x49;
      label_179();
      bit0x05 = 0;
      label_221();
      if (CY) {
      label_105:
        ACC = 0;
        r5 = ACC;
        r4 = ACC;
      label_106:
        /* // R4R5 -= (var0x4A var0x4B) */
        __asm__("clr  C");
        __asm__("mov  A, R5");
        __asm__("subb A, 0x4B");
        __asm__("mov  A, R4");
        __asm__("subb A, 0x4A");
        if (!CY) goto label_111;
        /* // R6R7 = (var0x4A,var0x4B) + 0x4AFF */
        __asm__("mov  A, 0x4B");
        __asm__("add  A, #0xFE");
        __asm__("mov  R7, A");
        __asm__("mov  A, 0x4A");
        __asm__("addc A, #0xFF");
        __asm__("mov  R6, A");
        ACC = r7;
        if (ACC != r5) goto label_107;
        ACC = r6;
        if (ACC != r4) goto label_107;
        I2CS |= bmLASTRD; //0x20;
      label_107:
        ACC = I2DAT;
        r3 = var0x47;
        r2 = var0x48;
        r1 = var0x49;
        DPL = r5;
        DPH = r4;
        function_009();
        bit0x05 = 0;
        label_221();
        if (CY) goto label_109;
      }
    } 
  label_108:
    I2CS |= bmSTOP; //0x40;
    return;
  label_109:
    ++r5;
    if (r5==0)
      ++r4;
    goto label_106;
  label_111:
    I2CS |= bmSTOP; //0x40;
    ACC = I2DAT;
    r3 = var0x47;
    r2 = var0x48;
    r1 = var0x49;
    DPL = r5;
    DPH = r4;
    function_009();
    CY = 1;
  }
}

/* // write ACC to XRAM, RAM; absolute address */
/* // r3==0x01: ADDR16 = (r2 r1) (__xdata) */
/* // r4==0x00: ADDR8  = r1 (__idata)      */
/* // r3==0xfe: ADDR8  = r1 (__xdata)      */
void label_179() {
  switch (r3) {
  case 0x01:
    DPL = r1; DPH = r2;
    __asm__("movx @DPTR, A");
    break;
  case 0x00:
    __asm__("mov @R1, A");
    break;
  case 0xFE:
    __asm__("movx @R1, A");
    break;
  default:
    return;
  }
}

/* // write ACC to XRAM,RAM; relative address */
void function_009() {
  r0 = ACC;
  switch (r3) {
  case 0x01:
    __asm__("mov A, DPL");
    __asm__("add A, R1 ");
    __asm__("mov DPL, A");
    __asm__("mov A, DPH");
    __asm__("addc A, R2");
    __asm__("mov DPH, A");
    __asm__("mov A, R0 ");
    __asm__("movx @DPTR, A");
    break;
  case 0x00:
    ACC = r1;
    ACC += DPL;
    __asm__("mov A, R1 ");
    __asm__("add A, DPL");
    __asm__("xch A, R0 ");
    __asm__("mov @R0, A");
    break;
  case 0xFE:
    __asm__("mov  A, R1 ");
    __asm__("add  A, DPL");
    __asm__("xch  A, R0 ");
    __asm__("movx @R0, A");
    break;
  default:
    return;
  }
}

void function_006() {
  var0x3B = r6;
  var0x3C = r7;
  var0x3D = r3;
  var0x3E = r2;
  var0x3F = r1;
  IOC = 0x82;
  __asm__("clr A");
  var0x45 = ACC;
  var0x46 = ACC;
 label_163:
  __asm__("clr  C");
  __asm__("mov  A, 0x46"); // var0x45,0x46 -= var0x3B,0x3C
  __asm__("subb A, 0x3C");
  __asm__("mov  A, 0x45");
  __asm__("subb A, 0x3B");
  if (CY) {
    r3 = var0x3D;
    ++var0x3F;
    ACC = var0x3F;
    r2 = var0x3E;
    if (ACC)
      ++var0x3E;
    --ACC;
    r1= ACC;
    function_007();
    r7 = ACC;
    __asm__("clr A");
    r5 = ACC;
    r4 = ACC;
  label_165:
    ACC = r7;
    if (ACC & 0x01)
      var0x43 = 0x8A;
    else
      var0x43 = 0x82;
    IOC = 0x43;
    ACC = r7;
    CY = 0;
    ACC >>= 1;
    r7 = ACC;
    if (IOC & 0x04) {
      ACC = var0x44;
      CY = 0;
      ACC >>=1;
      ACC |= 0x80;
      var0x44 = ACC;
    } else {
      ACC = var0x44;
      CY = 0;
      ACC >>= 1;
      var0x44 = ACC;
    }
    ACC = var0x43;
    ACC |= 0x01;
    IOC  = ACC;
    ++r5;
    if (r5 == 0)
      ++r4;
    ACC = r5;
    ACC ^= 0x08;
    ACC |= r4;
    if (ACC != 0)
      goto label_165;
    r3 = var0x40;
    ++var0x42;
    ACC = var0x42;
    r2 = var0x41;
    if (ACC -= 0)
      ++var0x41;
    --ACC;
    r1 = ACC;
    ACC = var0x44;
    label_179();
    ++var0x46;
    if (ACC != 0)
      goto label_163;
    ++var0x45;
    goto label_163;
  }
  IOC = 0x80;
}

/* // read from memory into ACC; absolute address */
void function_007() {
  switch (r3) {
  case 0x01:
    __asm__("mov DPL, R1");
    __asm__("mov DPH, R2");
    __asm__("movx A, @DPTR");
    break;
  case 0x00:
    __asm__("mov A, @R1");
    break;
  case 0xFE:
    __asm__("movx A, @R1");
    break;
  default:
    __asm__("mov DPL, R1");
    __asm__("mov DPH, R2");
    __asm__("clr A");
    __asm__("movc A, @DPTR");
  }
}

/* // read from memory into ACC; relative address */
void function_008() {
  if (r3 == 0x01) {
    /* // DPTR += R2R1 */
    __asm__("mov  A, DPL");
    __asm__("add  A, R1");
    __asm__("mov  DPL, A");
    __asm__("mov  A, DPH");
    __asm__("addc A, R2");
    __asm__("mov  DPH, A");
    __asm__("movx A, @DPTR");
    return;
  }
  if (CY) { // r3 < 0x01
    r0 = r1+DPL;
    __asm__("mov A, @R0");
    return;
  }
  if (r3 == 0xFE) {
    r0 = r1+DPL;
    __asm__("movx A, @R0");
    return;
  }
  /* // r3 != 0,1,0xFE */
  // DPTR += R2R1
  __asm__("mov  A, DPL");
  __asm__("add  A, R1");
  __asm__("mov  DPL, A");
  __asm__("mov  A, DPH");
  __asm__("addc A, R2");
  __asm__("mov  DPH, A");
  __asm__("clr  A");
  __asm__("movc A, @A+DPTR");
  return;
}

void label_221() {
  do { // label_221:
    if (I2CS & bmDONE) { //bmBIT0
      CY = 1;
      return;
    }
    if (I2CS & bmBERR)  // bmBIT2
      goto label_222;
    
  } while (bit0x05 == 0 || (I2CS & bmACK)) ; // bmBIT1
 label_222:
  I2CS |= bmSTOP; //0x40
  CY = 0;
  return;    
}

/* // EP2 auto-in transfer */
void function_011() {
  r7 = ((IOC & 0x08) != 0);
  IFCONFIG = 0x43; // bm3048MHZ | bmIFCFGMASK
  // 256 x 256 wait
  ACC = 0;
  r5 = ACC;
  r4 = ACC;
 label_188:
  ++r5;
  if (r5 != 0x00)
    goto label_188;
  ++r4;
 label_189:
  ACC  = r5;
  ACC ^= r4;
  if (ACC != 0)
    goto label_188;
  FIFORESET     = 0x02; SYNCDELAY; // EP2
  EP2FIFOCFG    = 0x09; SYNCDELAY; // bmAUTOIN | bmWORDWIDE
  EP2CFG        = 0xE0; SYNCDELAY; // bmVALID | bmDIR = valid | in | bulk | 512
  PINFLAGSAB    = 0x8C; SYNCDELAY;
  PINFLAGSCD    = 0xAE; SYNCDELAY;
  FIFOPINPOLAR  = 0x00; SYNCDELAY;
  EP2AUTOINLENH = 0x01; SYNCDELAY;
  EP2AUTOINLENL = 0xFE; SYNCDELAY; // length = 0x1FE = 510
  if ((EP24FIFOFLGS & 0x02) == 0)
    INPKTEND = 0x82; SYNCDELAY;  // skip | ep2
}

void function_012() {
  r3 = r7;
  r2 = r6;
  ACC = 0;
  r5 = ACC;
  r4 = ACC;
 label_191:
  I2CS |= bmSTART; // 0x80
  I2DAT = 0xA2;
  bit0x05 = 1;
  label_221();

}

void handle_resume() __interrupt RESUME_ISR {
  EICON &= 0xEF;
}

/* // copied routines from setupdat.h */
#if 0
BOOL handle_get_descriptor() {
  return FALSE;
}


BOOL handle_vendorcommand(BYTE cmd) {
  return FALSE;
}

// this firmware only supports 0,0
BOOL handle_get_interface(BYTE ifc, BYTE* alt_ifc) { 
  return TRUE;
}
BOOL handle_set_interface(BYTE ifc, BYTE alt_ifc) { 
  return TRUE;
}

// get/set configuration
BYTE handle_get_configuration() {
  return 1; 
}
BOOL handle_set_configuration(BYTE cfg) { 
  return cfg==1 ? TRUE : FALSE; // we only handle cfg 1
}

// copied usb jt routines from usbjt.h
void sudav_isr() __interrupt SUDAV_ISR {
  /* CLEAR_SUDAV(); */
}

void sof_isr () __interrupt SOF_ISR __using 1 {
  /* CLEAR_SOF(); */
}

void usbreset_isr() __interrupt USBRESET_ISR {
  /* handle_hispeed(FALSE); */
  /* CLEAR_USBRESET(); */
}
void hispeed_isr() __interrupt HISPEED_ISR {
  /* handle_hispeed(TRUE); */
  /* CLEAR_HISPEED(); */
}
#endif
