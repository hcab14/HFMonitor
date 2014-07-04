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

#include "mem.h"

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
/* 32 bit number: var24,25,26,27,28 */
__data __at 0x25 unsigned char var0x25;
__data __at 0x26 unsigned char var0x26;
__data __at 0x27 unsigned char var0x27;
__data __at 0x28 unsigned char var0x28;

__data __at 0x29 unsigned char var0x29;
__data __at 0x2A unsigned char var0x2A;
__data __at 0x2B unsigned char var0x2B;
__data __at 0x2C unsigned char var0x2C;
/* 32 bit number: var2D,2E,2F,30 */
__data __at 0x2D unsigned char var0x2D;
__data __at 0x2E unsigned char var0x2E;
__data __at 0x2F unsigned char var0x2F;
__data __at 0x30 unsigned char var0x30;

/* 16 bit number: var31,32 */
// sleep for var0x31,32 ms
__data __at 0x31 unsigned char var0x31; // H
__data __at 0x32 unsigned char var0x32; // L

__data __at 0x33 unsigned char var0x33; //
__data __at 0x34 unsigned char var0x34; //

/* pointer */
__data __at 0x35 unsigned char var0x35; // FLAG
__data __at 0x36 unsigned char var0x36; // ADDRH
__data __at 0x37 unsigned char var0x37; // ADDRL

/* pointer */
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

void f005_process_host_request();
void f001_handle_setupdata();
void function_002();
void function_003();
void f004_delay();
void f005_return_true();
void function_006();
/* void f007_access_byte_abs(); */
/* void f008_access_byte_rel(); */
/* void f009_write_byte_rel(); */
void function_010();
void function_011();
void function_012();
void function_013();
void function_014();
void f015_resume();
void function_016();
void function_017();
void function_018();
void f019_goto_idle();
void function_020();
void function_021();
void function_022();
void function_023();
void f024_usb_get_configuration();
void f025_usb_get_interface();
void function_026();
void function_027();
void f028_usb_set_configuration();
void f029_usb_set_interface();
void function_030();
void f031_return_true();
void f032_return_true();
void f033_return_true();
void f034_return_true();
void f035_return_true();
void label_221();

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
  r2_1 = r6;    r3_1 = r7;   // dev_dscr
  r2_2 = 0x09;  r3_2 = 0x12; // dev_qual_dscr
  r0_1 = 0x09;  r1_1 = 0x1C; // highspd_dscr
  r0_2 = 0x09;  r1_2 = 0x43; // fullspd_dscr
  r4_2 = 0x09;  r5_2 = 0x6A; // dev_strings

  ACC  = r6;
  ACC &= 0xC0;
  if (ACC != 0) {
  label_093:
    var0x2D = 0x00;
    var0x2E = 0x80;
    var0x2F = r6;
    var0x30 = r7;
     /* 16 bit address operation; r6r7 = 0x09AA(dev_strings_end) - r6r7 + 2 */
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
    function_010(); // r2r3 == r6r7 && r0r2 == r4r5
    if (CY == 1) {
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
  } // ACC != 0

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
  
 label_097: // main loop
  f005_process_host_request();
  if (bit0x01) {
    f001_handle_setupdata();
    bit0x01 = 0;
  }
 label_098:
  if (bit0x03 == 0)
    goto label_097;
  f005_return_true();
  if (CY == 0)
    goto label_097;
  bit0x03 = 0;
 label_099:
  f019_goto_idle();
  if (bit0x00 == 0) {
    if (WAKEUPCS & 0x80) { // bmWU2
      if (WAKEUPCS & 0x02) // bmWU2EN
        goto label_099;
    }
    if (WAKEUPCS & 0x40)   // bmWU
      if (WAKEUPCS & 0x01) // bmWUEN
        goto label_099;
  }
 label_101:
  f015_resume();
  f031_return_true();
  goto label_097;
}

void f005_process_host_request() {
  if ((EP1OUTCS & 0x02) != 0) // bmEPBUSY
    return;
  // host data is available
  // Label_003:
  var0x35 = 0x01; var0x36 = 0xE7; var0x37 = 0x80; // 0xE780: EP1OUTBUF[0]
  r3 = var0x35; r2 = var0x36; r1 = var0x37;
  f007_access_byte_abs();
/*         cjne	A, #0x09, Label_004 // CY = (A < #0x09) */
/* Label_004: */
/*         jc	Label_005 */
/*         ljmp	Label_044 */
  if (ACC < 0x09) {
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
      break;
    case 0x01:
    label_015:
      function_017();
      break;
    case 0x02: // fpga_check
      /* label_016: */
      var0x38 = 0x01; var0x39 = 0xE7; var0x3A = 0xC0; //var0x39,0x3A=EP1INBUF
      // var0x35,36,37
      r3     = var0x35;
      ++var0x37; ACC = var0x37;
      r2     = var0x36;
      if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
      f007_access_byte_abs(); // gptr = [(r1r2),r3] -> ACC
      r7 = ACC;
      r3 = var0x38;
      ++var0x3A; ACC = var0x3A;
      r2 = var0x39;
      if (ACC == 0) ++var0x39; --ACC; r1 = ACC;
      ACC = r7;
      function_011();
      /* label_041: */
      r3 = var0x38; r2 = var0x39; r1 = var0x3A;
      ACC = r7;
      l179_write_byte_abs();
      EP1INBC = 0x02;
    /* label_042: */
      __asm__("movx @DPTR, A"); SYNCDELAY;
      break;
    case 0x03:
      /*   /\* label_019: *\/ */
        var0x38 = 0x01; var0x39 = 0xE7; var0x3A = 0xC0; // var0x39,0x3A = EP1INBUF
        ACC = EP1OUTBC;
        var0x31 = 0x00;
        var0x32 = ACC;
        r3 = var0x35;
        ++var0x37; ACC = var0x37;
        r2 = var0x36;
        if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
        f007_access_byte_abs();
        r7 = ACC;
        r3 = var0x38;
        ++var0x3A; ACC = var0x3A;
        r2 = var0x39;
        if (ACC == 0) ++var0x39; --ACC; r1 = ACC;
        ACC = r7;
        l179_write_byte_abs();
        ACC = var0x32;
        ACC += 0xFF;
        r7 = ACC;
        ACC = var0x31;
        __asm__("addc A, #0xFF");
        r6 = ACC;
        var0x40 = var0x38; var0x41 = var0x39; var0x42 = var0x3A;
        r3 = var0x35; r2 = var0x36; r1 = var0x37;
        function_006();
	/*   goto label_027; */
	/* label_027: */	
	/*   goto label_034; */
	/* label_034: */
	EP1INBC = EP1OUTBC; SYNCDELAY;
      break;
    case 0x04:
    label_028:
      r3 = var0x35; r2 = var0x36; r1 = var0x37;
      __asm__("mov DPTR, #0x0001");
      f008_access_byte_rel();
      IOE = ACC;
      break;
    case 0x05:
    label_022:
      var0x38 = 0x01; var0x39 = 0xE7; var0x3A = 0xC0;  // 0xE7C0 EP1INBUF[0]
      ACC = EP1OUTBC;
      var0x31 = 0x00;
      var0x32 = ACC;
      __asm__("clr A"); r7 = ACC; r6 = ACC;
    label_023:
      CY = 0;
      // r6r7 -= var0x31,32
      __asm__("mov  A, R7");
      __asm__("subb A, 0x32");
      __asm__("mov  A, R6");
      __asm__("subb A, 0x31");
      if (CY) {
	r3 = var0x35;
	++var0x37; ACC = var0x37;
	r2 = var0x36;
	if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
	f007_access_byte_abs();
	r5  = ACC;
	r3  = var0x38;
	++var0x3A; ACC = var0x3A;
	r2  = var0x39;
	if (ACC == 0) ++var0x39; --ACC; r1 = ACC;
	r5 = ACC;
	l179_write_byte_abs();
	++r7;
	if (r7 == 0)
	  ++r6;
      label_026:
	goto label_023;
      }
    label_027:      
      EP1INBC = EP1OUTBC; SYNCDELAY;
      break;
    case 0x06:
    label_029:
      var0x38 = 0x01; var0x39 = 0xE7; var0x3A = 0xC0; // 0xE7C0 EP1INBUF[0]
      r3 = var0x35;
      ++var0x37; ACC = var0x37;
      r2 = var0x36;
      if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
      f007_access_byte_abs(); // [r2r3] -> ACC
      r7 = ACC;
      r3 = var0x38;
      ++var0x3A; ACC = var0x3A;
      r2 = var0x39;
      if (ACC == 0) ++var0x39; --ACC; r1 = ACC;
      ACC = r7;
      l179_write_byte_abs();
      r3 = var0x35;
      ++var0x37; ACC = var0x37;
      r2 = 0x36;
      if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
      f007_access_byte_abs(); // [r2r3] -> ACC
      var0x33 = 0x00;
      var0x34 = ACC;
      ++var0x37; ACC = var0x37;
      r2 = var0x36;
      if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
      f007_access_byte_abs();
      r6 = ACC;
      ACC = r6;
      var0x33 |= ACC;
      r2 = var0x36; r1 = var0x37;
      f007_access_byte_abs(); // [r2r3] -> ACC
      var0x31 = 0x00;
      var0x32 = ACC;
      r3 = var0x38; r2 = var0x39; r1 = var0x3A;
      var0x40 = var0x31;
      var0x41 = ACC;
      r5 = var0x34; r4 = var0x33;
      function_021();
      ACC = var0x32;
      ACC += 0x02;
      /* goto label_034; */
      EP1INBC = ACC; SYNCDELAY;
      break;
    case 0x07:
    label_035:
      var0x38 = 0x01; var0x39 = 0xE7; var0x3A = 0xC0;  // 0xE7C0 EP1INBUF[0]
      r3 = var0x35;
      ++var0x37; ACC= var0x37;
      r2 = var0x36;
      if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
      f007_access_byte_abs();
      r7 = ACC;
      r3 = var0x38;
      ++var0x3A; ACC = var0x3A;
      r2 = var0x39;
      if (ACC == 0) ++var0x39; --ACC; r1 = ACC;
      ACC = r7;
      l179_write_byte_abs();
      r3 = var0x35;
      ++var0x37; ACC = var0x37;
      r2 = var0x36;
      if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
      f007_access_byte_abs();
      var0x33 = 0x00;
      var0x34 = ACC;
      ++var0x37; ACC = var0x37;
      r2 = var0x36;
      if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
      f007_access_byte_abs();
      r6 = ACC;
      ACC = r6;
      var0x33 |= ACC;
      ++var0x37; ACC = var0x37;
      r2 = var0x36;
      if (ACC == 0) ++var0x36; --ACC; r1 = ACC;
      f007_access_byte_abs();
      var0x31 = 0x00;
      var0x32 = ACC;
      r2 = var0x36; r1 = var0x37;
      var0x40 = var0x31;
      var0x41 = ACC;
      r5 = var0x34; r4 = var0x33;
      function_026();
      r3 = var0x38; r2 = var0x39; r1 = var0x3A;
      ACC = r7;
      l179_write_byte_abs();
      EP1INBC = 0x02; SYNCDELAY;
      break;
    case 0x08:
      function_017();
      function_030();    
      break;
    }
  } // acc >= 0x09
 /* label_044: */
  __asm__("clr A");
  EP1OUTBC = ACC;
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

void function_026() {
  r7 = r5;
  r6 = r4;
  var0x47 = var0x40;
  var0x48 = var0x41;
  function_013();
  __asm__("clr A");
  __asm__("rlc A");
  __asm__("mov R7, A");
}

void function_027() {  
  if (GPCR2 & 0x10) // bmFULLSPEEDONLY
    CY = 0;
  else
    CY = 1;
}

void function_013() {
  var0x44 = r3;
  var0x45 = r2;
  var0x46 = r1;
  bit0x04 = 1;
  function_012();
  if (CY) {
  label_198:
    ACC= var0x48; --var0x48;
    r6 = var0x47;
    if (ACC == 0) --var0x47;
    ACC |= r6;
    if (ACC != 0) {
      r3 = var0x44;
      ++var0x46; ACC = var0x46;
      r2 = var0x45;
      if (ACC == 0) ++var0x45; --ACC; r1 = ACC;
      f007_access_byte_abs();
      function_022();
      if (CY)
	goto label_198;
      I2CS |= bmSTART; // 0x40
    } else {
      I2CS |= bmSTART; // 0x40
      CY = 1;      
    }
  }
}

void function_016() {
  r1 = r7;
  r6 = r4_2;
  r7 = r5_2;
 label_210:
  DPL = r7; DPH = r6;
  __asm__("inc  DPTR");
  __asm__("movx A, @DPTR");
  __asm__("xrl  A, 0x03"); // ACC ^= 0x03
  if (ACC == 0) { // ACC == 3
    r5 = r1; --r1;
    ACC = r5;
    if (ACC == 0)
      return;
    DPL = r7; DPH = r6;
    __asm__("movx A, @DPTR");
    r4 = 0x00;
    __asm__("add  A, R7");
    __asm__("mov  R5, A");  // r5 = ACC + r7
    __asm__("mov  A, R4");  // ACC = r4 = 0;
    __asm__("addc A, R6");
    __asm__("mov  R6, A");  // r6 += CY
    r7 = r5; //    __asm__("mov  R7, R5"); // r7 = r5
    goto label_210;
  }
  __asm__("clr A");
  r6 = ACC; r7 = ACC;
}

void function_018() { // SUDAV Interrupt; depends on application
  switch (SETUPDAT[1]) { // bmRequest
  case 0xD1: // label_216
    r7  = FIFORESET & 0x7F; // !bmNAKALL
    SYNCDELAY;
    __asm__("mov  A, R7");
    __asm__("movx @DPTR, A");
    CY = 0;
    break;

  case 0xD0:
    r7 = FIFORESET | 0x80; // bmNAKALL
    SYNCDELAY;
    __asm__("mov  A, R7");
    __asm__("movx @DPTR, A");
    CY = 0;
    break;

  default: // label_218
    CY = 1;
  }
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

void f001_handle_setupdata() {
    /*  0     0   254   249   248   247   246   251 */
    /*  1     0   254   249   248   247   246   251 */
    /*  2     1   255   250   249   248   247   252 */
    /*  3     2     0   251   250   249   248   253 */
    /*  4     3     1   252   251   250   249   254 */
    /*  5     4     2   253   252   251   250   255 */
    /*  6     5     3   254   253   252   251     0 */
    /*  7     6     4   255   254   253   252     1 */
    /*  8     7     5     0     0     0     0     5 */
    /*  9     8     6     1     0     0     0     5 */
    /* 10     9     7     2     1     0     0     5 */
    /* 11    10     8     3     2     1     0     5 */

  ACC =SETUPDAT[1];
  switch (SETUPDAT[1]) { // bmRequest
  case  0: // Label_066 (GET_STATUS)
    f033_return_true();
    if (CY) {
      switch (SETUPDAT[0]) { // bmRequestType
      case 0x81: // label_068 -> label_173
	ACC = 0x00;
	EP0BUF[0] = ACC;
	__asm__("inc DPTR");
	__asm__("movx @DPTR, A");
	EP0BCH = 0x00;
	EP0BCL = 0x02;
	break;

      case 0x82: // label_069
	r7  = (SETUPDAT[4] & 0x7E);
	r6  = 0x00;
	ACC = SETUPDAT[4];
	__asm__("setb C"); // CY  = 1;
	__asm__("subb A, #0x80");
	r4 = 0x00;
	if (CY)
	  r5 = 0x00;
	else
	  r5 = 0x01;
	// label_071:
	r6  = (r4|r6);
	DPL = (r5|r7) + 0x43;
	__asm__("mov  A, #0x0E");
	__asm__("addc A, R6");
	DPH = ACC;
	__asm__("clr A");
	__asm__("movc A, @A+DPTR");
	r7 = ACC;
	__asm__("rlc  A");
	__asm__("subb A, ACC");
	r6  = ACC;
	r7 += 0xA1; // ACC = r7; ACC += 0xA1; r7 = ACC;
	ACC = r6;
	__asm__("addc A, #0xE6");
	DPL = r7; DPH = ACC;
	__asm__("movx A, @DPTR");
	ACC &= 0x01;
	EP0BUF[0] = ACC;
	__asm__("clr A");
	// label_073:
	__asm__("inc DPTR");
	__asm__("movx @DPTR, A");
	EP0BCH = ACC;
	EP0BCL = 0x02;	
	break;

      case 0x80:
	CY = 0;
	__asm__("clr A");
	__asm__("rlc A");
	__asm__("add A, ACC");
	r7 = ACC;
	CY = 0x02;
	__asm__("clr A");
	__asm__("rlc A");
	ACC |= r7;
	break;
      default:
	EP0CS |= 0x01; // bmEPSTALL
      }
      break;
    }
    break;

  case  1: // Label_075 (CLEAR_FEATURE)
    f034_return_true();
    if (CY) {
      ACC = SETUPDAT[0] + 0xFE;
      switch (ACC) {
      case 0x02: // label_079
	if (SETUPDAT[2] == 0x00) {
	  r6  = (r4|r6);
	  DPL = (r5|r7) + 0x43;
	  __asm__("mov  A, #0x0E");
	  __asm__("addc A, R6");
	  DPH = ACC;
	  __asm__("clr A");
	  __asm__("movc A, @A+DPTR");
	  r7 = ACC;
	  __asm__("rlc  A");
	  __asm__("subb A, ACC");
	  r6  = ACC;
	  r7 += 0xA1; // ACC = r7; ACC += 0xA1; r7 = ACC;
	  ACC = r6;
	  __asm__("addc A, #0xE6");
	  DPL = r7; DPH = ACC;
	  __asm__("movx A, @DPTR");
	  ACC &= 0xFE;
	  __asm__("movx @DPTR, A");
	  ACC = SETUPDAT[4];
	  ACC &= 0x80;
	  ACC >>= 1;
	  ACC >>= 1;
	  ACC >>= 1;
	  ACC &= 0x1F;
	  r7 = ACC;
	  __asm__("movx A, @DPTR");
	  ACC &= 0x0F;
	  ACC += r7;
	  TOGCTL  = ACC;
	  TOGCTL |= 0x20;
	} else {
	  EP0CS |= 0x01; // bmEPSTALL
	}
	break;

      case 0x00: // label_077
	if (SETUPDAT[2] == 0x01) {
	  bit0x00 = 0;
	} else {
	  EP0CS |= 0x01; // bmEPSTALL
	}
	break;

      default:
	// goto label_091
      }
    }
    break;

  case  3: // Label_083 (SET_FEATURE)
    f035_return_true();
    if (CY) {
      switch(SETUPDAT[0]) {
      case 2: // Label_085
	r7 = SETUPDAT[4] & 0x7E;
	r6 = 0x00;
	ACC = SETUPDAT[4];
	__asm__("setb C");
	__asm__("subb A, #0x80");
	r4 = 0x00;
	if (CY == 0) {
	  r5 = 0x01;
	} else {
	  r5 = 0x00;
	}
	r6 |= r4;             // ACC = r4; ACC |= r6; r6 = ACC;
	DPL = (r5|r7) + 0x43; // ACC = r5; ACC |= r7; ACC += 0x43; DPL = ACC;
	__asm__("clr A");
	__asm__("movc A, @A+DPTR");
	r7 = ACC;
	__asm__("rlc A");
	__asm__("subb A, ACC");
	r6 = ACC;
	r7 += 0xA1; // ACC = r7; ACC += 0xA1; R7 = ACC;
	ACC = r6;
	__asm__("addc A, #0xE6");
	DPL = r7; DPH = ACC;
	__asm__("movx A, @DPTR");
	__asm__("orl  A, #0x01");
	__asm__("movx @DPTR, A");
	break;

      case 0:
	if (SETUPDAT[2] == 0x01) {
	  bit0x00 = 1;
	} else {
	  if ((SETUPDAT[2] ^ 0x02) != 0)
	    EP0CS |= 0x01; // bmEPSTALL
	}
	break;

      default: // Label_088
	EP0CS |= 0x01; // bmEPSTALL
      }
    }
    break;
  case  8: // Label_065 (GET_CONFIGURATION)
    f024_usb_get_configuration();
    break;
  case  9: // Label_064 (SET_CONFIGURATION)
    f028_usb_set_configuration();
    break;
  case 10: // Label_062 (GET_INTERFACE)
    f025_usb_get_interface();
    break;
  case 11: // Label_063 (SET_INTERFACE)
    f029_usb_set_interface();
    break;
  case  6: // Label_053 (GET_DESCRIPTOR)
    f035_return_true();
    if (CY) {
      switch (SETUPDAT[3]) {
      case 0x02: // Label_056
	SUDPTRH = r2_2;
	SUDPTRL = r3_2;
	break;
      case 0x03: // Label_058
	r7 = SETUPDAT[2];
	function_016();
	r2 = r6; r1 = r7; r3 = 0x01;
	if ((r2 | r1 | r3) != 0) {
	  SUDPTRH = r6;
	  SUDPTRL = r7;
	} else {
	  EP0CS |= 0x01; // bmEPSTALL
	}
	break;
      case 0x06: // Label_055
	function_027();
	if (CY) {
	  SUDPTRH = r2_2;
	  SUDPTRL = r3_2;	  
	} else {
	  EP0CS |= 0x01; // bmEPSTALL
	}	
	break;
      case 0x07: // Label_057
	SUDPTRH = r6_1;
	SUDPTRL = r7_1;
	break;
      case 0x01:
	SUDPTRH = r2_1;
	SUDPTRL = r3_1;
	break;
      default:  // Label_061
	EP0CS |= 0x01; // bmEPSTALL
      }
    }
    break;

  default: // label_089    
    function_018();
    if (CY) {
      EP0CS |= 0x01; // bmEPSTALL
    }
  }
  label_091:
  EP0CS |= 0x80; // bmHSNAK
}
void f005_return_true() { __asm__("setb C"); }


void function_010() {
  // ACC = (r3-r7) | (r2-r6) | (r1-r5) | (r0-r4)
  __asm__("mov  A, R3");
  __asm__("subb A, R7");
  __asm__("mov  B, A ");
  __asm__("mov  A, R2");
  __asm__("subb A, R6");
  __asm__("orl  B, A ");
  __asm__("mov  A, R1");
  __asm__("subb A, R5");
  __asm__("orl  B, A ");
  __asm__("mov  A, R0");
  __asm__("subb A, R4");
  __asm__("orl  A, B ");
}
void f015_resume() {
  if (WAKEUPCS & 0x01) { // bmWUEN
    if (WAKEUPCS & 0x40) { // bmWU
      // WAKEUP event has occurred
      goto label_208;
    }
  }
 label_207:
  if ((WAKEUPCS & 0x02) == 0) // bmWU2EN
    return;
  if ((WAKEUPCS & 0x80) == 0) // bmWU2
    return;
 label_208:
  USBCS |= 0x01; // bmSIGRESUME
  r7 = 0x14; r6 = 0x00;
  f004_delay(); // 20ms
  USBCS &= 0xFE; // clear bmSIGRESUME
}

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
void function_022() {
  I2DAT   = ACC;
  bit0x05 = 1;
 label_221:
  if (I2CS & 0x01) { // bmDONE
    CY = 1; return;
  }
  if (I2CS & 0x04) { // bmBERR
  label_222:
    I2CS |= 0x40; // bmSTART
    CY = 0; return;
  }
  if (bit0x05 == 0) 
    goto label_221;
  I2CS |= 0x40; // bmSTART
  CY = 0;  
}

void function_030() {
  IOE = 0x3B;
}
void f031_return_true() { CY = 1; }
void f032_return_true() { CY = 1; }
void f033_return_true() { CY = 1; }
void f034_return_true() { CY = 1; }
void f035_return_true() { CY = 1; }

void function_014() {
  if (bit0x04) {
    USBCS |= 0x0A; // bmDISCON | bmRENUM    
  } else {
    USBCS |= 0x08; // bmDISCON
  }
 label_206:
  r7 = 0xDC;  r6 = 0x05;
  f004_delay(); // 1500 ms
  USBIRQ = 0xFF;  // 0x80 | bmEP0ACK | bmHSGRANT    bmBIT5 | bmURES | bmSUSP | bmSUTOK | bmSOF | bmSUDAV
  EPIRQ  = 0xFF;  // bmEP0IN | bmEP0OUT | bmEP1IN | bmEP1OUT | bmEP2 | bmEP4 | bmEP6 | bmEP8
  CLEAR_USBINT(); // EXIF  &= 0xEF;
  USBCS &= 0xF7;  // bmHSM | 0x70 | bmNOSYNSOF | bmRENUM | bmSIGRESUME
}


// delay for r6r7 ms
void f004_delay() {
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

void f019_goto_idle() {
  WAKEUPCS |= 0xC0;    // bmWU | bmWU2
  SUSPEND   = WAKEUPCS;
  PCON     |= 0x01;    // go to idle state
  SYNCDELAY;
  SYNCDELAY2;
}

// usb_get_interface
void f025_usb_get_interface() {
  EP0BUF[0] = r6_2;
  EP0BCH = 0x00;
  EP0BCL = 0x01;
}
// usb_set_interface
void f029_usb_set_interface() {
  r6_2 = SETUPDAT[2];
  CY = 1;
}

// usb_get_configuration
void f024_usb_get_configuration() {
  EP0BUF[0] = r7_2;
  EP0BCH = 0x00;
  EP0BCL = 0x01;
  CY = 1;
}

// usb_set_configuration
void f028_usb_set_configuration() {
  r7_2 = SETUPDAT[2];
  CY = 1;
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
      l179_write_byte_abs();
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
        f009_write_byte_rel();
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
    f009_write_byte_rel();
    CY = 1;
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
    f007_access_byte_abs();
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
    l179_write_byte_abs();
    ++var0x46;
    if (ACC != 0)
      goto label_163;
    ++var0x45;
    goto label_163;
  }
  IOC = 0x80;
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
  ACC = 0; r5 = ACC; r4 = ACC;
 label_191:
  I2CS |= bmSTART; // 0x80
  I2DAT = 0xA2;
  bit0x05 = 1;
  label_221();
  if (CY == 0) {
    ++r5;
    if (r5 ==0)
      ++r4;
    ACC = r5;
    ACC ^= 0x03;
    ACC |= r4;
    if (ACC != 0) goto label_191;
  }
  ACC = r5;
  ACC ^= 0x03;
  ACC |= r4;
  if (ACC == 0) {
    I2CS |= 0x40;
    CY = 0;
  } else {
    ACC = r2;
    function_022();
    if (CY) {
      ACC = r3;
      function_022();
      if (CY)
	CY = 1;
    } else {
      I2CS |= 0x40;
    }
  }  
}

void handle_resume() __interrupt RESUME_ISR {
  EICON &= 0xEF;
}

// 0x0800
void sudav_isr() __interrupt SUDAV_ISR { // Label_225  /* CLEAR_SUDAV(); */
  bit0x01 = 1;
  CLEAR_SUDAV();
  /* EXIF   &= 0xEF; */
  /* USBIRQ  = 0x01; */
}
void sof_isr() __interrupt SOF_ISR { // Label_228  /* CLEAR_SOF(); */
  CLEAR_SOF();
  /* EXIF   &= 0xEF; */
  /* USBIRQ  = 0x02; */
  
}
void sutok_isr() __interrupt SUTOK_ISR { // Label_227
  CLEAR_SUTOK();
  /* EXIF   &= 0xEF; */
  /* USBIRQ  = 0x04; */
}
void suspend_isr() __interrupt SUSPEND_ISR { // Label_226
  CLEAR_SUSPEND();
  /* EXIF   &= 0xEF; */
  /* USBIRQ  = 0x08; */
}
void usbreset_isr() __interrupt USBRESET_ISR { // Label_204  /* handle_hispeed(FALSE); */  /* CLEAR_USBRESET(); */
  r4_1 = r0_2;
  r5_1 = r1_2;
  DPL = r5_1; DPH = r4_1;
  __asm__("inc DPTR");
  ACC= 0x02;
  __asm__("inc DPTR");
  __asm__("movx @DPTR, A");
  r6_1 = r0_1;
  r7_1 = r1_1;
  DPL = r7_1; DPH = r6_1;
  __asm__("inc DPTR");
  ACC= 0x07;
  __asm__("movx @DPTR, A");  
  CLEAR_USBRESET();
  /* EXIF   &= 0xEF; */
  /* USBIRQ  = 0x10; */
}
void hispeed_isr() __interrupt HISPEED_ISR { // Label_202  /* handle_hispeed(TRUE); */  /* CLEAR_HISPEED(); */
  if (USBCS & 0x80) { // bmHSM
    r4_1 = r0_1;
    r5_1 = r1_1;
    DPL = r5_1; DPH = r4_1;
    __asm__("inc DPTR");
    ACC = 0x02;
    __asm__("movx @DPTR, A");
    r6_1 = r0_2;
    r7_1 = r1_2;
    DPL = r7_1; DPH = r6_1;
    __asm__("inc DPTR");
    ACC = 0x07;
    __asm__("movx @DPTR, A");    
  }  
  CLEAR_HISPEED();
  /* EXIF   &= 0xEF; */
  /* USBIRQ  = 0x20; */
}
void ep0ack_isr() __interrupt EP0ACK_ISR { // Label_231
}
/* void spare_isr() __interrupt RESERVED_ISR { // Label_232 */
/* } */
void ep0in_isr() __interrupt EP0IN_ISR { // Label_233
}
void ep0out_isr() __interrupt EP0OUT_ISR { // Label_234
}
void ep1in_isr() __interrupt EP1IN_ISR { // Label_235
}
void ep1out_isr() __interrupt EP1OUT_ISR { // Label_236
}
void ep2_isr() __interrupt EP2_ISR { // Label_237
}
void ep4_isr() __interrupt EP4_ISR { // Label_238
}
void ep6_isr() __interrupt EP6_ISR { // Label_239
}
void ep8_isr() __interrupt EP8_ISR { // Label_240
}
void ibn_isr() __interrupt IBN_ISR { // Label_241
}
/* void spare_isr() __interrupt RESERVED_ISR { // Label_232 */
/* } */
void ep0ping_isr() __interrupt EP0PING_ISR { // Label_242
}
void ep1ping_isr() __interrupt EP1PING_ISR { // Label_243
}
void ep2ping_isr() __interrupt EP2PING_ISR { // Label_245
}
void ep4ping_isr() __interrupt EP4PING_ISR { // Label_245
}
void ep6ping_isr() __interrupt EP6PING_ISR { // Label_246
}
void ep8ping_isr() __interrupt EP8PING_ISR { // Label_247
}
void errlimit_isr() __interrupt ERRLIMIT_ISR { // Label_248
}
/* void spare_isr() __interrupt RESERVED_ISR { // Label_232 */
/* } */
/* void spare_isr() __interrupt RESERVED_ISR { // Label_232 */
/* } */
/* void spare_isr() __interrupt RESERVED_ISR { // Label_232 */
/* } */
void ep2isoerr_isr() __interrupt EP2ISOERR_ISR { // Label_249
}
void ep4isoerr_isr() __interrupt EP4ISOERR_ISR { // Label_250
}
void ep6isoerr_isr() __interrupt EP6ISOERR_ISR { // Label_251
}
void ep8isoerr_isr() __interrupt EP8ISOERR_ISR { // Label_252
}
// 0x0880
// gpif ints
void ep2pf_isr() __interrupt EP2PF_ISR { // Label_253
}
void ep4pf_isr() __interrupt EP4PF_ISR { // Label_254
}
void ep6pf_isr() __interrupt EP6PF_ISR { // Label_255
}
void ep8pf_isr() __interrupt EP8PF_ISR { // Label_256
}
void ep2ef_isr() __interrupt EP2EF_ISR { // Label_257
}
void ep4ef_isr() __interrupt EP4EF_ISR { // Label_258
}
void ep6ef_isr() __interrupt EP6EF_ISR { // Label_259
}
void ep8ef_isr() __interrupt EP8EF_ISR { // Label_260
}
void ep2ff_isr() __interrupt EP2FF_ISR { // Label_262
}
void ep4ff_isr() __interrupt EP4FF_ISR { // Label_262
}
void ep6ff_isr() __interrupt EP6FF_ISR { // Label_263
}
void ep8ff_isr() __interrupt EP8FF_ISR { // Label_264
}
void gpifdone_isr() __interrupt GPIFDONE_ISR { // Label_265
}
void gpifwf_isr() __interrupt GPIFWF_ISR { // Label_266
}

