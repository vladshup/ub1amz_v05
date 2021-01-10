// V1.0 19th November 2007
//
// Copyright 2006,2007 Phil Harman VK6APH
//
//  HPSDR - High Performance Software Defined Radio
//
//  Alex SPI interface.
//
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

//////////////////////////////////////////////////////////////
//
//		Alex Band Decoder & BPF2 selection
//
//////////////////////////////////////////////////////////////
/*
BPF design ranges:

BPF1: 1.5 MHz to 2.5 MHz for 160M
BPF2: 2 MHz to 6 MHz for 80/60M
BPF3: 5 MHz to 10 MHz for 40/30M
BPF4: 12 MHz to 24 MHz for 20/17M/15M
BPF5: 20 MHz to 35 MHz for 12/10M
LNA:  21 MHz to 54 MHz 

2020 	Mar changed by EU1SW for narrow BPF2 
*/

module BPF2_select(clock,frequency,BPF2);
input  wire        clock;
input  wire [31:0] frequency;
output reg   [7:0] BPF2;

always @(posedge clock)
begin
if 		(frequency <  1800000) 	BPF2 <= 8'b00000001;	// LPF_0_30
else if	(frequency <  2000000) 	BPF2 <= 8'b00000010;	// RX BPF 160M	
else if 	(frequency <  3500000)	BPF2 <= 8'b00000001;	// LPF_0_30
else if 	(frequency <  4000000)	BPF2 <= 8'b00000100;	// RX BPF 80M
else if 	(frequency <  7000000)	BPF2 <= 8'b00000001;	// LPF_0_30
else if 	(frequency <  7200000) 	BPF2 <= 8'b00001000; 	// RX BPF 40M
else if 	(frequency < 10000000) 	BPF2 <= 8'b00000001; 	// LPF_0_30
else if 	(frequency < 10150000) 	BPF2 <= 8'b00010000; 	// RX BPF 30M
else if 	(frequency < 14000000) 	BPF2 <= 8'b00000001; 	// LPF_0_30
else if 	(frequency < 14400000) 	BPF2 <= 8'b00100000; 	// RX BPF 20M
else if 	(frequency < 21000000) 	BPF2 <= 8'b00000001; 	// LPF_0_30
else if 	(frequency < 21500000) 	BPF2 <= 8'b01000000; 	// RX BPF 15M
else if 	(frequency < 28000000) 	BPF2 <= 8'b00000001; 	// LPF_0_30
else if 	(frequency < 30000000) 	BPF2 <= 8'b10000000; 	// RX BPF 10M
else					BPF2 <= 8'b00000001;	// LPF_0_30

end
endmodule
