//
//  Hermes Lite Core Wrapper for BeMicro CV
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
//
// (C) Steve Haynal KF7O 2014
//

module Hermes_Lite(

   input ptt_i,
	input	cwkey,
	input  KEY_DOT,                //dot input 
	input  KEY_DASH,               //dash input 
	output exp_ptt_n,
	input [11:0] ADC_in1,
	input [11:0] ADC_in2,
	input overflow,
	input overflow2,
	output [13:0] DAC_out,
	output DAC_ALC,
	output DAC_CLK,
	output audio_l,
	output audio_r,
	input adc_clk,
	input adc_rdy,
	input spare,
	input OSC_3072, // reference in
   output FPGA_PLL, //122.88MHz VCXO contol voltage
	//audio codec (TLV320AIC23B)
   output CBCLK,               	//3.072MHz
   output CLRCOUT,						//48 khz
   output CMCLK,                  //12.288MHz 
   inout i2c_sda,
   output i2c_scl,
   output CDIN,                   //RX data to TLV320
   input  CDOUT,                  //Mic data from TLV320
	output [6:0]userout_mux,
	output CTRL_TRSW,
	
	//12 bit adc's (ADC78H90CIMT/MCP320x)
   output ADCMOSI,                
   output ADCCLK,
   input  ADCMISO,
   output nADCCS, 
  
// RMII Ethernet PHY
	output [1:0] rmii_tx,
	output rmii_tx_en,
	input [1:0] rmii_rx,
   input rmii_osc,
	input rmii_crs_dv,
   inout PHY_MDIO,
   output PHY_MDC
	
   );

// PARAMETERS

// Ethernet Interface
parameter MAC = {8'h00,8'h1c,8'hc0,8'ha2,8'h22,8'h5d};
parameter IP = {8'd0,8'd0,8'd0,8'd0};

// Clock Frequency
parameter CLK_FREQ = 61440000;


// Number of Receivers
parameter NR = 2; // number of receivers to implement

wire IF_locked;
wire locked;
wire locked2;
wire CLRCLK;
wire IF_48;

wire pll_12288;
wire pll_3072;
wire pll_12800;
assign CLRCOUT = CLRCLK;
assign CLRCIN = CLRCLK;
assign CBCLK = pll_3072;
assign DAC_CLK = CLK_122;
assign CMCLK = pll_12288;

ifclocks PLL_IF_inst( .inclk0(adc_clk), .c0(pll_12288), .c1(CLRCLK), .c2(pll_3072), .c3(CLK_122), .locked(locked));

pll48mhz PLL_IF_inst2( .inclk0(adc_clk), .c0(IF_48), .c1(pll_12800), .locked(locked2));

assign IF_locked = (locked && locked2);

assign FPGA_PLL = OSC_3072 ^ pll_12800;

assign userout_mux[0]=SPI_SDO;
assign userout_mux[1]=SPI_SCK;
assign userout_mux[2]=SPI_STROBE;
assign userout_mux[3]=clock;
assign userout_mux[4]=LE;
assign userout_mux[5]=ADF_out;
assign userout_mux[6]=tune;


//assign userout_mux = userout;


// RMII2MII Conversion
wire [3:0] PHY_TX;
wire PHY_TX_EN;              //PHY Tx enable
reg PHY_TX_CLOCK;           //PHY Tx data clock
wire [3:0] PHY_RX;     
wire RX_DV;                  //PHY has data flag
reg PHY_RX_CLOCK;           //PHY Rx data clock
wire PHY_RESET_N;

RMII2MII_rev2 RMII2MII_inst(
	.clk(rmii_osc),
	.resetn(1'b1),
	.phy_RXD(rmii_rx),
	.phy_CRS(rmii_crs_dv),
	.mac_RXD(PHY_RX),
	.mac_RX_CLK(PHY_RX_CLOCK),
	.mac_RX_DV(RX_DV),
	.mac_TXD(PHY_TX),
	.mac_TX_EN(PHY_TX_EN),
	.phy_TXD(rmii_tx),
	.phy_TX_EN(rmii_tx_en),
	.mac_TX_CLK(PHY_TX_CLOCK),
	.mac_MDC_in(),
	.phy_MDC_out(),
	.mac_MDO_oen(),
	.mac_MDO_in(),
	.phy_MDIO(),
	.mac_MDI_out(),
	.phy_resetn()
);


// Hermes Lite Core
hermes_lite_core #(
	.MAC(MAC),
	.IP(IP),
	.CLK_FREQ(CLK_FREQ),
	.NR(NR)
	) 

	hermes_lite_core_inst(
   .adc_clk(adc_clk),
	.adc_rdy(adc_rdy),
	.IF_locked(IF_locked),
	.CLRCLK(CLRCLK),
	.audio_l (audio_l),
	.audio_r (audio_r),
	.exp_ptt_n(exp_ptt_n),
	.pll_3072(pll_3072),
	.pll_12288(pll_12288),
	.IF_48(IF_48),
	.dipsw(3'b100),
	.DAC_ALC(DAC_ALC),
	.ptt_i(ptt_i),
	.cwkey(cwkey),
	.KEY_DOT(KEY_DOT),                //dot input 
	.KEY_DASH(KEY_DASH),               //dash input 
	.DAC_out(DAC_out),
	.ADC_in1(ADC_in1),
	.ADC_in2(ADC_in2),
	.overflow(overflow),
	.overflow2(overflow2),
	.CLK_122(CLK_122),         
   .i2c_sda(i2c_sda),
	.i2c_scl(i2c_scl),                   
   .CDOUT(CDOUT),                  
   .CDIN(CDIN), 
	.clock(clock),							
	.LE(LE),						
	.ADF_out(ADF_out),
	.SPI_SDO (SPI_SDO),                //SPI data to Alex or Apollo 
	.SPI_SCK(SPI_SCK),                //SPI clock to Alex or Apollo 
	.SPI_STROBE(SPI_STROBE), 
	.userout(userout),
	.tune(tune),
	.ADCMOSI(ADCMOSI),                
   .ADCCLK(ADCCLK),
   .ADCMISO(ADCMISO),
   .nADCCS(nADCCS),
	.CTRL_TRSW(CTRL_TRSW),
	

    // MMI Ethernet PHY
  	.PHY_TX(PHY_TX),
  	.PHY_TX_EN(PHY_TX_EN),        
  	.PHY_TX_CLOCK(PHY_TX_CLOCK),
  	.PHY_RX(PHY_RX),     
  	.RX_DV(RX_DV),
  	.PHY_RX_CLOCK(PHY_RX_CLOCK),         
  	.PHY_RESET_N(PHY_RESET_N),
	.PHY_MDIO(PHY_MDIO),             
	.PHY_MDC(PHY_MDC)
);             

endmodule 
