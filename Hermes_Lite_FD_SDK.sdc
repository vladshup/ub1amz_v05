# Hermes.sdc
#
#
#
#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3


#**************************************************************************************
# Create Clock
#**************************************************************************************
# externally generated clocks (with respect to the FPGA)
#

create_clock -period 122.880MHz	  [get_ports adc_clk]					-name adc_clk
create_clock -period 61.440MHz	  [get_ports adc_rdy]					-name adc_rdy	
create_clock -period 50.000MHz    [get_ports rmii_osc] 				-name rmii_osc


derive_pll_clocks 
derive_clock_uncertainty


#*************************************************************************************
# Create Generated ClocK
#*************************************************************************************
# internally generated clocks
#
# PLL generated clocks feeding output pins 
#create_generated_clock -name CMCLK    [get_ports CMCLK]
#create_generated_clock -name CLRCIN  [get_ports CLRCIN]
#create_generated_clock -name CLRCOUT [get_ports CLRCOUT]
#create_generated_clock -name DAC_CLK [get_ports DAC_CLK]



#*************************************************************************************
# Set Clock Groups
#*************************************************************************************


set_clock_groups -asynchronous -group { rmii_osc \
					Tx_clock_2 \
					PHY_RX_CLOCK \
					PHY_RX_CLOCK_2 \
					} \
					-group {adc_clk CLK_122 \
							PLL_IF_inst|altpll_component|auto_generated|pll1|clk[0]} \
							PLL_IF_inst|altpll_component|auto_generated|pll1|clk[1]} \
							PLL_IF_inst|altpll_component|auto_generated|pll1|clk[2]} \
							PLL_IF_inst|altpll_component|auto_generated|pll1|clk[3]} \
					-group { OSC_3072 }


#set_clock_groups -asynchronous \
#               -group { PHY_CLK50 \
#                        network_inst|eth_pll_inst|altpll_component|auto_generated|pll1|clk[0] \
#                        network_inst|eth_pll_inst|altpll_component|auto_generated|pll1|clk[1] } \
#					-group { ADC1_CLK ADC2_CLK M_CLK \
#                        PLL_IF_inst|altpll_component|auto_generated|pll1|clk[3] \
#                        PLL_main|altpll_component|auto_generated|pll1|clk[3] \
#                        PLL_main|altpll_component|auto_generated|pll1|clk[0] \
#                        PLL_C_inst|altpll_component|auto_generated|pll1|clk[0] \
#                        PLL_C_inst|altpll_component|auto_generated|pll1|clk[1] \
#                        PLL_C_inst|altpll_component|auto_generated|pll1|clk[2] } \
#               -group { REF_CLK }					
					

## set input delays



# set output delays



## AD9866 RX Path



## AD9866 TX Path
## Adjust for PCB delays 






## Slow outputs
set_false_path -from * -to {leds[*] userout[*] exp_ptt_n}

## Slow inputs
set_false_path -from {extreset exp_present dipsw[*]} -to *
