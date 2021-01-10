// V2.0 13th September 2014
//
// Copyright 2014 Phil Harman VK6PH
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



//---------------------------------------------------
//		Alex SPI interface
//---------------------------------------------------




module SPI(
				input reset,
				input  spi_clock,
				input enable,
				input [31:0]Alex_data,
				output reg SPI_data,
				output reg SPI_clock,
				output reg Rx_load_strobe
			);

reg [3:0] spi_state;
reg [4:0] data_count;
reg [31:0] previous_Alex_data; 
reg loop_count;					


always @ (posedge spi_clock)
begin
case (spi_state)
0:	begin
		if (reset | ( enable & (Alex_data != previous_Alex_data))) begin
			data_count <= 31;				// set starting bit count to 15
			spi_state <= 1;
		end
	end		
1:	begin
	if (reset) 
		SPI_data <= 1'b0;
	else 
		SPI_data <= Alex_data[data_count];	// set up data to send
	spi_state <= 2;
	end
2:	begin
	SPI_clock <= 1'b1;					// set clock high
	spi_state <= 3;
	end
3:	begin
	SPI_clock <= 1'b0;					// set clock low
	spi_state <= 4;
	end
4:	begin
		if (data_count == 0)begin		// transfer complete
			Rx_load_strobe <= 1'b1; 	// strobe data
			spi_state <= 5;
		end
		else spi_state  <= 1;  			// go round again
	data_count <= data_count - 1'b1;
	end
5:	begin
	Rx_load_strobe <= 1'b0;				// reset strobe
		loop_count <= loop_count + 1'b1; // loop_count increments each time the SPI data word is sent after a word change is detected
		if (loop_count == 1'b1) begin			
				data_count <= 31;		// set starting bit count to 47
				spi_state <= 1;			// send data word twice
		end
		else begin
			previous_Alex_data <= Alex_data; // save current data
			spi_state <= 0;						// reset for next run
		end
		
	end
	
default: spi_state <= 0;	
	
endcase
end

endmodule
