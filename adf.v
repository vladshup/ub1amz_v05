
module ADF (clk, band, clock, ADF_out, LE);

input clk;								// 48 kHz

input [2:0]band;						//band data 

output reg clock;							// clock to ADF
output reg LE;						// high when Register data available
output reg ADF_out;						// data to ADF


reg ADF;
reg [5:0] write;
reg [4:0] loop_count;
reg [2:0] previous_band;



// Set up reg data to send 
always @*	
begin
  case (select)
6'b000000: REG_data=32'h580005;
6'b000001: REG_data=32'hEF603C;
6'b000010: REG_data=32'h4B3;
6'b000011: REG_data=32'h10E42;
6'b000100: REG_data=32'h8008061;
6'b000101: REG_data=32'h240058;
6'b001000: REG_data=32'h580005;
6'b001001: REG_data=32'hEF603C;
6'b001010: REG_data=32'h4B3;
6'b001011: REG_data=32'h10E42;
6'b001100: REG_data=32'h8008031;
6'b001101: REG_data=32'h340008;
6'b010000: REG_data=32'h580005;
6'b010001: REG_data=32'hDF603C;
6'b010010: REG_data=32'h4B3;
6'b010011: REG_data=32'h10E42;
6'b010100: REG_data=32'h8008041;
6'b010101: REG_data=32'h3C8038;
6'b011000: REG_data=32'h580005;
6'b011001: REG_data=32'hBF603C;
6'b011010: REG_data=32'h4B3;
6'b011011: REG_data=32'h10E42;
6'b011100: REG_data=32'h80080C1;
6'b011101: REG_data=32'h348028;
6'b100000: REG_data=32'h580005;
6'b100001: REG_data=32'h9F603C;
6'b100010: REG_data=32'h4B3;
6'b100011: REG_data=32'h10E42;
6'b100100: REG_data=32'h8008301;
6'b100101: REG_data=32'h2901A8;
6'b101000: REG_data=32'h580005;
6'b101001: REG_data=32'h9F603C;
6'b101010: REG_data=32'h4B3;
6'b101011: REG_data=32'h10E42;
6'b101100: REG_data=32'h8008601;
6'b101101: REG_data=32'h268148;
6'b110000: REG_data=32'h0;
6'b110001: REG_data=32'h0;
6'b110010: REG_data=32'h0;
6'b110011: REG_data=32'h0;
6'b110100: REG_data=32'h0;
6'b110101: REG_data=32'h0;
6'b111000: REG_data=32'h0;
6'b111001: REG_data=32'h0;
6'b111010: REG_data=32'h0;
6'b111011: REG_data=32'h0;
6'b111100: REG_data=32'h0;
6'b111101: REG_data=32'h0;
  
  default: REG_data = 0;
  endcase
end

always @* select <= {band,load};

reg [5:0] select;
reg   [2:0] load;

reg  [31:0] REG_data;
reg  [31:0] latch_REG_data;
reg   [4:0] bit_cnt;



// Write Operation 
always @ (negedge clk)
begin
  
  case (write)
  4'd0:
  begin
    LE <= 1'b0;        					// set TLV320 CS high
    bit_cnt <= 5'd31;   					// set starting bit count to 15
    latch_REG_data <= REG_data;		// save the current settings
    write <= 4'd1;
    
  end

  4'd1:
  begin
    LE  <= 1'b0;                		// start data transfer with nCS low
    ADF_out <= latch_REG_data[bit_cnt];  			// set data up
    write  <= 4'd2;
  end

  4'd2:
  begin
    clock <= 1'b1;               			// clock data into TLV320
    write  <= 4'd3;
  end

  4'd3:
  begin
    clock <= 1'b0;               			// reset clock
    write  <= 4'd4;
  end

  4'd4:
  begin
    if (bit_cnt == 0) 						// word transfer is complete, check for any more
      write <= 4'd5;
    else
    begin
      bit_cnt <= bit_cnt - 1'b1;
      write <= 4'd1;    						// go round again
    end
    begin 
    previous_band <= band; 	   			// save the current boost setting 
    end 
  end

  4'd5:
  begin
    if (load == 5) begin					
		LE <= 1'b1;        					         
		  if (band != previous_band) begin  
			load <= 0;
			write <= 4'd0;
		  end
	      else write <= 4'd5;     			// hang out here forever
	end
    else begin									// else get next data             	
      LE <= 1'b1;
		write  <= 4'd0;           
      load <= load + 3'b1;  				// select next data word to send
    end
  end
  
  default: write <= 4'd0;
  endcase
end

endmodule

