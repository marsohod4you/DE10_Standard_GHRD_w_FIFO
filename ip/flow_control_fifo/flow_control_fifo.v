// (C) 2001-2013 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.



/*

This component is an Avalon-MM based FIFO with hardware flow control compatible
with the DMA-330 core in the hard processor system (HPS).  This component has a 
data port for writing data into the FIFO and reading data out of the FIFO.  The
CSR port is for setting the read and write watermarks as well as reading various
status bits.

The write watermark represents the amount of space (in words) that the component
needs to wait for being available in the FIFO before it performs a burst request
to the DMA.  This watermark must match the burst size programmed into the DMA
memory-to-device channel code.

The read watermark represents the amount of space (in words) that the component
needs to buffer in the FIFO before it performs a burst request to the DMA.
This watermark must match the burst size programmed into the DMA
device-to-memory channel code.

The data width configured for this component must match the burst size programmed
into the DMA channel code.  Failure to do so will cause a FIFO over/under flow.

The Synopsys protocol is used for the hardware flow control which is somewhat
different than the protocol used by the DMA-330 core (ARM protocol).  The protocol
is as follows:

1)  Single transfer line is asserted any time the TX FIFO channel is not full or
    the RX FIFO channel is not empty.
    
2)  Burst transfer line is asserted when the programmed watermark has not been
    crossed.  
    
3)  The single and burst transfer lines must remain asserted until the DMA issues
    the acknowledge back to the peripheral.  Even if the programmed watermark is
    crossed or the FIFO becomes full/empty, the peripheral must continue issuing
    the burst and single requests until acknowledged by the DMA.
    
4)  When a transfer is acknowledged the peripheral *must* deassert the burst and
    single request lines for at least one clock cycle.  


The following diagram shows the FIFO fill level and watermarks and describes when
each of the request lines will be asserted:
    
<------------------------------- FIFO DEPTH -------------------------------->

|---------------------------------------------------------------------------|
|***************************************                                    |
|---------------------------------------------------------------------------|
0        ^                             ^                           ^   FIFO_DEPTH
         |                             |                           |
     RX watermark                  Fill Level                TX Watermark

    - RX single is high because the FIFO is not empty
    - RX burst is high because the fill level is equal to or greater than the read watermark
    - TX single is high because the FIFO is not full
    - TX burst is high because the fill level is equal to or less than the write watermark
    


Author:  JCJB
Date:  11/14/2013
Revision:  1.0

Revision History:

1.0 - First version



Data Port (Width depends on the DATA_WIDTH parameter)

Offset        Access        Register
------        ------        --------
  0             W          Write Data
  1             R          Read Data

  
  
CSR Port (32-bit)

Offset        Access        Register
------        ------        --------
  0            R/W      TX Watermark
  1            R/W      RX Watermark
  2             R       [3..0] --> rx_burst, rx_single, tx_burst, tx_single
  3             R       [25..0] --> fifo_full, fifo_empty, fifo_used[23:0]
  4             R       Data width
  5             R       FIFO depth
  6           Wclr      FIFO clear (write 1 to clear the FIFO)
  7             R       Unused (zeros)


*/


// synthesis translate_off
`timescale 1ns / 1ps
// synthesis translate_on


module flow_control_fifo (
  clk,
  reset,
  
  // data port for pushing and popping the FIFO
  d_address,
  d_write,
  d_writedata,
  d_byteenable,
  d_read,
  d_readdata,

  // CSR port for controlling the read and write watermarks as well as reading back status bits
  csr_address,
  csr_write,
  csr_writedata,
  csr_byteenable,
  csr_read,
  csr_readdata,

  // transmit peripheral request interface
  tx_single,  // this will assert when the FIFO is not full
  tx_burst,   // this will assert when the FIFO fill level is below or reaches the programmed write watermark
  tx_ack,     // this will assert when the DMA is done with a transfer, the tx_burst line must be low when tx_ack is high
  
  rx_single,  // this will assert when the FIFO is not empty
  rx_burst,   // this will assert when the FIFO fill level is reaches or passes the programmed read watermark
  rx_ack      // this will assert when the DMA is done with a transfer, the rx_burst line must be low when the rx_ack is high
);

  parameter DATA_WIDTH = 32;    // width of the data port (FIFO width)
  parameter FIFO_DEPTH = 1024;  // valid values are 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8092 because I'm too lazy to write a log2 function for the next parameter
  localparam FIFO_DEPTH_LOG2 = (FIFO_DEPTH == 16)? 4 : (FIFO_DEPTH == 32)? 5 : (FIFO_DEPTH == 64)? 6 : (FIFO_DEPTH == 128)? 7 : (FIFO_DEPTH == 256)? 8 :
                               (FIFO_DEPTH == 512)? 9 : (FIFO_DEPTH == 1024)? 10 : (FIFO_DEPTH == 2048)? 11 : (FIFO_DEPTH == 4096)? 12 : 13;  // if this is expanded it can't go over 23

  input clk;
  input reset;
  
  input d_address;   // address 0 for writes, address 1 for reads
  input d_write;
  input [DATA_WIDTH-1:0] d_writedata;
  input [(DATA_WIDTH/8)-1:0] d_byteenable;  // going to ignore byte enables and always write/read full words
  input d_read;
  output wire [DATA_WIDTH-1:0] d_readdata;  // has a fixed 0 cycle latency

  input [2:0] csr_address;  // address 0 for writing the tx level watermark, 1 for writing the rx level water mark, 2 for reading the rx and tx single and burst lines, 3 for reading the FIFO fill level and full & empty flags, 4 for clearing the FIFO
  input csr_write;
  input [31:0] csr_writedata;
  input [3:0] csr_byteenable;
  input csr_read;
  output reg [31:0] csr_readdata;  // fixed read latency of 1
  
  output reg tx_single;
  output reg tx_burst;
  input tx_ack;
 
  output reg rx_single;
  output reg rx_burst;
  input rx_ack;
  
  reg [31:0] tx_water_mark;         // write to address 0 of the CSR space to write this
  reg [31:0] rx_water_mark;          // write to address 1 of the CSR space to write this
  wire fifo_full;
  wire fifo_empty;  
  wire [23:0] fifo_used;
  wire fifo_clear;                     // write 1 to address 6 of the CSR space to perform a synchronous clear of the FIFO
  wire fifo_write;
  wire fifo_read;
  reg [31:0] csr_readdata_mux;
  wire set_tx_single;
  wire reset_tx_single;
  wire set_tx_burst;
  wire reset_tx_burst;
  wire set_rx_single;
  wire reset_rx_single;
  wire set_rx_burst;
  wire reset_rx_burst;
  
  
  scfifo the_scfifo (
    .clock (clk),
    .aclr (reset),
    .sclr (fifo_clear),
    .usedw (fifo_used[FIFO_DEPTH_LOG2-1:0]),
    .wrreq (fifo_write),
    .data (d_writedata),
    .rdreq (fifo_read),
    .q (d_readdata),
    .empty (fifo_empty),
    .full (fifo_full)
  );
  defparam the_scfifo.add_ram_output_register = "ON";
  defparam the_scfifo.lpm_numwords = FIFO_DEPTH;
  defparam the_scfifo.lpm_showahead = "ON";
  defparam the_scfifo.lpm_width = DATA_WIDTH;
  defparam the_scfifo.lpm_widthu = FIFO_DEPTH_LOG2;
  defparam the_scfifo.overflow_checking = "OFF";    // letting the PRI interface take care of flow control
  defparam the_scfifo.underflow_checking = "OFF";   // letting the PRI interface take care of flow control


  
  always @ (posedge clk or posedge reset)
  begin
    if (reset)
    begin
      tx_water_mark <= FIFO_DEPTH;             // tx_burst and tx_single will have the same behavior by default
    end
    else if ((csr_write == 1) & (csr_address == 3'b000))
    begin
      if (csr_byteenable[0] == 1)
        tx_water_mark[7:0] <= csr_writedata[7:0]; 
       if (csr_byteenable[1] == 1)
        tx_water_mark[15:8] <= csr_writedata[15:8];
       if (csr_byteenable[2] == 1)
        tx_water_mark[23:16] <= csr_writedata[23:16];
       if (csr_byteenable[3] == 1)
        tx_water_mark[31:24] <= csr_writedata[31:24];   
    end
  end

  
  always @ (posedge clk or posedge reset)
  begin
    if (reset)
    begin
      rx_water_mark <= 1;                      // rx_burst and rx_single will have the same behavior by default
    end
    else if ((csr_write == 1) & (csr_address == 3'b001))
    begin
      if (csr_byteenable[0] == 1)
        rx_water_mark[7:0] <= csr_writedata[7:0]; 
       if (csr_byteenable[1] == 1)
        rx_water_mark[15:8] <= csr_writedata[15:8];
       if (csr_byteenable[2] == 1)
        rx_water_mark[23:16] <= csr_writedata[23:16];
       if (csr_byteenable[3] == 1)
        rx_water_mark[31:24] <= csr_writedata[31:24];   
    end
  end

  
  always @ (posedge clk or posedge reset)
  begin
    if (reset)
    begin
      csr_readdata <= 32'h00000000;
    end
    else if (csr_read == 1)
    begin
      csr_readdata <= csr_readdata_mux;
    end
  end
  
/*  
  The following four registers (tx_single, tx_burst, rx_single, rx_burst) must not deassert once asserted
  until the DMA sends the acknowledge back.  So for example tx_single can't deassert when the FIFO fills
  until the DMA sends the ack back.  This will cause a cycle when ack is asserted where the PRI does not
  issue a burst or single transfer which allows the logic in the HPS performing the adapatation of this 
  Synopsys handshake protocol over to the ARM protocol enough time to distinguish where one transfer completes
  and the next request begins.
*/
   
  always @ (posedge clk or posedge reset)
  begin
    if (reset)
    begin
      tx_single <= 0;
    end
    else
    begin  // reset must win over set
      if (reset_tx_single == 1)
      begin
        tx_single <= 0;
      end
      else if (set_tx_single == 1)
      begin
        tx_single <= 1;
      end
    end
  end  
  
  
  always @ (posedge clk or posedge reset)
  begin
    if (reset)
    begin
      tx_burst <= 0;
    end
    else
    begin  // reset must win over set
      if (reset_tx_burst == 1)
      begin
        tx_burst <= 0;
      end
      else if (set_tx_burst == 1)
      begin
        tx_burst <= 1;
      end
    end
  end


  always @ (posedge clk or posedge reset)
  begin
    if (reset)
    begin
      rx_single <= 0;
    end
    else
    begin  // reset must win over set
      if (reset_rx_single == 1)
      begin
        rx_single <= 0;
      end
      else if (set_rx_single == 1)
      begin
        rx_single <= 1;
      end
    end
  end 

  
  always @ (posedge clk or posedge reset)
  begin
    if (reset)
    begin
      rx_burst <= 0;
    end
    else
    begin  // reset must win over set
      if (reset_rx_burst == 1)
      begin
        rx_burst <= 0;
      end
      else if (set_rx_burst == 1)
      begin
        rx_burst <= 1;
      end
    end
  end  

  //this reg used in SignalTap to start capture either for write or for read FIFO
  reg transfer;
  always @ (posedge clk or posedge reset)
  begin
	if(reset) transfer = 1'b0;
	else
		transfer <= fifo_write | fifo_read;
  end

  always @ (csr_address, tx_water_mark, rx_water_mark, rx_burst, rx_single, tx_burst, tx_single, fifo_used, fifo_empty, fifo_full)
  begin
    case (csr_address)
      3'b000:  csr_readdata_mux = tx_water_mark;
      3'b001:  csr_readdata_mux = rx_water_mark;
      3'b010:  csr_readdata_mux = {28'h0000000, rx_burst, rx_single, tx_burst, tx_single};
      3'b011:  csr_readdata_mux = {6'b000000, fifo_full, fifo_empty, fifo_used[23:0]};
      3'b100:  csr_readdata_mux = DATA_WIDTH;
      3'b101:  csr_readdata_mux = FIFO_DEPTH;
      default: csr_readdata_mux = { 31'h00000000, transfer }; //avoid compiler remove transfer register
    endcase  
  end

  assign fifo_clear = (csr_write == 1) & (csr_address == 3'b110) & (csr_writedata[0] == 1);
  assign fifo_write = (d_write == 1) & (d_address == 0);
  assign fifo_read = (d_read == 1) & (d_address == 1);
  assign fifo_used[23:FIFO_DEPTH_LOG2] = { {(24-FIFO_DEPTH_LOG2-1){1'b0}}, fifo_full};  // when the fifo becomes full we need to make sure we use the full flag as well otherwise the used signal will roll over to all zeros
  
  assign set_tx_single = (fifo_full == 0) & (tx_single == 0);               // when the FIFO is not full and an acknowledge has forced tx_single low set tx_single
  assign reset_tx_single = (tx_ack == 1);                                   // reset the reset request when acknowledge comes back
  assign set_tx_burst = (fifo_used <= tx_water_mark) & (tx_burst == 0);     // when there isn't enough room for a burst into the FIFO deassert the burst signal
  assign reset_tx_burst = (tx_ack == 1);                                    // reset the burst request when acknowledge comes back
  
  assign set_rx_single = (fifo_empty == 0) & (rx_single == 0);              // when the FIFO is not full and an acknowledge has forced tx_single low set tx_single
  assign reset_rx_single = (rx_ack == 1);                                   // reset the single request when acknowledge comes back
  assign set_rx_burst = (fifo_used >= rx_water_mark) & (rx_burst == 0);     // when there isn't enough data stored in the FIFO for a burst deassert the burst signal
  assign reset_rx_burst = (rx_ack == 1);                                    // reset the burst request when acknowledge comes back
  
endmodule
  