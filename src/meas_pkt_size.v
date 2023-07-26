//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 25-Jul-23  DWW  1000  Initial creation
//====================================================================================
/*

    This module counts the number of bytes in each packet of an AXI stream.

    On the last data-cycle of every packet, the number of bytes in the packet
    are written to the TX_LEN AXI Stream.
*/
module meas_pkt_size
(
    input clk, resetn,

    //========  The input side of the AXI data stream we're measuring  =========
    input [511:0] AXIS_RX_TDATA,
    input [63:0]  AXIS_RX_TKEEP,
    input         AXIS_RX_TVALID,
    input         AXIS_RX_TLAST,
    output        AXIS_RX_TREADY,
    //==========================================================================


    
    //========  The output side of the AXI data stream we're measuring  ========
    output [511:0] AXIS_TX_TDATA,
    output [63:0]  AXIS_TX_TKEEP,
    output         AXIS_TX_TVALID,
    output         AXIS_TX_TLAST,
    input          AXIS_TX_TREADY,
    //==========================================================================


    //=========  This is where we output the packet lengths we measure  ========
    output [15:0] AXIS_LEN_TDATA,
    output        AXIS_LEN_TVALID,
    input         AXIS_LEN_TREADY
    //==========================================================================

);

// Tie the output stream to the input stream
assign AXIS_TX_TDATA  = AXIS_RX_TDATA;
assign AXIS_TX_TKEEP  = AXIS_RX_TKEEP;
assign AXIS_TX_TVALID = AXIS_RX_TVALID;
assign AXIS_TX_TLAST  = AXIS_RX_TLAST;
assign AXIS_RX_TREADY = AXIS_TX_TREADY;


//==============================================================================
// This state machine counts the number of one bits in TKEEP, thereby 
// determining the number of data-bytes in the AXIS_RX_TDATA field. 
//==============================================================================
reg[7:0] data_byte_count;
integer i;
//------------------------------------------------------------------------------
always @(AXIS_RX_TKEEP)
begin
    data_byte_count = 0;  
    for(i=0;i<64;i=i+1)   
        if(AXIS_RX_TKEEP[i]) 
            data_byte_count = data_byte_count + 1;
end
//==============================================================================

reg[15:0] packet_size;

// AXIS_LEN contains the measured length of the incoming data packet
assign AXIS_LEN_TDATA = packet_size + data_byte_count;

// AXIS_LEN has valid data on the cycle where we see TLAST on the incoming data packet
assign AXIS_LEN_TVALID = (AXIS_RX_TREADY & AXIS_RX_TVALID & AXIS_RX_TLAST);

always @(posedge clk) begin
    if (resetn == 0) begin
        packet_size <= 0;
    end else begin
        
        // If this is a valid incoming data-cycle...
        if (AXIS_RX_TVALID & AXIS_RX_TREADY) begin
            if (AXIS_RX_TLAST == 0)
                packet_size <= packet_size + data_byte_count;
            else 
                packet_size <= 0;
        end
    end
end



endmodule
