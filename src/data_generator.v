//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 04-Oct-22  DWW  1000  Initial creation
//====================================================================================

module data_generator
(
    input clk, resetn,

    input[63:0] packet_count,
    input[7:0]  packet_length,
    input       start,

    //===============  AXI Stream interface for outputting data ================
    output [511:0] AXIS_TX_TDATA,
    output [63:0]  AXIS_TX_TKEEP,
    output reg     AXIS_TX_TVALID,
    output         AXIS_TX_TLAST,
    input          AXIS_TX_TREADY
    //==========================================================================
 );




reg[ 1:0] fsm_state;            // State of the state machine
reg[ 7:0] latched_pl;           // Packet length, latched 
reg[ 7:0] cycle_index;          // Counts continuously from 1 to 'latched_pl'
reg[16:0] packet_num;           // Current packet number being transmitted
reg[ 7:0] counter;              // Count of data-cycles transmitted
reg[63:0] packets_remaining;    // How many packets remain to be transmitted?
reg       restart;              // When this is 1, a new batch of packets gets sent

// This is high on any data-cycle that is the last data-cycle of the packet
wire eop = (cycle_index == latched_pl);

wire[7:0] tag[0:15];
assign tag[ 0] = 8'h00;
assign tag[ 1] = 8'h11;
assign tag[ 2] = 8'h22;
assign tag[ 3] = 8'h33;
assign tag[ 4] = 8'h44;
assign tag[ 5] = 8'h55;
assign tag[ 6] = 8'h66;
assign tag[ 7] = 8'h77;
assign tag[ 8] = 8'h88;
assign tag[ 9] = 8'h99;
assign tag[10] = 8'hAA;
assign tag[11] = 8'hBB;
assign tag[12] = 8'hCC;
assign tag[13] = 8'hDD;
assign tag[14] = 8'hEE;
assign tag[15] = 8'hFF;

//--------------------------------------------------------------------------------------------
// Fill in TX_DATA with predictable data
//--------------------------------------------------------------------------------------------

// In the debugger, this marks an easily visble data-cycle boundary
assign AXIS_TX_TDATA[0 +: 32] = 32'hFFFF_FFFF;  

// The other 15 32-bit words have predictable data in them
genvar i;
for (i=1; i<16; i=i+1) begin
    assign AXIS_TX_TDATA[i*32 +:32] = {tag[i], counter, packet_num[15:8], packet_num[7:0]};
end
//--------------------------------------------------------------------------------------------

// Fill in the outgoing data-packet fields
assign AXIS_TX_TKEEP = -1;
assign AXIS_TX_TLAST = eop;


always @(posedge clk) begin

    if (resetn == 0) begin
        counter        <= 0;
        packet_num     <= 0;
        cycle_index    <= 0;
        AXIS_TX_TVALID <= 0;
        fsm_state      <= 0;
        restart        <= 0;
    end else begin    

        // If the "start" line goes high, we're going to to restart packet
        // transmission as soon as the current packet is finished transmitting.
        if (start) restart <= 1;

        case(fsm_state)

        // If we're starting a new set of packet transmissions...
        0:  if (restart) begin
                restart           <= 0;
                packet_num        <= 0;
                counter           <= 0;
                cycle_index       <= 1;            
                packets_remaining <= packet_count;
                latched_pl        <= (packet_length == 0) ? 4 : packet_length;
                if (packet_count) begin
                    fsm_state      <= 1;
                    AXIS_TX_TVALID <= 1;
                end
            end 

        // If the most recent data-cycle was accepted by the receiver...
        1:  if (AXIS_TX_TVALID & AXIS_TX_TREADY) begin
         
                // If this was the last data-cycle of the packet...
                if (eop) begin
                    
                    // If this was the last packet (or we're restarting)...                    
                    if (restart || packets_remaining == 1) begin
                        AXIS_TX_TVALID <= 0;
                        fsm_state      <= 0;
                    end
                    
                    // There is one fewer packets remaining
                    packets_remaining  <= packets_remaining - 1;
                    
                    // Increment the packet number
                    packet_num  <= packet_num + 1;
                end
                
                // "cycle_index" continously counts from 1 to 'latched_pl'.
                if (cycle_index == latched_pl)
                    cycle_index <= 1;
                else
                    cycle_index <= cycle_index + 1;

                // Counter increases on each data-cycle we transfer
                counter <= counter + 1;
            end
         
        endcase

    end


end


endmodule






