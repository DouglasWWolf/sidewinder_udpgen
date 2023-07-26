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
reg[63:0] packet_num;           // Current packet number being transmitted
reg[63:0] counter;              // Count of data-cycles transmitted
reg[63:0] packets_remaining;    // How many packets remain to be transmitted?
reg       restart;              // When this is 1, a new batch of packets gets sent

// This is high on any data-cycle that is the last data-cycle of the packet
wire eop = (cycle_index == latched_pl);

// Fill in the packet data
assign AXIS_TX_TKEEP = -1;

assign AXIS_TX_TDATA = {64{counter[7:0]}};

//assign AXIS_TX_TDATA[0   +: 64] = counter;
//assign AXIS_TX_TDATA[64  +: 64] = packet_num;
//assign AXIS_TX_TDATA[384 +: 64] = ~packet_num;
//assign AXIS_TX_TDATA[448 +: 64] = ~counter;

assign AXIS_TX_TLAST            = eop;


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






