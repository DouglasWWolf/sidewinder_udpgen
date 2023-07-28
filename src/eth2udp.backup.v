
//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 25-Jul-23  DWW  1000  Initial creation
//====================================================================================
/*

    This module formats an AXI stream as a UDP packet.
    
    The FIFO that feeds the AXIS_RX bus must be large enough to contain and entire packet!

*/
module eth2udp # 
(
    parameter[ 7:0] SRC_MAC = 1,    
    parameter[ 7:0] SRC_IP0 = 10,
    parameter[ 7:0] SRC_IP1 = 1,
    parameter[ 7:0] SRC_IP2 = 1,
    parameter[ 7:0] SRC_IP3 = 2,

    parameter[ 7:0] DST_IP0 = 10,
    parameter[ 7:0] DST_IP1 = 1,
    parameter[ 7:0] DST_IP2 = 1,
    parameter[ 7:0] DST_IP3 = 255,
    parameter[15:0] SRC_PORT = 1000,
    parameter[15:0] DST_PORT = 32000,
    
    parameter SWIDTH = 512
) 
(
    input clk, resetn,

    //=====================  The incoming ethernet packet ======================
    input [SWIDTH-1:0]      AXIS_RX_TDATA,
    input                   AXIS_RX_TVALID,
    input                   AXIS_RX_TLAST,
    output                  AXIS_RX_TREADY,
    //==========================================================================


    //=====================  The incoming packet length  =======================
    input [15:0]            AXIS_LEN_TDATA,
    input                   AXIS_LEN_TVALID,
    output                  AXIS_LEN_TREADY,
    //==========================================================================

    
    //========================  The outgoing UDP packet  =======================
    output reg [SWIDTH-1:0] AXIS_TX_TDATA,
    output [(SWIDTH/8)-1:0] AXIS_TX_TKEEP,
    output reg              AXIS_TX_TVALID,
    output                  AXIS_TX_TLAST,
    input                   AXIS_TX_TREADY
    //==========================================================================

);

// This is the width (in bytes) of the Ethernet/IPv4/UDP header
localparam HWIDTHB = 42; 

// This is the width (in bits) of the Ethernet/IPv4/UDP header
localparam HWIDTH = HWIDTHB * 8;

// How many bits remain in AXIS_TX_TDATA after the Eth/IPv4/UDP header is packed into it
localparam REMAINDER = SWIDTH - HWIDTH;

// This is the "AXIS_TX_TKEEP" field for the last data-cycle in the packet
localparam[HWIDTHB-1:0] final_tkeep = -1;

// This is the state of the primary state machine
reg[1:0] fsm_state;

// The statically declared ethernet header fields
localparam[47:0] eth_dst_mac    = {48'hFFFFFFFFFFFF};
localparam[47:0] eth_src_mac    = {40'hC400AD0000, SRC_MAC};
localparam[15:0] eth_frame_type = 16'h0800;

// The statically declared IPv4 header fields
localparam[15:0] ip4_ver_dsf    = 16'h4500;
localparam[15:0] ip4_id         = 16'hDEAD;
localparam[15:0] ip4_flags      = 16'h4000;
localparam[15:0] ip4_ttl_prot   = 16'h4011;
localparam[15:0] ip4_srcip_h    = {SRC_IP0, SRC_IP1};
localparam[15:0] ip4_srcip_l    = {SRC_IP2, SRC_IP3};
localparam[15:0] ip4_dstip_h    = {DST_IP0, DST_IP1};
localparam[15:0] ip4_dstip_l    = {DST_IP2, DST_IP3};

// The statically declared UDP header fields
localparam[15:0] udp_src_port   = SRC_PORT;
localparam[15:0] udp_dst_port   = DST_PORT;
localparam[15:0] udp_checksum   = 0;

// Compute both the IPv4 packet length and UDP packet length
wire[15:0]       ip4_length     = 28 + AXIS_LEN_TDATA;
wire[15:0]       udp_length     =  8 + AXIS_LEN_TDATA;

// Compute the checksum of the fixed fields in the IPv4 header
localparam[31:0] ip4_partial_cs = ip4_ver_dsf
                                + ip4_id
                                + ip4_flags
                                + ip4_ttl_prot
                                + ip4_srcip_h
                                + ip4_srcip_l
                                + ip4_dstip_h
                                + ip4_dstip_l;

// The 32-bit checksum of the 9 IPv4 header fields includes the length
wire[31:0] ip4_32_cs = ip4_partial_cs + ip4_length;

// Compute the 16-bit IPv4 checksum
wire[15:0] ip4_checksum = ~(ip4_32_cs[15:0] + ip4_32_cs[31:16]);

// This is the 42-byte packet header for a UDP packet
wire[HWIDTH-1:0] pkt_header =
{
    // Ethernet header fields
    eth_dst_mac,
    eth_src_mac,
    eth_frame_type,

    // IPv4 header fields
    ip4_ver_dsf,
    ip4_length,
    ip4_id,
    ip4_flags,
    ip4_ttl_prot,
    ip4_checksum,
    ip4_srcip_h,
    ip4_srcip_l,
    ip4_dstip_h,
    ip4_dstip_l,

    // UDP header fields
    udp_src_port,
    udp_dst_port,
    udp_length,
    udp_checksum
};


// The Ethernet IP sends the bytes from least-sigificant-byte to most-significant-byte.  
// This means we need to create a little-endian (i.e., reversed) version of our packet 
// header.
wire[HWIDTH-1:0] pkt_header_le;
genvar i;
for (i=0; i<HWIDTHB; i=i+1) begin
    assign pkt_header_le[i*8 +:8] = pkt_header[(HWIDTHB-1-i)*8 +:8];
end 


// This is 42 1-bits for writing to the TX_TKEEP line on the last data-cycle of a packet
localparam[HWIDTHB-1:0] pkt_tkeep = -1;

// This controls the state of AXIS_RX_TREADY in states 1 and 3
reg axis_rx_tready;

// Always contains the AXIS_RX signals from the previous cycle
reg axis_rx_tlast;
reg[SWIDTH-1:0] axis_rx_tdata;

//=====================================================================================================================
// Signals on the AXIS_TX bus depend upon what state the state-machine is in.
//=====================================================================================================================
assign AXIS_TX_TKEEP  = (fsm_state == 3) ? final_tkeep : -1;
assign AXIS_TX_TLAST  = (fsm_state == 3);
assign AXIS_RX_TREADY = (fsm_state == 2) ?  AXIS_TX_TREADY : axis_rx_tready;
//=====================================================================================================================


//=====================================================================================================================
// In state 1, AXIS_TX has valid data the moment a packet-length arrives on AXIS_TLEN.  State 1 is when we
// emit the Eth/IPv4/UDP header along with the first 22 bytes of packet-data
//=====================================================================================================================
always @* begin
    case (fsm_state) 
        0: AXIS_TX_TVALID = 0;
        1: AXIS_TX_TVALID = (AXIS_LEN_TREADY & AXIS_LEN_TVALID);
        2: AXIS_TX_TVALID = 1;
        3: AXIS_TX_TVALID = 1;
    endcase
end
//=====================================================================================================================


//=====================================================================================================================
// State 0: AXIS_TX_TDATA = 0, we're just coming out of reset
// State 1: AXIS_TX_TDATA = the packet header and the first 22 bytes of user data 
// State 2: AXIS_TX_TDATA = 42 bytes from previous data-cycle, and first 22 bytes of this data cycle
// State 3: AXIS_TX_TDATA = 42 bytes from previous data-cycle, and 22 bytes of zero-filled bits
//
// In state 1, we use axis_rx_tready to determine whether the first data-cycle of the packet is buffer in register
// axis_rx_tdata, or whether it's still sitting in AXIS_RX_TDATA
//---------------------------------------------------------------------------------------------------------------------
// Presuming a 64-byte SWIDTH and a 42-byte HWIDTH, this is 22 bytes of all 0 bits
localparam[REMAINDER-1:0] filler = 0;
//=====================================================================================================================
always @* begin
    case (fsm_state)
        0: AXIS_TX_TDATA = 0;
        1: AXIS_TX_TDATA = (axis_rx_tready == 0) ? {axis_rx_tdata[0 +: REMAINDER], pkt_header_le}
                                                 : {AXIS_RX_TDATA[0 +: REMAINDER], pkt_header_le};
        2: AXIS_TX_TDATA = {AXIS_RX_TDATA[0 +: REMAINDER], axis_rx_tdata[REMAINDER +: HWIDTH]};
        3: AXIS_TX_TDATA = {filler,                        axis_rx_tdata[REMAINDER +: HWIDTH]};
    endcase 
end
//=====================================================================================================================



//=====================================================================================================================
// This state machine has 4 states:
//
//   0 = We just came out of reset.  This state initializes things.
//
//   1 = Waiting for a "packet length" to arrive on AXIS_LEN.  When it does, the 42-byte packet header and the first
//       22 bytes of user-data are emitted on AXIS_TX.
//
//   2 = Waiting for incoming data-cycles on AXIS_LEN.  Every time we see a data-cycle, we emit 42 bytes from the
//       prior data-cycle and 22 bytes from the data-cycle that just arrived
//
//   3 = We emit the final 22 bytes of user-data
//
// The design of this state machine and accompanying logic is such that we assume that the incoming packet-length
// will be a multiple of 64.   If it's not, we will pad the outgoing packet to a multiple of 64 bytes, but a standard
// TCP/IP stack on the receiving end is smart enough to ignore the extra bytes.
//
// Since logic outside of this routine buffers up the entire packet prior to presenting us with a packet-length on
// the AXIS_LEN bus, we may assume that an incoming cycle of user data (on AXIS_RX) will be available on every
// consecutive cycle after receiving a packet-length.
//=====================================================================================================================

// We are able to receive data from AXIS_LEN in state 1, when the TX bus is ready for us to send
assign AXIS_LEN_TREADY = (resetn == 1 & fsm_state == 1 & AXIS_TX_TREADY);

always @(posedge clk) begin
    if (resetn == 0) begin
        axis_rx_tready <= 0;
        fsm_state      <= 0;
    
    end else case(fsm_state) 
        
        // Here we're coming out of reset
        0:  begin
                axis_rx_tready <= 1;
                fsm_state      <= 1;
            end


        // Here we're waiting for a packet-length to arrive on the AXIS_LEN bus.  While
        // we're waiting, we will capture the first data-cycle of the AXIS_RX bus.
        1:  begin

                // While we're waiting for data to arrive on AXIS_LEN, read the
                // first cyle of packet-data to arrive on AXIS_RX.  We can use the 
                // state of "axis_rx_tready" to determine whether the first cycle
                // of user data is sitting in axis_rx_tdata or in AXIS_RX_TDATA.
                //
                // The data-cycle we're waiting for on AXIS_RX could arrive before
                // the data on AXIS_LEN or on the same cyle as the data on AXIS_LEN.
                if (AXIS_RX_TREADY & AXIS_RX_TVALID) begin
                    axis_rx_tdata  <= AXIS_RX_TDATA;
                    axis_rx_tlast  <= AXIS_RX_TLAST;
                    axis_rx_tready <= 0;                    
                end

                // If a packet-length arrives, the packet header and the first
                // part of the user data are immediately emitted.  We know for a fact
                // that the entire packet will be queued up and waiting for us
                // on the AXIS_RX_BUS by the time we receive a packet-length on the
                // AXIS_LEN bus.
                if (AXIS_LEN_TREADY & AXIS_LEN_TVALID) begin
                     if (axis_rx_tready == 0)
                        fsm_state <= axis_rx_tlast ? 3 : 2;
                    else
                        fsm_state <= AXIS_RX_TLAST ? 3 : 2;
                end
            end


        // Every time a data-cycle clocks out on the AXIS_TX bus, save a copy of the RX
        // data that is waiting for us.  (The first 22 bytes of that RX data was just
        // emitted.)  When we encounter the last cycle of the RX packet, go to state
        // 3 in order to emit it.
        //
        // AXIS_RX_TREADY is driven by AXIS_TX_TREADY, so we know that data will 
        // arrive on the RX bus on every data-cycle where the TX bus is ready to 
        // retransmit it.
        2:  if (AXIS_TX_TREADY & AXIS_TX_TVALID) begin
                axis_rx_tdata <= AXIS_RX_TDATA;
                if (AXIS_RX_TLAST) begin
                    axis_rx_tready <= 0;
                    fsm_state      <= 3;
                end
            end

        // Here we're waiting for the last cycle of data to be emitted
        3:  if (AXIS_TX_TREADY & AXIS_TX_TVALID) begin
                axis_rx_tready <= 1;
                fsm_state      <= 1;                
            end
        
    endcase
end
//=====================================================================================================================


endmodule