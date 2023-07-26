
//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 25-Jul-23  DWW  1000  Initial creation
//====================================================================================
/*

    This module formats an AXI stream as a UDP packet

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
    parameter[15:0] DST_PORT = 32000
) 
(
    input clk, resetn,

    //=====================  The incoming ethernet packet ======================
    input [511:0] AXIS_RX_TDATA,
    input [63:0]  AXIS_RX_TKEEP,
    input         AXIS_RX_TVALID,
    input         AXIS_RX_TLAST,
    output        AXIS_RX_TREADY,
    //==========================================================================


    //=====================  The incoming packet length  =======================
    input [15:0] AXIS_LEN_TDATA,
    input        AXIS_LEN_TVALID,
    output reg   AXIS_LEN_TREADY,
    //==========================================================================

    
    //========================  The outgoing UDP packet  =======================
    output [511:0] AXIS_TX_TDATA,
    output [63:0]  AXIS_TX_TKEEP,
    output         AXIS_TX_TVALID,
    output         AXIS_TX_TLAST,
    input          AXIS_TX_TREADY
    //==========================================================================

);

// This is the state of the primary state machine
reg fsm_state;

// The statically declared ethernet header fields
localparam[47:0] eth_dst_mac    = {48'hFFFFFFFFFFFF};
localparam[47:0] eth_src_mac    = {40'hC400AD0000, SRC_MAC};
localparam[15:0] eth_frame_type = 16'h0800;

// The statically declared IPv4 header fields
localparam[15:0] ip4_ver_dsf    = 16'h4500;
localparam[15:0] ip4_id         = 16'h0001;
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
wire[511:0] pkt_header =
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
wire[511:0] pkt_header_le;
genvar i;
for (i=0; i<42; i=i+1) begin
    assign pkt_header_le[i*8 +:8] = pkt_header[(41-i)*8 +:8];
end 


// This is 42 1-bits for writing to the TX_TKEEP line
localparam[41:0] pkt_tkeep = -1;

//=====================================================================================================================
// The output stream is either our pkt_header, or is driven directly by the input stream
//=====================================================================================================================
assign AXIS_TX_TVALID = (fsm_state == 0) ? (AXIS_LEN_TREADY & AXIS_LEN_TVALID) : AXIS_RX_TVALID;
assign AXIS_TX_TDATA  = (fsm_state == 0) ? pkt_header_le : AXIS_RX_TDATA;
assign AXIS_TX_TKEEP  = (fsm_state == 0) ? pkt_tkeep     : AXIS_RX_TKEEP;
assign AXIS_TX_TLAST  = (fsm_state == 0) ? 0             : AXIS_RX_TLAST;
assign AXIS_RX_TREADY = (fsm_state == 0) ? 0             : AXIS_TX_TREADY;
//=====================================================================================================================


//=====================================================================================================================
// This state machine decides whether to drive the AXIS_TX stream from the constructed packet-header or from
// the AXIS_RX stream.
//=====================================================================================================================
always @(posedge clk) begin
    if (resetn == 0) begin
        fsm_state       <= 0;
        AXIS_LEN_TREADY <= 0;
    
    end else case(fsm_state) 
        
        // In state 0, we wait for a packet-length to arrive on AXIS_TLEN,
        // then we go to state 1
        0:  begin
                AXIS_LEN_TREADY <= 1;
                if (AXIS_LEN_TREADY & AXIS_LEN_TVALID) begin
                     fsm_state       <= 1;
                     AXIS_LEN_TREADY <= 0;
                end
            end

        // In state 1, we wait for the last data-cycle of the packet to transmit,
        // then we go back to state 0
        1:  if (AXIS_TX_TREADY & AXIS_TX_TVALID & AXIS_TX_TLAST) begin
                AXIS_LEN_TREADY <= 1;
                fsm_state       <= 0;
            end

    endcase
end
//=====================================================================================================================


endmodule