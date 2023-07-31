
//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 25-Jul-23  DWW  1000  Initial creation
//====================================================================================
/*

    This module formats an AXI stream as a UDP packet.  It does this by buffering up
    an incoming packet (in a FIFO) while it counts the number of bytes in the
    packet.  Once the incoming packet has arrived, the packet-length is written into
    its own FIFO.

    The thread that reads those two FIFOs builds a valid Eth/IPv4/UDP packet header 
    then outputs (byte packed) the packet header followed by the packet data.
        
    The incoming AXI Stream should be byte packed; only the last beat (the beat with
    AXIS_PD_TLAST asserted) may have a TKEEP value with bits set to 0.
    
    Notable busses:

    AXIS_PL feeds the input of the packet-length FIFO
    AXIS_PD feeds the input of the packet-data FIFO

    AXIS_LEN comes from the output of the packet-length FIFO
    AXIS_RX  comes from the output of the packet-data FIFO

    AXIS_TX is the output stream containing a UDP packet

*/
module axis2udp # 
(
    // This is the width of the incoming and outgoing data bus
    parameter SWIDTH = 512,      

    // Last octet of the source MAC address
    parameter[ 7:0] SRC_MAC = 2,    
    
    // The source IP address
    parameter[ 7:0] SRC_IP0 = 10,
    parameter[ 7:0] SRC_IP1 = 1,
    parameter[ 7:0] SRC_IP2 = 1,
    parameter[ 7:0] SRC_IP3 = 2,

    // The destiniation IP address
    parameter[ 7:0] DST_IP0 = 10,
    parameter[ 7:0] DST_IP1 = 1,
    parameter[ 7:0] DST_IP2 = 1,
    parameter[ 7:0] DST_IP3 = 255,
    
    // The source and destination UDP ports
    parameter[15:0] SRC_PORT = 1000,
    parameter[15:0] DST_PORT = 32002,

    // This must be large enough to contain at least 1 of the largest packet.  Min is 16.
    parameter DATA_FIFO_SIZE = 64,

    // This must be at least as large as the number of the smallest packets that
    // can fit into the data FIFO.   Min is 16.
    parameter MAX_PACKET_COUNT = 32
) 
(
    input clk, resetn,

    //=========================  Incoming Packet Data  =========================
    input [SWIDTH-1:0]      AXIS_PD_TDATA,
    input [(SWIDTH/8)-1:0]  AXIS_PD_TKEEP,
    input                   AXIS_PD_TVALID,
    input                   AXIS_PD_TLAST,
    output                  AXIS_PD_TREADY,
    //==========================================================================


    
    //========================  The outgoing UDP packet  =======================
    output reg [SWIDTH-1:0] AXIS_TX_TDATA,
    output [(SWIDTH/8)-1:0] AXIS_TX_TKEEP,
    output reg              AXIS_TX_TVALID,
    output                  AXIS_TX_TLAST,
    input                   AXIS_TX_TREADY
    //==========================================================================

);

//=============  The packet-data incoming from the FIFO output  =============
wire [SWIDTH-1:0] AXIS_RX_TDATA;
wire              AXIS_RX_TVALID;
wire              AXIS_RX_TLAST;
wire              AXIS_RX_TREADY;
//==========================================================================

//============  The packet-length incoming from the FIFO output ============
wire [15:0]       AXIS_LEN_TDATA;
wire              AXIS_LEN_TVALID;
wire              AXIS_LEN_TREADY;
//==========================================================================


//=============  This feeds the input of the packet-length FIFO  ===========
wire [15:0]       AXIS_PL_TDATA;
wire              AXIS_PL_TVALID;
wire              AXIS_PL_TREADY;
//==========================================================================


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

// Compute the 32-bit version of the IPv4 header checksum
wire[31:0] ip4_cs32 = ip4_ver_dsf
                    + ip4_id
                    + ip4_flags
                    + ip4_ttl_prot
                    + ip4_srcip_h
                    + ip4_srcip_l
                    + ip4_dstip_h
                    + ip4_dstip_l
                    + ip4_length;

// Compute the 16-bit IPv4 checksum
wire[15:0] ip4_checksum = ~(ip4_cs32[15:0] + ip4_cs32[31:16]);

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

// We are able to receive data from AXIS_LEN in state 1 only when the TX bus is ready for us to send
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




//=====================================================================================================================
// This block counts the number of one bits in AXIS_PD_TKEEP, thereby determining the number of data-bytes in the
// AXIS_PD_TDATA field. 
//=====================================================================================================================
reg[7:0] data_byte_count;
//---------------------------------------------------------------------------------------------------------------------
integer n;
always @*
begin
    data_byte_count = 0;  
    for (n=0;n<64;n=n+1) begin   
        data_byte_count = data_byte_count + AXIS_PD_TKEEP[n];
    end
end
//=====================================================================================================================


//=====================================================================================================================
// This state machine writes entries to the packet-length FIFO
//=====================================================================================================================
reg[15:0] packet_size;
//---------------------------------------------------------------------------------------------------------------------

// AXIS_PL contains the measured length of the incoming data packet
assign AXIS_PL_TDATA = packet_size + data_byte_count;

// AXIS_PL has valid data on the cycle where we see TLAST on the incoming data packet
assign AXIS_PL_TVALID = (AXIS_PD_TREADY & AXIS_PD_TVALID & AXIS_PD_TLAST);

always @(posedge clk) begin
    if (resetn == 0) begin
        packet_size <= 0;
    end else begin
        
        // On every beat of incoming packet data, accumulate the packet-length.
        // When we see the last beat of the packet, write the packet-length to the FIFO
        if (AXIS_PD_TVALID & AXIS_PD_TREADY) begin
            if (AXIS_PD_TLAST == 0)
                packet_size <= packet_size + data_byte_count;
            else 
                packet_size <= 0;
        end
    end
end
//=====================================================================================================================


//====================================================================================
// This FIFO holds the incoming packet data
//====================================================================================
xpm_fifo_axis #
(
   .FIFO_DEPTH(DATA_FIFO_SIZE),    // DECIMAL
   .TDATA_WIDTH(SWIDTH),           // DECIMAL
   .FIFO_MEMORY_TYPE("auto"),      // String
   .PACKET_FIFO("false"),          // String
   .USE_ADV_FEATURES("0000")       // String
)
packet_data_fifo
(
    // Clock and reset
   .s_aclk   (clk   ),                       
   .m_aclk   (clk   ),             
   .s_aresetn(resetn),

    // The input bus to the FIFO
   .s_axis_tdata (AXIS_PD_TDATA ),
   .s_axis_tvalid(AXIS_PD_TVALID),
   .s_axis_tlast (AXIS_PD_TLAST ),
   .s_axis_tready(AXIS_PD_TREADY),

    // The output bus of the FIFO
   .m_axis_tdata (AXIS_RX_TDATA ),     
   .m_axis_tvalid(AXIS_RX_TVALID),       
   .m_axis_tlast (AXIS_RX_TLAST ),         
   .m_axis_tready(AXIS_RX_TREADY),     

    // Unused input stream signals
   .s_axis_tdest(),
   .s_axis_tid  (),
   .s_axis_tstrb(),
   .s_axis_tuser(),
   .s_axis_tkeep(),

    // Unused output stream signals
   .m_axis_tdest(),             
   .m_axis_tid  (),               
   .m_axis_tstrb(), 
   .m_axis_tuser(),         
   .m_axis_tkeep(),           

    // Other unused signals
   .almost_empty_axis(),
   .almost_full_axis(), 
   .dbiterr_axis(),          
   .prog_empty_axis(), 
   .prog_full_axis(), 
   .rd_data_count_axis(), 
   .sbiterr_axis(),
   .wr_data_count_axis(),
   .injectdbiterr_axis(),
   .injectsbiterr_axis()
);
//====================================================================================


//====================================================================================
// This FIFO holds the length of the incoming data packets
//====================================================================================
xpm_fifo_axis #
(
   .FIFO_DEPTH(MAX_PACKET_COUNT),  // DECIMAL
   .TDATA_WIDTH(16),               // DECIMAL
   .FIFO_MEMORY_TYPE("auto"),      // String
   .PACKET_FIFO("false"),          // String
   .USE_ADV_FEATURES("0000")       // String
)
packet_length_fifo
(
    // Clock and reset
   .s_aclk   (clk   ),                       
   .m_aclk   (clk   ),             
   .s_aresetn(resetn),

    // The input bus to the FIFO
   .s_axis_tdata (AXIS_PL_TDATA ),
   .s_axis_tvalid(AXIS_PL_TVALID),
   .s_axis_tready(AXIS_PL_TREADY),

    // The output bus of the FIFO
   .m_axis_tdata (AXIS_LEN_TDATA ),     
   .m_axis_tvalid(AXIS_LEN_TVALID),       
   .m_axis_tready(AXIS_LEN_TREADY),     

    // Unused input stream signals
   .s_axis_tdest(),
   .s_axis_tid  (),
   .s_axis_tstrb(),
   .s_axis_tuser(),
   .s_axis_tkeep(),
   .s_axis_tlast(),

    // Unused output stream signals
   .m_axis_tdest(),             
   .m_axis_tid  (),               
   .m_axis_tstrb(), 
   .m_axis_tuser(),         
   .m_axis_tkeep(),           
   .m_axis_tlast(),         

    // Other unused signals
   .almost_empty_axis(),
   .almost_full_axis(), 
   .dbiterr_axis(),          
   .prog_empty_axis(), 
   .prog_full_axis(), 
   .rd_data_count_axis(), 
   .sbiterr_axis(),
   .wr_data_count_axis(),
   .injectdbiterr_axis(),
   .injectsbiterr_axis()
);
//====================================================================================

endmodule