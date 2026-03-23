`timescale 1ns/1ps

module LDPC_encode(
  input  wire 			  clk,
  input  wire 			  rst,          // synchroniczny reset do 0
  
  // Strumień wejściowy
  input  wire [15:0]   in_k,
  input  wire [15:0]	  in_filler_num,
  input  wire [8:0]	  in_zc,
  input  wire 			  in_bg,
  
  input  wire 			  in_valid,
  input  wire [63:0]   in_chunk,
  input  wire 			  in_last,      // =1 przy ostatnim chunku DANYCH KAŻDEGO CB
  input  wire 			  in_last_cb,   // =1 przy ostatnim chunku DANYCH OSTATNIEGO CB
  
  // Strumień wyjściowy
  
  output	reg			  out_valid,
  output reg  [63:0]   out_chunk,
  output reg			  out_last,
  output reg  [5:0]	  out_pad_left,		 //number of pad bits from MSB
  output	reg  [5:0]	  out_pad_right	    //number of pad bits from LSB
  
);

	// Params
   localparam [7:0]	 W 						= 64;                 		// szerokość słowa danych
	
	// ======== BG1 constants (wstaw swoje z INFO) ========
	localparam integer BG1_ROWS   = 46;
	localparam integer BG1_NNZ    = 316;    
	localparam integer BG1_NUMZC  = 51;    

	// ======== BG2 constants ========
	localparam integer BG2_ROWS   = 42;
	localparam integer BG2_NNZ    = 197;    
	localparam integer BG2_NUMZC  = 51;     
	
	//FSM 1, STORE CB
	localparam [1:0] 	 MEM_IDLE		 			= 2'd0;
	localparam [1:0] 	 MEM_COLLECTING			= 2'd1;
	
	reg 		  [1:0]	 state_fsm1;
	
	//FSM 2,  CALC PARITY
	localparam [4:0] 	 PARITY_IDLE		 					= 5'd0;
	localparam [4:0] 	 PARITY_FIND_ZC		 				= 5'd1;
	localparam [4:0] 	 PARITY_FIRST_WINDOW_PREP			= 5'd2;
	localparam [4:0] 	 PARITY_CB_BEGIN						= 5'd3;
	localparam [4:0] 	 PARITY_ROWPTR							= 5'd4;
	localparam [4:0] 	 PARITY_SHIFT		 					= 5'd5;
	localparam [4:0] 	 PARITY_READ_WORDS					= 5'd6;
	localparam [4:0] 	 PARITY_FILL_BUFF		 				= 5'd7;
	localparam [4:0] 	 PARITY_WINDOW_PREP					= 5'd8;
	localparam [4:0] 	 PARITY_ROTATE		 					= 5'd9;
	localparam [4:0] 	 PARITY_FILL_BUFF_WINDOW_MOVE		= 5'd10;
	localparam [4:0] 	 PARITY_XOR								= 5'd11;
	localparam [4:0] 	 PARITY_FIRST_PB						= 5'd12;
	localparam [4:0] 	 PARITY_EQUATION						= 5'd13;
	localparam [4:0] 	 PARITY_SAVE_PB						= 5'd14;
	localparam [4:0] 	 PARITY_EQUATION_PREP_ROT			= 5'd15;
	localparam [4:0] 	 PARITY_EQUATION_PREP_SYNDROMES	= 5'd16;
	
	
	
	reg 	     [4:0]	state_fsm2;
	
	//FSM 3, OUT
	localparam [2:0] 	 LETGO_IDLE		 			= 3'd0;
	localparam [2:0] 	 LETGO_OUT_CB_PREP		= 3'd1;
	localparam [2:0] 	 LETGO_OUT_CB				= 3'd2;
	localparam [2:0] 	 LETGO_OUT_PB_SYNDROMES	= 3'd3;
	localparam [2:0] 	 LETGO_OUT_PB		  		= 3'd4;
	
	reg 		  [2:0]	 state_fsm3;
	
	//ROMS
	// 16-bit ROM words (hex lines)
	reg [15:0] bg1_row_ptr [0:BG1_ROWS];                // ROWS+1
	reg [15:0] bg1_col_idx [0:BG1_NNZ-1];               // NNZ
	reg [15:0] bg1_shift   [0:BG1_NUMZC*BG1_NNZ-1];     // NUMZC*NNZ
	reg [15:0] bg1_zc_list [0:BG1_NUMZC-1];             // NUMZC
	
	reg [15:0] bg2_row_ptr [0:BG2_ROWS];
	reg [15:0] bg2_col_idx [0:BG2_NNZ-1];
	reg [15:0] bg2_shift   [0:BG2_NUMZC*BG2_NNZ-1];
	reg [15:0] bg2_zc_list [0:BG2_NUMZC-1];
	
	initial begin
	  $readmemh("D:/Docs/Uczelnia/Dyplom1/matlab/ldpc_roms_out/BG1_row_ptr.memh", bg1_row_ptr);
	  $readmemh("D:/Docs/Uczelnia/Dyplom1/matlab/ldpc_roms_out/BG1_col_idx.memh", bg1_col_idx);
	  $readmemh("D:/Docs/Uczelnia/Dyplom1/matlab/ldpc_roms_out/BG1_shift.memh",   bg1_shift);
	  $readmemh("D:/Docs/Uczelnia/Dyplom1/matlab/ldpc_roms_out/BG1_zc_list.memh", bg1_zc_list);
	
	  $readmemh("D:/Docs/Uczelnia/Dyplom1/matlab/ldpc_roms_out/BG2_row_ptr.memh", bg2_row_ptr);
	  $readmemh("D:/Docs/Uczelnia/Dyplom1/matlab/ldpc_roms_out/BG2_col_idx.memh", bg2_col_idx);
	  $readmemh("D:/Docs/Uczelnia/Dyplom1/matlab/ldpc_roms_out/BG2_shift.memh",   bg2_shift);
	  $readmemh("D:/Docs/Uczelnia/Dyplom1/matlab/ldpc_roms_out/BG2_zc_list.memh", bg2_zc_list);
	end
	
	reg  [15:0] rowptr_q, col_q, shift_q, zc_q;
	reg  [15:0] rowptr_q2; // czasem wygodne na row_ptr[row+1]
	reg			rowptr_rdy,col_rdy,shift_rdy,zc_rdy;
	reg  [15:0] rowptr_addr;
	wire [15:0] rowptr_addr2 = rowptr_addr + 16'b1;
	reg  [15:0] col_addr;
	reg  [31:0] shift_addr;
	reg  [15:0] zc_addr;
	reg  [31:0] shift_base;
	
	// CB RAM (buffer)
	
	reg  [63:0] cb_mem [0:1023]; 		  // memory ofr CBs
	reg  [3:0]  cb_num;					  // current CB
	reg  [3:0]  cb_count;				  // how many CB
	reg  [9:0]  wr_addr_cb;			  	  // write memory address
	reg  [7:0]  cb_words [0:15]; 		  // how many words cb has
	reg  [9:0]  cb_first_addr[0:15];   // address of first cb chunk
	reg  [9:0]  cb_last_addr[0:15];    // address of last cb chunk
	reg  [15:0] k;					  	     // k bits per cb (with fillers)
	reg  [8:0]  zc; 				        // LDPC lifting value
	reg			bg;
	wire [15:0]	kb = bg ? 16'd22 : 16'd10;
	wire [15:0]	pb = bg ? 16'd46 : 16'd42;
	reg  [15:0] filler_num;  		  // number of filler bits in cb
	wire [15:0] data_bits = k-filler_num;
	reg			mem_done;
	reg			first_cb_word_out;
	
	// RAM READ HANDLING
	reg  [63:0] rd_data_cb;
	reg  [9:0]  rd_addr_cb;
	reg         rd_cb_rdy;
	reg  [63:0] prev;						  // previous word
	reg  [63:0] curr;						  // current word
	
	// PARITY CALC
	reg  [63:0] parity_blocks [0:4095]; // memory for parity blocks
	reg  [11:0]	wr_addr_pb;
	reg  [11:0] rd_addr_pb;
	reg  [63:0] rd_data_pb;
	reg			rd_pb_rdy;
   reg			wr_pb_en;
	reg  [63:0] wr_data_pb;
	reg  [63:0] syndromes [0:24];      // first 4 syndromes used for calc
	reg  [63:0] syndromes_pb [0:24];   // first 4 parity blocks used for calc
	reg  [63:0]	buffer [0:5];			  // 6 words per one block (max 6 words depends on zc)
	reg  [63:0]	acc [0:5];				  // XOR accumulator
	wire [2:0]  active_words = (zc + 9'd63) >> 6;  // 1..6
	reg  [3:0]  i_buf;
	reg  [3:0]  i_bufrot;
	reg  [63:0]	rot_buffer [0:5];
	
	//ZC BLOCK HANDLING
	reg  [15:0]	 start_bit;
	wire [5:0]   bit_off = start_bit[5:0];
	wire [9:0]   start_word = start_bit >> 6;
	reg  [15:0]  zc_idx;
	reg  [15:0]  e0;
	reg  [15:0]  e1;
	reg  [15:0]  rot;
	reg  [2:0] 	 wshift;   // rot/64   (0..5)
	reg  [5:0] 	 bshift;   // rot%64
	reg  [7:0]   col;
	wire [5:0]   zc_mod_w = zc[5:0];
	wire [63:0]  zc_mask = (zc_mod_w==0) ? ~64'b0 : (~64'b0 << (6'd64-zc_mod_w)); 
	reg  [2:0]	 pb14;
	reg			 pb_done;
	reg  [4:0]   i0,i1,i2;
	
	reg  [2:0]	 pb14_out;
	reg  [2:0]	 i_pb;
	
	integer ii;
	
	always @(posedge clk) begin
		if (~rst) begin
	 // ---------------- OUTPUTS ----------------
		 out_valid       <= 1'b0;
	
		 // ---------------- FSM STATES ----------------
		 state_fsm1      <= MEM_IDLE;
		 state_fsm2      <= PARITY_IDLE;
		 state_fsm3      <= LETGO_IDLE;
	
		 // ---------------- FLAGS / RDY ----------------
		 mem_done        <= 1'b0;
		 zc_rdy          <= 1'b0;
		 rowptr_rdy      <= 1'b0;
		 col_rdy         <= 1'b0;
		 shift_rdy       <= 1'b0;
		 rd_cb_rdy       <= 1'b0;
		 rd_pb_rdy       <= 1'b0;
		 wr_pb_en        <= 1'b0;
		 pb_done         <= 1'b0;
		 first_cb_word_out	  <= 1'b0;
	
		 // ---------------- INPUT LATCHED PARAMS ----------------
		 k               <= 16'd0;
		 zc              <= 9'd0;
		 bg              <= 1'b0;
		 filler_num	  <= 16'b0;
	
		 // ---------------- ADDRESSES / COUNTERS ----------------
		 wr_addr_cb      <= 10'd0;
		 rd_addr_cb      <= 10'd0;
	
		 wr_addr_pb      <= 12'd0;
		 rd_addr_pb      <= 12'd0;
	
		 cb_num          <= 4'd0;
		 cb_count        <= 4'd0;
	
		 rowptr_addr     <= 16'd0;
		 col_addr        <= 16'd0;
		 shift_addr      <= 32'd0;
		 zc_addr         <= 16'd0;
		 shift_base      <= 32'd0;
	
		 // ---------------- PIPE / DATA REGS ----------------
		 rd_data_cb      <= 64'd0;
		 rd_data_pb      <= 64'd0;
		 prev            <= 64'd0;
		 curr            <= 64'd0;
	
		 // ---------------- PARITY / LOOP REGS ----------------
		 pb14            <= 3'd0;
		 pb14_out        <= 3'd0;
		 wr_data_pb      <= 64'd0;
	
		 start_bit       <= 16'd0;
		 zc_idx          <= 16'd0;
		 e0              <= 16'd0;
		 e1              <= 16'd0;
	
		 rot             <= 16'd0;
		 wshift          <= 3'd0;
		 bshift          <= 6'd0;
		 col             <= 8'd0;
	
		 i_buf           <= 4'd0;
		 i_bufrot        <= 4'd0;
		 i0              <= 5'd0;
		 i1              <= 5'd0;
		 i2              <= 5'd0;
		 i_pb				  <= 3'd0;
		
		 // ---------------- SMALL ARRAYS (SAFE TO CLEAR) ----------------
		 for (ii = 0; ii < 6; ii = ii + 1) begin
			buffer[ii]     <= 64'd0;
			rot_buffer[ii] <= 64'd0;
			acc[ii]        <= 64'd0;
		 end
	
		 for (ii = 0; ii < 25; ii = ii + 1) begin
			syndromes[ii]    <= 64'd0;
			syndromes_pb[ii] <= 64'd0;
		 end
	
		 // ---------------- CB METADATA ARRAYS ----------------
		 for (ii = 0; ii < 16; ii = ii + 1) begin
			cb_words[ii]       <= 8'd0;
			cb_first_addr[ii]  <= 10'd0;
			cb_last_addr[ii]   <= 10'd0;
		 end
	
		 // ---------------- OPTIONAL: ROM OUTPUT REGS (ŻEBY NIE BYŁO X) ----------------
		 rowptr_q        <= 16'd0;
		 rowptr_q2       <= 16'd0;
		 col_q           <= 16'd0;
		 shift_q         <= 16'd0;
		 zc_q            <= 16'd0;
	  end else begin
			
			if (bg) begin
			 rowptr_q  <= bg1_row_ptr[rowptr_addr];
			 rowptr_q2 <= bg1_row_ptr[rowptr_addr2];
			 col_q     <= bg1_col_idx[col_addr];
			 shift_q   <= bg1_shift[shift_addr];
			 zc_q      <= bg1_zc_list[zc_addr];
		  end else begin
			 rowptr_q  <= bg2_row_ptr[rowptr_addr];
			 rowptr_q2 <= bg2_row_ptr[rowptr_addr2];
			 col_q     <= bg2_col_idx[col_addr];
			 shift_q   <= bg2_shift[shift_addr];
			 zc_q      <= bg2_zc_list[zc_addr];
		  end
			
			
			if (in_valid) begin
				cb_mem[wr_addr_cb] <= in_chunk;
				wr_addr_cb 			 	 <= wr_addr_cb + 10'b1;
			end
			rd_data_cb <= cb_mem[rd_addr_cb];  // synchroniczny odczyt (zarejestrowany)
			
			if (wr_pb_en) begin
				parity_blocks[wr_addr_pb] <= wr_data_pb;
				wr_addr_pb 	<= wr_addr_pb + 12'b1;
			end
			rd_data_pb <= parity_blocks[rd_addr_pb];  // synchroniczny odczyt (zarejestrowany)
			
			
			
			out_valid       	 <= 1'b0;
			out_last			 	 <= 1'b0;
			mem_done			 	 <= 1'b0;
			pb_done         	 <= 1'b0;
			wr_pb_en			 	 <= 1'b0;
			out_pad_left		 <= 6'b0;
			out_pad_right 		 <= 6'b0;
			
			
// ------------------------------ TB SAVING FSM ---------------------------------
			case (state_fsm1)
				MEM_IDLE: begin
					if(in_valid) begin
						k							 <= in_k;
						zc							 <= in_zc;
						bg							 <= in_bg;
						filler_num			 	 <= in_filler_num;
						cb_words[cb_num] 	 	 <= 8'b1;
						cb_first_addr[cb_num] <= wr_addr_cb;
						if (in_last) begin
							cb_num			 <= cb_num + 8'b1;
							cb_last_addr[cb_num] <= wr_addr_cb;
							if (in_last_cb) begin
								cb_count		 <= cb_num + 8'b1;
								mem_done 	 <= 1'b1;
							end
						end else begin
							state_fsm1		 <= MEM_COLLECTING;
						end
					end
				end
				
				MEM_COLLECTING: begin
					if(in_valid) begin
						cb_words[cb_num] 	 	 <= cb_words[cb_num] + 1'b1;
						if (in_last) begin
							cb_num			 <= cb_num + 8'b1;
							cb_last_addr[cb_num] <= wr_addr_cb;
							state_fsm1		 <= MEM_IDLE;
							if (in_last_cb) begin
								cb_count		 <= cb_num + 8'b1;
								mem_done 	 <= 1'b1;
							end
						end
					end
				end
				
				default: begin
					state_fsm1	<= MEM_IDLE;
				end
			endcase
// ------------------------------ TB SAVING FSM ---------------------------------
			case (state_fsm2)
				PARITY_IDLE: begin
					if (mem_done) begin
						cb_num	  <= 4'b0;
						rd_addr_cb <= 10'b0;
						zc_addr	  <= 16'b0;
						zc_rdy	  <= 1'b0;
						state_fsm2 <= PARITY_FIND_ZC;		
					end
				end

				PARITY_FIND_ZC: begin
					if (zc_rdy) begin
						if (zc_q == {7'd0, zc}) begin
							zc_idx 		  <= zc_addr - 16'b1;
							shift_base 	  <= (zc_addr - 16'b1) * (bg ? BG1_NNZ : BG2_NNZ);
							rd_addr_cb 	  <= rd_addr_cb + 10'b1;
							state_fsm2	  <= PARITY_FIRST_WINDOW_PREP;
						end else begin
							zc_addr <= zc_addr + 16'b1;
						end
					end else begin
						zc_rdy  <= 1'b1;
						zc_addr <= zc_addr +16'b1;
					end
				end
				
				PARITY_FIRST_WINDOW_PREP: begin
					prev			  <= rd_data_cb;
					state_fsm2	  <= PARITY_CB_BEGIN;
				end
				
				
				PARITY_CB_BEGIN: begin
					curr			  <= rd_data_cb;
					start_bit	  <= 16'b0;
					rowptr_addr	  <= 16'b0;
					rowptr_rdy	  <= 1'b0;
					state_fsm2 <= PARITY_ROWPTR;
				end
				
				PARITY_ROWPTR: begin
					if (rowptr_rdy) begin
						e0 			<= rowptr_q;
						e1				<= rowptr_q2 - 16'b1;
						col_addr		<= rowptr_q;
						col_rdy		<= 1'b0;
						shift_addr	<= shift_base + rowptr_q;
						shift_rdy	<= 1'b0;
						state_fsm2  <= PARITY_SHIFT;
					end else begin
						rowptr_rdy <= 1'b1;
					end
				end
				
				PARITY_SHIFT: begin
					if (col_rdy & shift_rdy) begin
						col			<= col_q;
						rot			<= shift_q;
						wshift	   <= shift_q[8:6];
						bshift	   <= shift_q[5:0];
						if (col_q >= kb) begin
							if (rowptr_addr < 16'd4) begin
								state_fsm2	<= PARITY_EQUATION_PREP_SYNDROMES;
							end else begin
								buffer[0]	<= syndromes_pb[(col_q - kb)*6]; 
								buffer[1]	<= syndromes_pb[(col_q - kb)* 6 + 1 ]; 
								buffer[2]	<= syndromes_pb[(col_q - kb)* 6 + 2]; 
								buffer[3]	<= syndromes_pb[(col_q - kb)* 6 + 3]; 
								buffer[4]	<= syndromes_pb[(col_q - kb)* 6 + 4]; 
								buffer[5]	<= syndromes_pb[(col_q - kb)* 6 + 5];
								i_buf 		<= active_words;
								state_fsm2	<= PARITY_FILL_BUFF;
							end
						end else begin
							start_bit	<= col_q * zc;
							state_fsm2	<= PARITY_READ_WORDS;
						end
					end else begin
						col_rdy 		<= 1'b1;
						shift_rdy	<= 1'b1;
					end
				end
				
				PARITY_READ_WORDS: begin
					if (rd_addr_cb == cb_first_addr[cb_num] + start_word + 10'b1) begin
						curr		   <= (rd_addr_cb > cb_last_addr[cb_num]) ? 64'b0 : rd_data_cb;
						i_buf			<= 4'b0;
						state_fsm2	<= PARITY_FILL_BUFF;
 					end else begin
						if (cb_first_addr[cb_num]+start_word > cb_last_addr[cb_num]) begin
							prev			<= 64'b0;
							rd_addr_cb   <= cb_first_addr[cb_num] + start_word + 10'b1;
						end else begin
							rd_addr_cb   <= cb_first_addr[cb_num] + start_word;
							rd_cb_rdy 	 <= 1'b0;
							state_fsm2	<= PARITY_WINDOW_PREP;
						end
					end
				end
				
				PARITY_WINDOW_PREP: begin
					if (rd_cb_rdy) begin
						prev			  <= rd_data_cb;
						state_fsm2	  <= PARITY_READ_WORDS;
					end else begin
						rd_cb_rdy	  <= 1'b1;
						rd_addr_cb <= rd_addr_cb + 10'b1;
					end
				end
				
				PARITY_FILL_BUFF: begin
					if (i_buf==active_words) begin
						buffer[i_buf-4'b1] <= buffer[i_buf-4'b1] & zc_mask; 
						i_bufrot	  			 <= 1'b0;
						i0			  			 <= wshift;
						i1			  			 <= (wshift + 3'd1 == active_words) ? 3'd0 : wshift + 3'd1; 
						i2			  			 <= (wshift + 3'd2 >= active_words) ? wshift + 3'd2 - active_words  : wshift + 3'd2;
						state_fsm2 			 <= (rot==16'b0) ? PARITY_XOR : PARITY_ROTATE;
					end else begin
						buffer[i_buf] 	  <= stitch64(prev,curr,bit_off); 
						i_buf 		  	  <= i_buf + 4'b1;
						prev 				  <= curr;
						rd_addr_cb		  <= rd_addr_cb + 10'b1;
						rd_cb_rdy		  <= 1'b0;
						state_fsm2	 	  <= PARITY_FILL_BUFF_WINDOW_MOVE;
					end
				end
				
				PARITY_FILL_BUFF_WINDOW_MOVE: begin
					if(rd_cb_rdy) begin
						rd_cb_rdy  <= 1'b0;
						curr		  <= (rd_addr_cb > cb_last_addr[cb_num]) ? 64'b0 : rd_data_cb ;
						state_fsm2 <= PARITY_FILL_BUFF;
					end else begin
						rd_cb_rdy <= 1'b1;
					end
				end
				
				PARITY_ROTATE: begin
					if (i_bufrot == active_words) begin
						buffer[0] 		<= rot_buffer [0];
						buffer[1] 		<= rot_buffer [1];
						buffer[2] 		<= rot_buffer [2];
						buffer[3] 		<= rot_buffer [3];
						buffer[4] 		<= rot_buffer [4];
						buffer[5] 		<= rot_buffer [5];
						state_fsm2 		<= pb14 ?  PARITY_EQUATION : PARITY_XOR;
					end else begin	
						if (i0  == active_words - 4'b1) begin
							rot_buffer[i_bufrot] <= (zc_mod_w == 6'b0) ? stitch64(buffer[i0], buffer[i1], bshift) : stitch64_2 (buffer[i0], buffer[i1], bshift, zc_mod_w);
							bshift					<= (zc_mod_w == 6'b0) ? bshift : bshift + (6'd64 - zc_mod_w);
							i_bufrot <= i_bufrot + 4'b1;
							i0			<= ( i0 + 4'b1 == active_words) ? 4'b0 : i0 + 4'b1;
							i1			<= ( i1 + 4'b1 == active_words) ? 4'b0 : i1 + 4'b1;
							i2			<= ( i2 + 4'b1 == active_words) ? 4'b0 : i2 + 4'b1; 
						end else if (i1  == active_words - 4'b1) begin
									 if (zc_mod_w >= bshift || zc_mod_w == 6'b0) begin
										rot_buffer[i_bufrot] <= stitch64(buffer[i0], buffer[i1], bshift);
										i_bufrot <= i_bufrot + 4'b1;
										if ( zc_mod_w == bshift && zc_mod_w > 6'b0) begin
											bshift					<= 6'b0;
											i0							<= 4'b0;
											i1							<= (active_words < 4'd2) ? 4'd0 : 4'd1; 
											i2							<= (active_words <= 4'd2) ? 4'd0 : 4'd2; 
										end else begin
											i0			<= ( i0 + 4'b1 == active_words) ? 4'b0 : i0 + 4'b1;
											i1			<= ( i1 + 4'b1 == active_words) ? 4'b0 : i1 + 4'b1;
											i2			<= ( i2 + 4'b1 == active_words) ? 4'b0 : i2 + 4'b1;
										end
										 
									 end else begin
										rot_buffer[i_bufrot] <= stitch64_3(buffer[i0],buffer[i1],buffer[i2], bshift,zc_mod_w);
										bshift				 	<= bshift - zc_mod_w;
										i_bufrot					<= i_bufrot+ 4'd1;
										i0							<= 4'b0;
										i1							<= (active_words < 4'd2) ? 4'd0 : 4'd1; 
										i2							<= (active_words <= 4'd2) ? 4'd0 : 4'd2; 
									 end
								end else begin
									 rot_buffer[i_bufrot] <= stitch64(buffer[i0], buffer[i1], bshift);
									 i_bufrot <= i_bufrot + 4'b1;
									 i0			<= ( i0 + 4'b1 == active_words) ? 4'b0 : i0 + 4'b1;
									 i1			<= ( i1 + 4'b1 == active_words) ? 4'b0 : i1 + 4'b1;
									 i2			<= ( i2 + 4'b1 == active_words) ? 4'b0 : i2 + 4'b1; 
								end
						 end
						 
					end
				
				PARITY_XOR: begin
					acc[0] 	<= acc[0] ^ buffer[0];
					acc[1] 	<= acc[1] ^ buffer[1];
					acc[2] 	<= acc[2] ^ buffer[2];
					acc[3] 	<= acc[3] ^ buffer[3];
					acc[4] 	<= acc[4] ^ buffer[4];
					acc[5] 	<= acc[5] ^ buffer[5];
					
					if ((e0 + 16'b1) < e1) begin
						e0 			<= e0 + 16'b1;
						col_addr		<= e0 + 16'b1;
						col_rdy		<= 1'b0;
						shift_addr	<= shift_base + e0 + 16'b1;
						shift_rdy	<= 1'b0;
						state_fsm2  <= PARITY_SHIFT;
					end else begin
						rowptr_addr	<= rowptr_addr + 16'b1;
						rowptr_rdy 	<= 1'b0;
						state_fsm2	<= PARITY_SAVE_PB;
						i_pb			<= 3'b0;
					end
				end
				
				PARITY_EQUATION_PREP_SYNDROMES: begin
					acc[0] 	<= 64'b0;
					acc[1] 	<= 64'b0;
					acc[2] 	<= 64'b0;
					acc[3] 	<= 64'b0;
					acc[4] 	<= 64'b0;
					acc[5] 	<= 64'b0;
					syndromes [rowptr_addr*6] 		<= acc[0];
					syndromes [rowptr_addr*6 + 1] <= acc[1];
					syndromes [rowptr_addr*6 + 2] <= acc[2];
					syndromes [rowptr_addr*6 + 3] <= acc[3];
					syndromes [rowptr_addr*6 + 4] <= acc[4];
					syndromes [rowptr_addr*6 + 5] <= acc[5];
					rowptr_addr	<= rowptr_addr + 16'b1;
					rowptr_rdy 	<= 1'b0;
					state_fsm2 							<= (rowptr_addr == 16'd3) ? PARITY_EQUATION_PREP_ROT : PARITY_ROWPTR;
					if (rowptr_addr == 16'd3) begin
						pb14 			<= 3'b1;
						shift_rdy	<= 1'b0;
						shift_addr  <= bg ? shift_base + 32'd35 : shift_base + 32'd23;
						buffer[0] 	<= syndromes [0] ^ syndromes[6] ^ syndromes [12] ^ acc[0];
						buffer[1] 	<= syndromes [1] ^ syndromes[7] ^ syndromes [13] ^ acc[1];
						buffer[2] 	<= syndromes [2] ^ syndromes[8] ^ syndromes [14] ^ acc[2];
						buffer[3] 	<= syndromes [3] ^ syndromes[9] ^ syndromes [15] ^ acc[3];
						buffer[4] 	<= syndromes [4] ^ syndromes[10] ^ syndromes [16] ^ acc[4];
						buffer[5] 	<= syndromes [5] ^ syndromes[11] ^ syndromes [17] ^ acc[5];
					end
				end
				
				PARITY_EQUATION_PREP_ROT: begin
					if (shift_rdy) begin
						if (shift_q > 16'b0) begin
							rot			<= (pb14 == 3'b1) ? zc - shift_q : shift_q;
							wshift	   <= (pb14 == 3'b1) ? ((zc - shift_q) >> 6) : shift_q[8:6];
							bshift	   <= (pb14 == 3'b1) ? zc - shift_q : shift_q[5:0];
							i_buf			<= active_words;
							state_fsm2 	<= PARITY_FILL_BUFF;
						end else begin
							rot			<= shift_q;
							state_fsm2	<= PARITY_EQUATION;
						end
					end else begin
						shift_rdy 	<= 1'b1;
					end
				end
				
				PARITY_SAVE_PB: begin
					if (i_pb == active_words) begin
						acc[0] 	<= 64'b0;
						acc[1] 	<= 64'b0;
						acc[2] 	<= 64'b0;
						acc[3] 	<= 64'b0;
						acc[4] 	<= 64'b0;
						acc[5] 	<= 64'b0;
						if (rowptr_addr == pb) begin
							if (cb_num + 4'b1 == cb_count) begin
								pb_done			<= 1'b1;
								state_fsm2		<= PARITY_IDLE;
							end else begin
								cb_num 		  <= cb_num + 4'b1;
								rd_addr_cb 	  <= rd_addr_cb + 10'b1;
								state_fsm2	  <= PARITY_FIRST_WINDOW_PREP;
							end
							
						end else begin 
							state_fsm2		<= PARITY_ROWPTR;
						end
					end else begin
						wr_pb_en 		<= 1'b1;
						wr_data_pb		<= acc[i_pb];
						i_pb				<= i_pb + 4'b1;
					end
				end
				
				PARITY_EQUATION: begin
					
					case (pb14)
						3'd1: begin
							syndromes_pb[0] 		<= buffer[0];
							syndromes_pb[1]	 	<= buffer[1];
							syndromes_pb[2]	 	<= buffer[2];
							syndromes_pb[3]	 	<= buffer[3];
							syndromes_pb[4]	 	<= buffer[4];
							syndromes_pb[5]	 	<= buffer[5];
							shift_rdy				<= 1'b0;
							shift_addr  			<= bg ? shift_base + 32'd17 : shift_base + 32'd6;
							pb14						<= 3'd2;
							state_fsm2 				<=PARITY_EQUATION_PREP_ROT;
						end
						
						3'd2: begin
							syndromes_pb[6] 		<= syndromes[0] ^ buffer[0];
							syndromes_pb[7]	 	<= syndromes[1] ^ buffer[1];
							syndromes_pb[8]	 	<= syndromes[2] ^ buffer[2];
							syndromes_pb[9]	 	<= syndromes[3] ^ buffer[3];
							syndromes_pb[10]	 	<= syndromes[4] ^ buffer[4];
							syndromes_pb[11]	 	<= syndromes[5] ^ buffer[5];
							syndromes_pb[18] 		<= syndromes[18] ^ buffer[0];
							syndromes_pb[19]	 	<= syndromes[19] ^ buffer[1];
							syndromes_pb[20]	 	<= syndromes[20] ^ buffer[2];
							syndromes_pb[21]	 	<= syndromes[21] ^ buffer[3];
							syndromes_pb[22]	 	<= syndromes[22] ^ buffer[4];
							syndromes_pb[23]	 	<= syndromes[23] ^ buffer[5];
							pb14						<= 3'd3;
						end
						
						3'd3: begin
							syndromes_pb[12] 		<= bg ? syndromes[12] ^ syndromes_pb[18] : syndromes[6] ^ syndromes_pb[6];
							syndromes_pb[13] 		<= bg ? syndromes[13] ^ syndromes_pb[19] : syndromes[7] ^ syndromes_pb[7];
							syndromes_pb[14] 		<= bg ? syndromes[14] ^ syndromes_pb[20] : syndromes[8] ^ syndromes_pb[8];
							syndromes_pb[15] 		<= bg ? syndromes[15] ^ syndromes_pb[21] : syndromes[9] ^ syndromes_pb[9];
							syndromes_pb[16] 		<= bg ? syndromes[16] ^ syndromes_pb[22] : syndromes[10] ^ syndromes_pb[10];
							syndromes_pb[17] 		<= bg ? syndromes[17] ^ syndromes_pb[23] : syndromes[11] ^ syndromes_pb[11];
							pb14_out					<= 3'b0;
							i_pb						<= 3'd0;
							pb14					   <= 3'd4;
						end
						
						3'd4: begin
							if (pb14_out == 3'd4) begin
								pb14_out			<= 3'b0;
								i_pb  			<= 3'b0;
								pb14				<= 3'b0;
							end else begin
								wr_pb_en 		<= 1'b1;
								wr_data_pb		<= syndromes_pb[5'd6*pb14_out+i_pb];
								i_pb  			<= (i_pb + 3'b1 ==active_words) ? 3'b0 : i_pb + 3'b1;
								pb14_out			<= (i_pb + 3'b1 ==active_words) ? pb14_out + 3'b1 : pb14_out;
								out_pad_right	<= ((i_pb + 3'b1 ==active_words) && (zc_mod_w > 6'b0)) ? 6'd64 - zc_mod_w	: 6'b0;
							end
						end
						
						default : begin
							state_fsm2 				<=PARITY_ROWPTR;
						end
					endcase
				end
				
				
				default: begin
					state_fsm2	<= PARITY_IDLE;
				end
			endcase
			
			case (state_fsm3)
				LETGO_IDLE: begin
					if(pb_done) begin
						start_bit	<=  16'd2 * zc;
						rd_addr_pb	<= 16'b0;
						rd_pb_rdy	<= 1'b0;
						rd_cb_rdy	<= 1'b0;
						cb_num		<= 4'b0;
						i_pb			<= 3'b0;
						state_fsm3	<= LETGO_OUT_CB_PREP;
					end
				end
				
				LETGO_OUT_CB_PREP: begin
					first_cb_word_out <=  1'b1;
					rd_cb_rdy			<= 1'b0;	
					rd_addr_cb			<= cb_first_addr[cb_num] + start_word;
					state_fsm3  		<= LETGO_OUT_CB; 
				end
				
				LETGO_OUT_CB: begin
					if (rd_cb_rdy) begin
						if (rd_addr_cb > cb_last_addr[cb_num]) begin
							out_pad_right		<= 6'd64 - data_bits[5:0];
							if (first_cb_word_out) begin
								out_valid			 <= 1'b1;
								out_chunk			 <= rd_data_cb;
								out_pad_left		 <= bit_off;
								first_cb_word_out	 <= 1'b0;
							end else begin
								out_valid			 <= 1'b1;
								out_chunk			 <= rd_data_cb;
								rd_addr_pb		<= rd_addr_pb + 16'b1;
								state_fsm3			 <= LETGO_OUT_PB;
							end
						end else begin
							if (first_cb_word_out) begin
								out_valid			 <= 1'b1;
								out_chunk			 <= rd_data_cb;
								out_pad_left		 <= bit_off;
								first_cb_word_out	 <= 1'b0;
								rd_addr_cb			 <= rd_addr_cb + 16'b1;
							end else begin
								out_valid		<= 1'b1;
								out_chunk		<= rd_data_cb;
								rd_addr_cb		<= rd_addr_cb + 16'b1;
							end
						end
					end else begin
						rd_cb_rdy  <= 1'b1;
						rd_addr_cb <= rd_addr_cb + 16'b1;
					end
					
				end
				
//				LETGO_OUT_PB_SYNDROMES: begin
//					if (pb14_out == 3'd4) begin
//						pb14_out			<= 3'b0;
//						i_pb  			<= 3'b0;
//						rd_addr_pb		<= rd_addr_pb + 16'b1;
//						state_fsm3		<= LETGO_OUT_PB;
//					end else begin
//						out_valid		<= 1'b1;
//						out_chunk		<= syndromes_pb[5'd6*pb14_out+i_pb];
//						i_pb  			<= (i_pb + 3'b1 ==active_words) ? 3'b0 : i_pb + 3'b1;
//						pb14_out			<= (i_pb + 3'b1 ==active_words) ? pb14_out + 3'b1 : pb14_out;
//						out_pad_right	<= (i_pb + 3'b1 ==active_words) ? 6'd64 - zc_mod_w	: 6'b0;
//					end
//				end
				
				LETGO_OUT_PB: begin
					if (rd_addr_pb == (active_words*pb*(cb_num+4'b1))) begin
						out_valid		<= 1'b1;
						out_chunk		<= rd_data_pb;
						out_pad_right	<= (zc_mod_w > 6'b0) ? 6'd64 - zc_mod_w : 6'b0;
						i_pb  			<= 3'b0;
						out_last			<= (cb_num == cb_count - 4'b1) ? 1'b1 : 1'b0;
						state_fsm3		<= (cb_num == cb_count - 4'b1) ? LETGO_IDLE : LETGO_OUT_CB_PREP;
						cb_num			<= cb_num + 4'b1;
					end else begin
						out_valid		<= 1'b1;
						out_chunk		<= rd_data_pb;
						i_pb  			<= (i_pb + 3'b1 ==active_words) ? 3'b0 : i_pb + 3'b1;
						out_pad_right	<= ((i_pb + 3'b1 ==active_words) && (zc_mod_w > 6'b0)) ? 6'd64 - zc_mod_w	: 6'b0;
						rd_addr_pb		<= rd_addr_pb + 16'b1;
					end
				end
				
				default: begin
					state_fsm3	<= LETGO_IDLE;
				end
			endcase
		end
	end
	
	function automatic [63:0] stitch64;
	  input [63:0] w0;   
	  input [63:0] w1;   
	  input [5:0]  offs; 
	  reg   [127:0] pair;
	begin
	  pair = {w0, w1};
	  stitch64 = (offs == 0) ? w0 : ((pair << offs) >> 64);
	end
	endfunction
	
	function automatic [63:0] stitch64_2; //jeśli ostatnie słowo w rot_buf jest pierwszym stitch słowem 
	  input [63:0] w0;   
	  input [63:0] w1;   
	  input [5:0]  offs_l; 
	  input [5:0]  offs_r; //nie do końca offset co ilość valid bitów 
	  reg	  [63:0] w0_t;
	  reg	  [63:0] w1_t;
	 begin
	  w0_t = w0 >> (64 - offs_r);
	  w0_t = w0_t << (offs_l + 64 - offs_r);
	  w1_t = w1 >> (offs_r - offs_l);
	  stitch64_2 = w0_t | w1_t;
	  
	end
	endfunction
	
	function automatic [63:0] stitch64_3; //jeśli ostatnie słowow rot_buf jest drugim stitch słowem ORAZ ma za mało valid bitów aby uzupełnić do 64
	  input [63:0] w0;   
	  input [63:0] w1;
	  input [63:0] w2;
	  input [5:0]  offs_l;
	  input [5:0]  offs_r; //nie do końca offset co ilość valid bitów
	  reg   [127:0] pair;
	  reg	  [63:0]  w01_t;
	  reg	  [63:0]  w2_t;
	begin
		pair = {w0, w1};
		pair = pair << offs_l;
		w01_t = pair[127:64];
		w01_t = w01_t >> (offs_l - offs_r);
		w01_t = w01_t << (offs_l - offs_r);
		w2_t  = w2 >> (64 - (offs_l - offs_r));
		stitch64_3 = w01_t | w2_t;
	end
	endfunction
endmodule