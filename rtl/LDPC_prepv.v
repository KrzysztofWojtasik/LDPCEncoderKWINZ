`timescale 1ns/1ps

// LDPC_prep
// Moduł przygotowania bloku transportowego (TB) do kodowania kanałowego LDPC (NR).
//
// Funkcje:
// - Buforowanie całej ramki TB w pamięci wewnętrznej słowami 64-bitowymi.
// - Wyznaczenie parametrów segmentacji i kodowania LDPC na podstawie długości TB:
//   * wybór długości CRC dla TB (16/24 bity) zależnie od progu 3824,
//   * wybór grafu bazowego (BG) na tej samej podstawie,
//   * obliczenie liczby bloków kodowych (CB), długości kcb, minimalnego Zc oraz dobranego Zc,
//   * wyznaczenie k oraz liczby bitów dopełniających (filler).
// - Przygotowanie strumienia wyjściowego bloków kodowych:
//   * dla wielu CB: pobieranie odpowiedniej liczby bitów z bufora, dopisanie CRC dla CB i emisja słów 64-bitowych,
//   * dla pojedynczego CB: bezpośrednia emisja zawartości TB z bufora.
// - Udostępnienie metadanych (k, Zc, BG, liczba bitów dopełniających) zsynchronizowanych ze strumieniem wyjściowym.
//
// Interfejs wejściowy:
// - in_valid kwalifikuje słowo 64-bitowe na in_chunk.
// - in_last sygnalizuje, że dostarczone słowo jest ostatnim słowem danych TB.
//
// Interfejs wyjściowy:
// - out_valid kwalifikuje słowo 64-bitowe na out_chunk.
// - out_last_chunk sygnalizuje ostatnie słowo danego bloku kodowego na wyjściu.
// - out_last_cb sygnalizuje, że emitowany jest ostatni blok kodowy ramki TB.
// - out_filler_cnt, out_k, out_zc, out_bg przenoszą parametry LDPC dla aktualnie emitowanej sekwencji.

module LDPC_prep(
  input  wire 			  clk,
  input  wire 			  rst,          // synchroniczny reset aktywny w stanie niskim
  
  // Strumień wejściowy
  input  wire [15:0]   tb_len,    	 // długość TB w bitach
  
  input  wire 			  in_valid,     // kwalifikacja słowa wejściowego
  input  wire [63:0]   in_chunk,   	 // słowo danych 64-bit
  input  wire 			  in_last,      // znacznik ostatniego słowa danych TB
  
  
  // Strumień wyjściowy
  output reg  			out_valid,        // kwalifikacja słowa wyjściowego
  output reg [63:0]  out_chunk,        // słowo wyjściowe 64-bit
  output reg  			out_last_chunk,   // znacznik ostatniego słowa bieżącego bloku kodowego
  output reg  			out_last_cb,      // znacznik ostatniego bloku kodowego TB
  output reg [15:0]  out_filler_cnt,   // liczba bitów dopełniających (filler)
  output reg [15:0]  out_k,            // docelowa długość bloku kodowego (w bitach)
  output reg [8:0]	out_zc,           // dobrany współczynnik Zc
  output reg 			out_bg            // wybór grafu bazowego (1: BG1, 0: BG2)
  
  
);
	// Parametry stałe
   localparam [7:0]	 W 						= 64;                 		// szerokość słowa danych
	
	// Automat 1: buforowanie TB w pamięci
	localparam [1:0] 	 MEM_IDLE		 		= 2'd0;
	localparam [1:0] 	 MEM_COLLECTING		= 2'd1;
	localparam [1:0] 	 MEM_FINISHING		  	= 2'd2;
	
	// Automat 2: obliczanie parametrów LDPC (segmentacja i dobór Zc)
	localparam [2:0] 	 S_IDLE   	 			= 3'd0;
	localparam [2:0] 	 S_PREP_CB_COUNT  	= 3'd1;
	localparam [2:0] 	 S_CALC_CB_COUNT  	= 3'd2;
	localparam [2:0] 	 S_PREP_KCB		  		= 3'd3;
	localparam [2:0] 	 S_CALC_KCB 		  	= 3'd4;
	localparam [2:0] 	 S_PREP_ZC_MIN	  		= 3'd5;
	localparam [2:0] 	 S_CALC_ZC_MIN 	 	= 3'd6;
	
	// Automat 3: przygotowanie i emisja bloków kodowych (w tym CRC dla CB)
	localparam [2:0] 	 CB_IDLE   			= 3'd0;
	localparam [2:0] 	 CB_PREP_WINDOW   = 3'd1;
	localparam [2:0] 	 CB_CRC    			= 3'd2;
	localparam [2:0] 	 CB_OUT 	  			= 3'd3;
	localparam [2:0] 	 CB_OUT_TB 			= 3'd4;
	
	
	// Bufor TB (pamięć słów 64-bit)
	reg  [63:0] tb_mem [0:1023]; 	// pamięć
	reg  [9:0]  wr_addr;			  	// adres zapisu
	reg  [9:0]  rd_addr;         	// adres odczytu
	reg  [63:0] rd_data;          // dane odczytane (zarejestrowane)
	reg			rd_rdy;           // znacznik dostępności rd_data w ścieżce sterowania
	reg  [9:0]  tb_words;         // liczba zapisanych słów TB
	reg  [9:0]  tb_last_addr;     // adres ostatniego słowa TB
	reg  [7:0]	tb_last_length;   // liczba ważnych bitów w ostatnim słowie TB (bez dopełnienia)
	
	// Rejestry pomocnicze
	reg   [15:0]  tb_len_r;
	
	// Stany i flagi sterujące
	reg   [1:0]   	state_fsm1;
	reg			  	tb_stored;	  									// flaga: TB zbuforowany w całości
	reg   [2:0]   	state_fsm2;
	reg			  	params_calc;	  									// flaga: parametry LDPC obliczone
	wire			  	tb_ready = tb_stored && params_calc;		// gotowość do emisji CB
	reg 	[2:0]	  	state_fsm3;
	reg	[3:0]	  	cb_idx;         									// indeks aktualnego CB
	reg	[15:0]  	emitt_bits;										// liczba bitów pozostałych do pobrania dla CB
	reg 	[7:0]	  	taken_bits;										// przesunięcie bitowe w bieżącym słowie bufora
	reg	[W-1:0]	w0;													// słowo pomocnicze do składania danych
	
	// Parametry LDPC / segmentacji
	wire          tb_crc24 	  = (tb_len_r > 16'd3824) ? 1'b1 : 1'b0;             				 // wybór CRC TB: 24 bity (1) / 16 bitów (0)
	wire  [5:0]   tb_crc_len  = tb_crc24 ? 6'd24 : 6'd16;   											 // długość CRC TB
	wire  [15:0]  b   		  = tb_len_r + tb_crc_len;         										 // długość TB powiększona o CRC TB
	wire  		  bg          = tb_crc24;							 										 // wybór grafu bazowego
	wire  [13:0]  kcb_max	  = bg ? 14'd8448 : 14'd3840;    										 // maksymalna długość CB (zależnie od BG)
	reg   [8:0]   cb_count;	  																					 // liczba bloków kodowych CB
	reg   [15:0]  bc;   		  																					 // długość po dodaniu CRC dla CB (TB+CRC_TB+CRC_CB)
	reg   [15:0]  kcb;																					 		 // długość danych w CB przed dopełnieniem do k
	wire  [4:0]   kb          = bg ? 5'd22 :																 // liczba bloków danych w CB (zależnie od BG i b)
															(b>640) ? 5'd10 : 
																				  (b>560) ? 5'd9 : 
																										(b>192) ? 5'd8 : 5'd6; 
	reg 	[8:0]	  zc_min;																						 // minimalne Zc (ceil(kcb/kb))
	reg   [8:0]   zc;  																							 // dobrane Zc z listy dopuszczalnych wartości
	reg 	[15:0]  k;																								 // docelowa długość CB po dopełnieniu (k=Zc*kb)
	
	
	
	
	// Blok obliczenia wyniku funkcji ceil oraz reszty z dzielenia
	reg	 		  ceil_rst;
	reg			  ceil_enable;
	reg	[15:0]  ceil_num;
	reg	[15:0]  ceil_den;
	wire	[15:0]  ceil_result;
	wire			  ceil_valid;
	wire  [15:0]  ceil_reminder;
	
	
	Ceil u_Ceil (
    .clk       	(clk),
    .rst       	(ceil_rst),
	 .enable			(ceil_enable),
	 .num				(ceil_num),
	 .den				(ceil_den),
	 .ceil			(ceil_result),
	 .result_valid (ceil_valid),
	 .reminder 		(ceil_reminder)
  );

	// Interfejs do modułu dopisującego CRC dla bloku kodowego (CB)
	reg				crc_rst;
	reg				crc_in_valid;
	reg	[15:0]	crc_cb_len;     // długość danych CB w bitach (bez CRC CB)
	reg	[63:0]	crc_in_chunk;   // słowo wejściowe 64-bit do wyliczania CRC CB
	reg				crc_in_last;    // znacznik ostatniego słowa danych CB
	wire				crc_out_valid;
	wire 	[63:0]	crc_out_chunk;  // słowo wyjściowe po dopisaniu CRC CB
	wire				crc_out_last;   // znacznik ostatniego słowa po dopisaniu CRC CB
	wire	[7:0]		crc_padding;    // liczba bitów dopełnienia w słowie końcowym
	
	crc_cb_parallel crc (
	 .clk				(clk),
	 .rst          (crc_rst),
	 .cb_len       (crc_cb_len),
    .in_valid		(crc_in_valid),
	 .in_chunk		(crc_in_chunk),
	 .in_last		(crc_in_last),
    .out_valid		(crc_out_valid),
	 .out_chunk		(crc_out_chunk),
	 .out_last		(crc_out_last),
	 .padding		(crc_padding)
	);
	
	
	always @(posedge clk) begin
		if (~rst) begin
// ---------------- WYJŚCIA ----------------
		 out_valid       <= 1'b0;
		 out_chunk       <= {W{1'b0}};
		 out_last_chunk  <= 1'b0;
		 out_last_cb     <= 1'b0;
		 out_filler_cnt  <= 16'd0;
		 out_k           <= 16'd0;
		 out_zc			  <= 9'b0;
		 out_bg			  <= 1'b0;
	
// ---------------- PAMIĘĆ TB / FLAGI ----------------
		 wr_addr       <= 10'd0;
		 rd_addr			<= 10'd0;
		 rd_data			<= 64'b0;
		 rd_rdy			<= 1'b0;
		 tb_words      <= 10'd0;
		 tb_last_addr  <= 10'd0;
		 tb_last_length<= 8'd0;
		 tb_stored     <= 1'b0;
	
// ---------------- PARAMETRY LDPC ----------------
		 tb_len_r     <= 16'd0;
		 cb_count     <= 9'd0;
		 bc           <= 16'd0;
		 kcb          <= 16'd0;
		 zc_min       <= 9'd0;
		 zc           <= 9'd0;
		 k            <= 16'd0;
	
		 params_calc  <= 1'b0;
	
// ---------------- STANY AUTOMATÓW ----------------
		 state_fsm1   <= MEM_IDLE;
		 state_fsm2   <= S_IDLE;
		 state_fsm3   <= CB_IDLE;
	
// ---------------- LICZNIKI EMISJI ----------------
		 cb_idx       <= 4'd0;
		 emitt_bits   <= 16'd0;
		 taken_bits   <= 8'd0;
		 w0			  <= 64'b0;
	
// ---------------- STEROWANIE CEIL ----------------
		 ceil_rst     <= 1'b0;
		 ceil_enable  <= 1'b0;
		 ceil_num     <= 16'd0;
		 ceil_den     <= 16'd0;
	
// ---------------- STEROWANIE CRC CB ----------------
		 crc_rst      <= 1'b0;          
		 crc_in_valid <= 1'b0;
		 crc_in_last  <= 1'b0;
		 crc_in_chunk <= {W{1'b0}};
		 crc_cb_len   <= 16'd0;
	
	  end else begin
			
		 // Zapis słowa TB do bufora (kwalifikowany in_valid)
		 if (in_valid) begin
				tb_mem[wr_addr] <= in_chunk;
				wr_addr	 		 <= wr_addr + 10'b1;
			end
		 // Synchroniczny odczyt bufora (dane pojawiają się z rejestracją w rd_data)
		 rd_data  <= tb_mem[rd_addr];  
		 
		 // Domyślnie wyjścia i sterowanie CRC wygaszane w każdym cyklu (sygnały impulsowe)
		 out_valid       <= 1'b0;
		 out_chunk       <= {W{1'b0}};
		 out_last_chunk  <= 1'b0;
		 out_last_cb     <= 1'b0;
	
		 crc_in_valid    <= 1'b0;
		 crc_in_last     <= 1'b0;
			
// ------------------------------ AUTOMAT 1: ZAPIS TB DO BUFORA ---------------------------------
			case (state_fsm1)
				MEM_IDLE: begin
					if(~tb_stored && in_valid) begin
						tb_words 		 <= tb_words + 1'b1;
						if (in_last) begin 
							tb_stored 	 	 <= 1'b1;
							tb_last_addr 	 <= wr_addr;
							state_fsm1   	 <= MEM_FINISHING;
						end else begin
							state_fsm1		 <= MEM_COLLECTING;
						end
					end
				end
				
				MEM_COLLECTING: begin
					if(in_valid) begin
						tb_words 		 <= tb_words + 1'b1;
						if (in_last) begin
							tb_stored 	 	 <= 1'b1;
							tb_last_addr 	 <= wr_addr;
							state_fsm1   	 <= MEM_FINISHING;
						end 
					end
				end
				MEM_FINISHING: begin
					wr_addr 		<= 10'b0;
					state_fsm1	<= MEM_IDLE;
				end
				default: begin
					state_fsm1	<= MEM_IDLE;
				end
			endcase
// ------------------------------ AUTOMAT 2: OBLICZANIE PARAMETRÓW LDPC ---------------------------------
			case (state_fsm2)
				S_IDLE: begin
					// Start obliczeń parametrów po pojawieniu się pierwszego słowa TB
					if(in_valid) begin
						tb_len_r   <= tb_len;
						state_fsm2 <= S_PREP_CB_COUNT;
						ceil_rst   <= 1'b0;
					end
				end
				
				S_PREP_CB_COUNT: begin
					// Wyznaczenie liczby CB: dla b>kcb_max stosowane jest ceil(b/(kcb_max-24))
					if (b>kcb_max) begin
						ceil_num 	<= b;
						ceil_den 	<= kcb_max-14'd24;
						ceil_enable <= 1'b1;
						ceil_rst 	<= 1'b1;
						state_fsm2	<= S_CALC_CB_COUNT;
					end else begin
						// Przypadek bez segmentacji: pojedynczy CB
						cb_count   <= 8'b1; 
						bc			  <= b;
						kcb		  <= b;
						ceil_rst	  <= 1'b1;
						state_fsm2 <= S_PREP_ZC_MIN;
					end
				end
				
				S_CALC_CB_COUNT: begin
					ceil_enable <= 1'b1;
					if(ceil_valid)begin
						cb_count   	<= ceil_result;
						bc			  	<= b + ceil_result*9'd24;
						ceil_enable	<= 1'b0;
						state_fsm2 	<= S_PREP_KCB;
					end
				end
				
				S_PREP_KCB: begin
					// kcb = ceil(bc/cb_count)
					ceil_enable	<= 1'b1;
					ceil_num		<= bc;
					ceil_den		<= cb_count;
					state_fsm2	<= S_CALC_KCB;
				end
				
				S_CALC_KCB: begin
					ceil_enable <= 1'b1;
					if(ceil_valid) begin
						kcb 		   <= ceil_result;
						ceil_enable	<= 1'b0;
						state_fsm2  <= S_PREP_ZC_MIN;
					end
				end
				
				S_PREP_ZC_MIN: begin
					// zc_min = ceil(kcb/kb)
					ceil_enable	<= 1'b1;
					ceil_num	   <= kcb;
					ceil_den	   <= kb;
					state_fsm2	<= S_CALC_ZC_MIN;
				end
				
				S_CALC_ZC_MIN: begin
					ceil_enable <= 1'b1;
					if(ceil_valid) begin
						// Dobór Zc z listy dozwolonych wartości oraz wyznaczenie k=Zc*kb
						zc_min 	   <= ceil_result;
						zc				<= find_zc(ceil_result);
						out_zc		<= find_zc(ceil_result);
						k				<= bg ? 5'd22*find_zc(ceil_result) : 5'd10*find_zc(ceil_result) ;
						ceil_enable <= 1'b0;
						params_calc <= 1'b1;
						state_fsm2  <= S_IDLE;
					end
				end
				default: begin
					state_fsm2		<= S_IDLE;
				end
			endcase
// ------------------------------ AUTOMAT 3: PRZYGOTOWANIE/EMISJA CB ---------------------------------
		case (state_fsm3)
				CB_IDLE: begin
					// Start emisji, gdy TB jest zbuforowany i parametry są gotowe
					if(tb_ready) begin
						out_filler_cnt 	<= k-kcb;
						out_k					<= k;
						out_bg				<= bg;
						out_last_cb			<= (cb_idx == cb_count-1) ?	1'b1 : 1'b0;
						if (cb_count == 1) begin
							// Przypadek pojedynczego CB: emisja zawartości TB bez ścieżki CRC CB
							rd_addr		<= 10'b1;
							rd_rdy		<= 1'b0;
							state_fsm3  <= CB_OUT_TB;
						end else begin
								// Przypadek wielu CB: przygotowanie CRC CB i pobierania okna danych z bufora
								crc_rst			<= 1'b1;
								crc_cb_len		<= kcb-16'd24;
								emitt_bits		<= kcb-16'd24;
								rd_addr		<= (cb_idx == 4'b0) ? 10'b1 : rd_addr;
								rd_rdy		<= 1'b0;
								state_fsm3	<= (cb_idx == 4'b0) ? CB_PREP_WINDOW : CB_CRC;
						end
					end
				end
				
				CB_PREP_WINDOW: begin
					// Zainicjowanie słowa pomocniczego do składania danych bitowo na granicy słów bufora
					if (rd_rdy) begin
						state_fsm3 		<= CB_CRC;		
					end else begin
						rd_rdy	<= 1'b1;
						w0			<= rd_data;
					end
				
				end
				
				
				
				CB_CRC: begin
					// Podawanie danych CB do modułu CRC (słowami 64-bitowymi), z obsługą ostatniego słowa części danych
					out_last_cb	<= (cb_idx == cb_count-1) ?	1'b1 : 1'b0;
					if (rd_rdy) begin
						if (emitt_bits > 16'd64) begin
							crc_in_valid 	<= 1'b1;
							crc_in_chunk	<= stitch64(w0, rd_data, taken_bits);
							rd_addr			<= rd_addr + 10'b1;
							rd_rdy			<= 1'b0;
							emitt_bits  	<= emitt_bits - 16'd64;
							w0					<= rd_data;
						end else begin
							if (emitt_bits == 16'd64) begin
								crc_in_valid 	<= 1'b1;
								crc_in_chunk 	<= stitch64(w0, rd_data, taken_bits);
								crc_in_last		<= 1'b1;
								rd_addr			<= rd_addr + 10'b1;
								rd_rdy			<= 1'b0;
								emitt_bits  	<= emitt_bits - 16'd64;
								w0					<= rd_data;
								state_fsm3		<= CB_OUT;
							end else begin
								crc_in_valid 	<= 1'b1;
								crc_in_last		<= 1'b1;
								crc_in_chunk	<= msb_keep(stitch64(w0, rd_data, taken_bits), emitt_bits);
								if (taken_bits + emitt_bits >= 16'd64) begin
								  rd_addr   	<= rd_addr + 10'd1;
								  rd_rdy			<= 1'b0;
								  w0				<= rd_data;
								  taken_bits <= (taken_bits + emitt_bits - 16'd64);
								end else begin
								  taken_bits <= (taken_bits + emitt_bits);
								end
								emitt_bits		<= 16'b0;
								state_fsm3		<= CB_OUT;
							end
						end
					end else begin
						rd_rdy	<= 1'b1;
					end
					// Emisja strumienia wyjściowego z modułu CRC CB
					if (crc_out_valid) begin
						out_valid 	<= 1'b1;
						out_chunk	<= crc_out_chunk;
					end
					
				end
				CB_OUT: begin
					// Emisja słów po CRC CB oraz zakończenie bieżącego CB
					out_last_cb	<= (cb_idx == cb_count-1) ?	1'b1 : 1'b0;
					if (crc_out_valid) begin
						if (crc_out_last) begin
							out_valid 		<= 1'b1;
							out_chunk		<= crc_out_chunk;
							out_last_chunk	<= 1'b1;
							cb_idx			<= cb_idx + 4'b1;
							crc_rst			<= 1'b0;
							state_fsm3		<= CB_IDLE;
							if(out_last_cb) begin
								// Zakończenie całej ramki TB
								tb_stored	<= 1'b0;
								params_calc <= 1'b0;
								cb_idx 		<= 4'b0;
								rd_addr		<= 10'b0;
								taken_bits	<= 8'b0;
							end
						end else begin
							out_valid 	<= 1'b1;
							out_chunk	<= crc_out_chunk;
						end	
					end
				end
				
				CB_OUT_TB: begin
					// Emisja bufora TB słowo po słowie dla przypadku pojedynczego CB
					out_last_cb	<= 1'b1;
					out_valid	<= 1'b1;
					out_chunk	<= rd_data;
					rd_addr 		<= rd_addr + 10'b1;
					if (rd_addr == tb_last_addr + 10'b1) begin
						out_last_chunk <= 1'b1;
						tb_stored		<= 1'b0;
						params_calc 	<= 1'b0;
						rd_addr			<= 10'b0;
						state_fsm3		<= CB_IDLE;
					end
				end
				
				default: begin
					state_fsm3	<= CB_IDLE;
				end
				
				
			endcase
		end
	end
	
	// Funkcja składania słowa 64-bit z pary kolejnych słów bufora z uwzględnieniem przesunięcia bitowego
	function [63:0] stitch64;
	  input [63:0] w0, w1;
	  input [7:0]  offs;         
	  reg   [127:0] pair;
	  begin
		 pair    = {w0, w1};
		 stitch64 = (pair << offs)>>64;
	  end
	endfunction
	
	// Funkcja maskowania: pozostawienie n najbardziej znaczących bitów (MSB) w słowie 64-bit
	function [63:0] msb_keep;
	  input [63:0] x;
	  input [15:0] n;            // liczba zachowanych bitów MSB (0..64)
	  reg   [63:0] mask;
	  begin
		mask = 64'hFFFF_FFFF_FFFF_FFFF << (16'd64 - n);
		msb_keep = x & mask;
	  end
	endfunction
	
	// Funkcja doboru Zc: wybór najmniejszej dopuszczalnej wartości Zc nie mniejszej niż zc_min
	function [8:0] find_zc(input integer zc_min);
		begin
		  if (zc_min <=   2) find_zc = 2;
		  else if (zc_min <=   3) find_zc = 3;
		  else if (zc_min <=   4) find_zc = 4;
		  else if (zc_min <=   5) find_zc = 5;
		  else if (zc_min <=   6) find_zc = 6;
		  else if (zc_min <=   7) find_zc = 7;
		  else if (zc_min <=   8) find_zc = 8;
		  else if (zc_min <=   9) find_zc = 9;
		  else if (zc_min <=  10) find_zc = 10;
		  else if (zc_min <=  11) find_zc = 11;
		  else if (zc_min <=  12) find_zc = 12;
		  else if (zc_min <=  13) find_zc = 13;
		  else if (zc_min <=  14) find_zc = 14;
		  else if (zc_min <=  15) find_zc = 15;
		  else if (zc_min <=  16) find_zc = 16;
		  else if (zc_min <=  18) find_zc = 18;
		  else if (zc_min <=  20) find_zc = 20;
		  else if (zc_min <=  22) find_zc = 22;
		  else if (zc_min <=  24) find_zc = 24;
		  else if (zc_min <=  26) find_zc = 26;
		  else if (zc_min <=  28) find_zc = 28;
		  else if (zc_min <=  30) find_zc = 30;
		  else if (zc_min <=  32) find_zc = 32;
		  else if (zc_min <=  36) find_zc = 36;
		  else if (zc_min <=  40) find_zc = 40;
		  else if (zc_min <=  44) find_zc = 44;
		  else if (zc_min <=  48) find_zc = 48;
		  else if (zc_min <=  52) find_zc = 52;
		  else if (zc_min <=  56) find_zc = 56;
		  else if (zc_min <=  60) find_zc = 60;
		  else if (zc_min <=  64) find_zc = 64;
		  else if (zc_min <=  72) find_zc = 72;
		  else if (zc_min <=  80) find_zc = 80;
		  else if (zc_min <=  88) find_zc = 88;
		  else if (zc_min <=  96) find_zc = 96;
		  else if (zc_min <= 104) find_zc = 104;
		  else if (zc_min <= 112) find_zc = 112;
		  else if (zc_min <= 120) find_zc = 120;
		  else if (zc_min <= 128) find_zc = 128;
		  else if (zc_min <= 144) find_zc = 144;
		  else if (zc_min <= 160) find_zc = 160;
		  else if (zc_min <= 176) find_zc = 176;
		  else if (zc_min <= 192) find_zc = 192;
		  else if (zc_min <= 208) find_zc = 208;
		  else if (zc_min <= 224) find_zc = 224;
		  else if (zc_min <= 240) find_zc = 240;
		  else if (zc_min <= 256) find_zc = 256;
		  else if (zc_min <= 288) find_zc = 288;
		  else if (zc_min <= 320) find_zc = 320;
		  else if (zc_min <= 352) find_zc = 352;
		  else if (zc_min <= 384) find_zc = 384;
		  else                  find_zc = 0; // wartość poza zakresem doboru
		end
	endfunction
endmodule
