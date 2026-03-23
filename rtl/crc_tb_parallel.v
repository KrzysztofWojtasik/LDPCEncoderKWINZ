// crc_tb_parallel
// Moduł dopisujący sumę kontrolną CRC do bloku transportowego (TB) w równoległym strumieniu 64-bitowym.
//
// Zakres funkcjonalny:
// - Przekazuje kolejne słowa danych 1:1 na wyjście (out_chunk) przy zachowaniu kolejności bitów MSB-first.
// - Wyznacza długość CRC na podstawie długości TB (zgodnie z 3GPP TS 38.212):
//     * in_tb_len > 3824  -> CRC24A (24 bity)
//     * in_tb_len <= 3824 -> CRC16  (16 bitów)
// - Po zakończeniu danych dopisuje CRC w jednym z trybów:
//     * S_APPEND: gdy ostatnie słowo jest pełne (lcs == 64) CRC jest emitowane jako osobne słowo,
//     * S_SERIAL: gdy ostatnie słowo jest niepełne (lcs < 64) końcówka danych oraz CRC są składane bitowo
//       i emitowane jako ostatnie słowa, z dopełnieniem zerami do granicy 64 bitów.
//
// Założenia interfejsu wejściowego:
// - Brak sygnału gotowości (ready): in_valid może być ustawiane wyłącznie, gdy in_chunk/in_last są poprawne i stabilne.
// - in_tb_len musi odpowiadać bieżącej ramce TB i pozostawać stabilne co najmniej od pierwszego słowa do końca ramki.
// - in_last oznacza ostatnie słowo danych TB.
// - Konwencja bitów: MSB-first.
//
// Sygnały wyjściowe:
// - out_valid: kwalifikacja danych na out_chunk w danym cyklu.
// - out_last: impuls w cyklu emisji ostatniego słowa całej ramki (po dopisaniu CRC).
// - out_padding: liczba bitów zer dopisanych na końcu ostatniego słowa (wyrównanie do 64 bitów).
// - out_tb_len: przekazana długość TB bez CRC (wartość ramkowa).

`timescale 1ns/1ps

module crc_tb_parallel(
  input  wire clk,
  input  wire rst,          			 // synchroniczny reset aktywny w stanie niskim
  input  wire [15:0] in_tb_len,      // długość TB w bitach

  // Strumień wejściowy
  input  wire 			 in_valid,      // kwalifikacja danych wejściowych
  input  wire [63:0]  in_chunk,      // 64-bitowe słowo danych (MSB-first)
  input  wire 			 in_last,       // znacznik ostatniego słowa danych TB

  // Strumień wyjściowy
  output reg  			out_valid,      // kwalifikacja wyjścia
  output reg [63:0]  out_chunk,      // słowo danych lub CRC (MSB-first)
  output reg  			out_last,       // impuls na ostatnim słowie ramki po dopisaniu CRC
  output reg [15:0]	out_tb_len      // długość TB bez CRC 
);

	// ============================== FSM / rejestry CRC ====================================
	localparam [7:0] W 		  = 8'd64;     // szerokość słowa danych
	localparam [1:0] S_IDLE   = 2'd0;      // stan spoczynkowy: oczekiwanie na początek ramki
	localparam [1:0] S_DATA   = 2'd1;      // przetwarzanie kolejnych pełnych słów danych
	localparam [1:0] S_APPEND = 2'd2;      // emisja CRC jako osobnego słowa (gdy lcs==64)
	localparam [1:0] S_SERIAL = 2'd3;      // składanie końcówki danych + CRC w trybie bitowym (gdy lcs<64)

	// Wybór wariantu CRC na podstawie długości TB
	wire          use_crc24 	= (in_tb_len > 16'd3824);     	// selekcja CRC24A/CRC16
	wire  [5:0]   crc_len   	= use_crc24 ? 6'd24 : 6'd16;	   // długość CRC w bitach
	wire  [5:0]   rem       	= in_tb_len[5:0];       			// in_tb_len mod 64
	wire  [6:0]   lcs       	= (rem==0) ? 7'd64 : {1'b0,rem}; // liczba ważnych bitów w ostatnim słowie danych
	wire  [7:0]   sum       	= lcs+crc_len;							// liczba bitów: końcówka danych + CRC
	wire  [7:0]   padding_cnt  = (sum <= 8'd64) ? (8'd64 - sum) : (8'd128 - sum);  // dopełnienie zerami do 64/128 bitów

	// Rejestracja parametrów ramki (utrzymanie spójności przy dalszych słowach)
	reg           use_crc24_r;
	reg   [7:0]   lcs_r,padding_cnt_r;

	reg   [1:0]   state;					// rejestr stanu automatu FSM

	// Stan CRC aktualizowany równolegle po 64 bitach (dla pełnych słów)
	reg   [23:0] crc24;               // bieżący stan CRC24A dla TB
	wire  [23:0] crc24_next;			 // CRC24A po dołączeniu kolejnego słowa 64-bit

	reg   [15:0] crc16;               // bieżący stan CRC16 dla TB
	wire  [15:0] crc16_next;			 // CRC16 po dołączeniu kolejnego słowa 64-bit

	// Sygnały dla ścieżki bitowej (tylko dla niepełnego ostatniego słowa)
	reg	        serial_in;           // bit wejściowy do crc_serial (MSB-first)
	reg	[W-1:0] serial_in_reg;       // rejestr przesuwający z ostatnim słowem danych
	reg			  serial_in_valid;     // kwalifikacja bitu serial_in
	reg			  serial_in_last;      // znacznik ostatniego bitu danych TB dla crc_serial

	// Wyjścia crc_serial dla obu długości CRC (emitują strumień: pozostałe dane + CRC)
	wire 			  ser24_valid, ser24_bit, ser24_last;
	wire 			  ser16_valid, ser16_bit, ser16_last;

	// Selekcja aktywnej instancji crc_serial zgodnie z parametrami ramki
	wire 			  serial_out_valid_sel = use_crc24_r ? ser24_valid : ser16_valid;
	wire 			  serial_out_sel       = use_crc24_r ? ser24_bit   : ser16_bit;
	wire 			  serial_out_last_sel  = use_crc24_r ? ser24_last  : ser16_last;

	// Rejestry składania końcowego słowa 64-bit z bitów strumienia crc_serial
	reg			  serial_out_last_seen;  // flaga zakończenia strumienia (ostatni bit CRC)
	reg	[W-1:0] serial_out_reg;       // rejestr budujący końcowe słowo (dane+CRC)
	reg			  serial_crc_enable;     // impuls załadowania stanu CRC do crc_serial (start liczenia końcówki)
	reg			  serial_rst;            // sterowanie resetem crc_serial (aktywny w stanie niskim)

	// Licznik cykli trybu S_SERIAL: steruje serial_in_last oraz momentami emisji out_chunk
	reg   [7:0]	  cnt;

	// Instancje crc_serial: przetwarzanie bitowe dla CRC24A oraz CRC16
	crc_serial #(.L(24)) crc_serial_24 (
    .clk       (clk),
    .rst       (serial_rst),
    .in_valid  (serial_in_valid),
    .in_bit    (serial_in),
    .in_last   (serial_in_last),
	 .custom_crc (crc24),                 // stan CRC po pełnych słowach (stan startowy dla końcówki)
	 .custom_crc_valid (serial_crc_enable),
    .out_valid (ser24_valid),
    .out_bit   (ser24_bit),
    .out_last  (ser24_last)
  );

   crc_serial #(.L(16)) crc_serial_16 (
    .clk       (clk),
    .rst       (serial_rst),
    .in_valid  (serial_in_valid),
    .in_bit    (serial_in),
    .in_last   (serial_in_last),
	 .custom_crc (crc16),                 // stan CRC po pełnych słowach (stan startowy dla końcówki)
	 .custom_crc_valid (serial_crc_enable),
    .out_valid (ser16_valid),
    .out_bit   (ser16_bit),
    .out_last  (ser16_last)
   );

  // Równoległa aktualizacja CRC po 64 bitach; dane interpretowane MSB-first
  crc_update #(
    .L(24),
    .W(W)
  ) crc_update_24 (
    .crc      (crc24),
    .data     (in_chunk),     // MSB-first
    .crc_next (crc24_next)
  );

  crc_update #(
    .L(16),
    .W(W)
  ) crc_update_16 (
    .crc      (crc16),
    .data     (in_chunk),     // MSB-first
    .crc_next (crc16_next)
  );

  // ============================== Sekwencja pracy ========================================
  // FSM steruje przepływem danych i emisją CRC:
  // - S_IDLE: inicjalizacja ramki, zatrzaśnięcie parametrów, decyzja o ścieżce końcowej
  // - S_DATA: przekazywanie danych 1:1 oraz aktualizacja CRC dla pełnych chunków
  // - S_APPEND: emisja CRC jako osobnego słowa 64-bit (CRC w MSB, reszta = 0)
  // - S_SERIAL: bitowe przetwarzanie niepełnej końcówki danych i dopisanie CRC do końcowego słowa
  always @(posedge clk) begin
    if (~rst) begin
      out_valid <= 1'b0;
      out_chunk   <= {W{1'b0}};
      out_last  <= 1'b0;
		out_tb_len  <= 16'b0;

		state      		<= S_IDLE;
		crc24      		<= 24'b0;
		crc16       	<= 16'b0;
		use_crc24_r 	<= 1'b0;
		lcs_r       	<= 7'b0;
		padding_cnt_r  <= 8'b0;

		serial_in 		 <= 1'b0;
		serial_in_reg   <= {W{1'b0}};
		serial_in_valid <=1'b0;
		serial_in_last  <=1'b0;

		serial_out_reg  <= {W{1'b0}};
		serial_out_last_seen <=1'b0;
		serial_crc_enable <=1'b0;
		serial_rst     <= 1'b0;
		cnt				<= 8'b0;

    end else begin
      // Wyjścia impulsowe: domyślnie wygaszane w każdym cyklu
      out_valid 	<= 1'b0;
      out_chunk   <= {W{1'b0}};
      out_last  	<= 1'b0;

		// serial_crc_enable: impuls inicjujący załadowanie stanu CRC do crc_serial
		serial_crc_enable <=1'b0;

      case (state)
        // ------------------------------ IDLE ---------------------------------
        S_IDLE: begin
          crc24 <= 24'b0;          // inicjalizacja CRC na początku ramki
			 crc16 <= 16'b0;          // inicjalizacja CRC na początku ramki
          if (in_valid) begin

				// Zatrzaśnięcie parametrów ramki (obowiązują do końca bieżącej ramki)
				use_crc24_r 	<= use_crc24;
				lcs_r       	<= lcs;
				padding_cnt_r  <= padding_cnt;
				out_tb_len		<= in_tb_len;

				// Rozgałęzienie dla ostatniego słowa danych: tryb dopisania CRC zależny od lcs
				if (in_last) begin
					if (lcs==W) begin
						// Ostatnie słowo pełne: CRC aktualizowane równolegle, następnie osobna emisja CRC (S_APPEND)
						if(use_crc24) begin
							crc24 <= crc24_next;
						end else begin
							crc16 <= crc16_next;
						end
						out_valid <= 1'b1;
						out_chunk   <= in_chunk;
						state       <= S_APPEND;
					end else begin
						// Ostatnie słowo niepełne: przejście do ścieżki bitowej (S_SERIAL)
						serial_in_reg <= in_chunk;
						serial_crc_enable <= 1'b1;
						cnt <= 8'b0;
						serial_rst <= 1'b1;     // aktywacja pracy crc_serial (reset aktywny w 0)
						state 	<= S_SERIAL;
					end
				end else begin
					// Słowo pośrednie: przekazanie 1:1 oraz aktualizacja CRC dla pełnych 64 bitów
					if(use_crc24) begin
						crc24 <= crc24_next;
					end else begin
						crc16 <= crc16_next;
					end
					out_valid <= 1'b1;
					out_chunk   <= in_chunk;
					state  <= S_DATA;
				end
          end
        end

        // ------------------------------ DATA ---------------------------------
        S_DATA: begin
          if (in_valid) begin
				if (in_last) begin
					// Ostatnie słowo danych: wybór ścieżki końcowej zależnie od lcs_r
					if (lcs_r==W) begin
						out_valid <= 1'b1;
						out_chunk   <= in_chunk;
						if(use_crc24_r) begin
							crc24 <= crc24_next;
						end else begin
							crc16 <= crc16_next;
						end
						state       <= S_APPEND;
					end else begin
						// Ostatni chunk niepełny: inicjacja trybu bitowego dla końcówki danych i CRC
						serial_in_reg <= in_chunk;
						serial_crc_enable <= 1'b1;
						cnt <= 8'b0;
						serial_rst <= 1'b1;
						state 	<= S_SERIAL;
					end
				end else begin
					// Chunk pośredni: przekazanie 1:1 i dalsza akumulacja CRC
					out_valid <= 1'b1;
					out_chunk   <= in_chunk;
					if(use_crc24_r) begin
						crc24 <= crc24_next;
					end else begin
						crc16 <= crc16_next;
					end
				end
          end
        end

        // ------------------------------ APPEND (CRC jako osobny chunk) ----------
        S_APPEND: begin

			 // Emisja CRC jako 64-bit: CRC w najbardziej znaczących bitach, pozostałe bity wyzerowane
			 out_valid <= 1'b1;
			 out_last <= 1'b1;     // ostatnie słowo całej ramki (po dopisaniu CRC)
			 if(use_crc24_r) begin
				out_chunk   <= {crc24,{40{1'b0}}};
          end else begin
				out_chunk   <= {crc16,{48{1'b0}}};
			 end

			 state    <= S_IDLE;   // powrót do oczekiwania na następną ramkę
        end

		  //------------------------------ SERIAL (końcówka danych + CRC bitowo) ----------
		  S_SERIAL: begin

			// Wyznaczenie ostatniego bitu danych TB przekazywanego do crc_serial (po lcs_r bitach)
			if (cnt == lcs_r-1) begin
				serial_in_last <= 1'b1;
			end

			// Podanie kolejnego bitu danych do crc_serial (MSB-first)
			serial_in_valid <= 1'b1;
			serial_in <= serial_in_reg[W-1];

			// Emisja pośredniego słowa, gdy suma (końcówka + CRC) przekracza 64 bity (wariant 128-bitowy)
			if (cnt == W+2) begin
				out_valid <= 1'b1;
				out_chunk <= serial_out_reg;
			end

			// Składanie 64-bitowego rejestru wyjściowego z bitów crc_serial (kolejność zachowana przez przesuw)
			if (serial_out_valid_sel) begin
				serial_out_reg <= {serial_out_reg[W-2:0],serial_out_sel};
			end

			// Detekcja końca strumienia bitowego (ostatni bit CRC)
			if (serial_out_last_sel==1)begin
				serial_out_last_seen <= 1'b1;
			end

			// Finalizacja: emisja końcowego słowa z dopełnieniem zerami oraz sygnalizacją out_last
			if (serial_out_last_seen==1) begin
				out_valid <= 1'b1;
				out_last	 <= 1'b1;
				out_chunk <= serial_out_reg << padding_cnt_r;
				serial_crc_enable <= 1'b0;
				serial_in_valid <= 1'b0;
				serial_rst <= 1'b0;      // zatrzymanie i wyzerowanie crc_serial po zakończeniu ramki
				state <= S_IDLE;
			end

			// Przesunięcie rejestru wejściowego w celu wystawienia następnego bitu na MSB
			serial_in_reg <= {serial_in_reg[W-2:0],1'b0};
			cnt <= cnt + 1'b1;

		  end
        default: begin
          state <= S_IDLE;
        end
      endcase
    end
  end

endmodule
