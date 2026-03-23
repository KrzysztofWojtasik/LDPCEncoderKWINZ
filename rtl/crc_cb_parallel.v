// rtl/crc_parallel.v
//
// crc_cb_parallel
// Moduł dopisujący CRC do bloku kodowego (CB) w torze LDPC (wariant równoległy 64-bit).
//
// Interfejs:
// - Dane wejściowe i wyjściowe przesyłane są słowami 64-bitowymi (MSB-first).
// - CRC dla CB: CRC24B (24 bity), następnie dopełnienie zerami do wielokrotności 64 bitów.
//
// Wejście:
// - cb_len   : długość CB w bitach (liczba bitów danych przed CRC)
// - in_valid : ważność słowa wejściowego
// - in_chunk : słowo danych 64-bit (MSB-first)
// - in_last  : znacznik ostatniego słowa danych CB
//
// Wyjście:
// - out_valid : ważność słowa wyjściowego
// - out_chunk : słowo wyjściowe (dane 1:1 lub słowo zawierające CRC/dopełnienie)
// - out_last  : znacznik ostatniego słowa całej ramki (dane+CRC+dopełnienie)
// - padding   : liczba dopisanych bitów '0' w końcowym słowie

`timescale 1ns/1ps

module crc_cb_parallel(
  input  wire clk,
  input  wire rst,
  input  wire [15:0] cb_len,

  // Strumień wejściowy (64 bity na takt)
  input  wire        in_valid,
  input  wire [63:0] in_chunk,
  input  wire        in_last,

  // Strumień wyjściowy (64 bity na takt)
  output reg         out_valid,
  output reg [63:0]  out_chunk,
  output reg         out_last,
  output reg [7:0]   padding
);

	// ============================== Parametry i stany FSM =================================
	localparam [7:0] W       = 8'd64;   // szerokość słowa danych
	localparam [1:0] S_IDLE  = 2'd0;    // stan spoczynkowy / początek bloku
	localparam [1:0] S_DATA  = 2'd1;    // przepuszczanie danych + aktualizacja CRC
	localparam [1:0] S_APPEND= 2'd2;    // dopisanie CRC jako osobne słowo (gdy ostatnie słowo danych było pełne)
	localparam [1:0] S_SERIAL= 2'd3;    // tryb bitowy dla niepełnego ostatniego słowa danych

	// ============================== Obliczenia długości i dopełnienia ======================
	wire  [5:0] crc_len      = 6'd24;                 			 // długość CRC24B
	wire  [5:0] rem          = cb_len[5:0];           			 // reszta z dzielenia cb_len przez 64
	wire  [6:0] lcs          = (rem==0) ? 7'd64 : {1'b0,rem}; // liczba ważnych bitów w ostatnim słowie danych
	wire  [7:0] sum          = lcs + crc_len;         			 // ważne bity: końcówka danych + CRC
	wire  [7:0] padding_cnt  = (sum <= 8'd64) ? (8'd64 - sum) : (8'd128 - sum); // dopełnienie do 64

	reg   [7:0] lcs_r, padding_cnt_r;                 // zatrzaski parametrów na czas przetwarzania jednego CB

	// ============================== CRC równoległy =========================================
	reg   [1:0]  state;
	reg   [23:0] crc;                                 // CRC24B (reszta bieżąca)
	wire  [23:0] crc_next;                            // CRC po przetworzeniu 64 bitów

	// ============================== CRC bitowy (dla końcówki) ===============================
	reg         serial_in;                            // bit podawany do crc_serial
	reg [W-1:0] serial_in_reg;                        // rejestr przesuwający dane końcowe (MSB-first)
	reg         serial_in_valid;
	reg         serial_in_last;

	wire        serial_out_valid;
	wire        serial_out_bit;
	wire        serial_out_last;

	reg         serial_out_last_seen;
	reg [W-1:0] serial_out_reg;                       // składanie słowa wyjściowego bit-po-bicie
	reg         serial_crc_enable;                    // załadowanie wartości startowej CRC do crc_serial
	reg         serial_rst;                           // reset dla crc_serial
	reg  [7:0]  cnt;                                  // licznik cykli w trybie S_SERIAL

	// crc_serial dla CRC24B: parametr L=248 oznacza wariant CRC24B (24 bity)
	crc_serial #(.L(248)) crc_serial (
    .clk             (clk),
    .rst             (serial_rst),
    .in_valid        (serial_in_valid),
    .in_bit          (serial_in),
    .in_last         (serial_in_last),
	  .custom_crc      (crc),
	  .custom_crc_valid(serial_crc_enable),
    .out_valid       (serial_out_valid),
    .out_bit         (serial_out_bit),
    .out_last        (serial_out_last)
  );

  // crc_update: równoległa aktualizacja CRC po 64 bitach (MSB-first)
  crc_update #(
    .L(248),
    .W(W)
  ) crc_update (
    .crc      (crc),
    .data     (in_chunk),
    .crc_next (crc_next)
  );

  // ============================== Sekwencja ==============================================
  always @(posedge clk) begin
    if (~rst) begin
      out_valid <= 1'b0;
      out_chunk <= {W{1'b0}};
      out_last  <= 1'b0;
		  padding   <= 8'b0;

		  state         <= S_IDLE;
		  crc           <= 24'b0;
		  lcs_r         <= 7'b0;
		  padding_cnt_r <= 8'b0;

		  serial_in            <= 1'b0;
		  serial_in_reg        <= {W{1'b0}};
		  serial_in_valid      <= 1'b0;
		  serial_in_last       <= 1'b0;

		  serial_out_reg       <= {W{1'b0}};
		  serial_out_last_seen <= 1'b0;
		  serial_crc_enable    <= 1'b0;
		  serial_rst           <= 1'b0;
		  cnt                 <= 8'b0;

    end else begin
      // wartości domyślne dla wyjść w cyklu (impulsy generowane są w logice stanów)
      out_valid <= 1'b0;
      out_chunk <= {W{1'b0}};
      out_last  <= 1'b0;
		  serial_crc_enable <= 1'b0;

      case (state)

        // ------------------------------ S_IDLE ---------------------------------
        // Inicjalizacja CRC, zatrzask parametrów długości i dopełnienia, decyzja o trybie zakończenia.
        S_IDLE: begin
			 crc <= 24'b0;
          if (in_valid) begin

				// Zatrzask parametrów ramki (stałe dla całego CB)
				lcs_r       	<= lcs;
				padding_cnt_r  <= padding_cnt;
				padding 			<= padding_cnt;

				// Obsługa ostatniego słowa danych CB
				if (in_last) begin
					if (lcs==W) begin
						// Ostatnie słowo pełne: CRC liczone równolegle, CRC emitowane jako osobne słowo
						crc			<= crc_next;
						out_valid 	<= 1'b1;
						out_chunk   <= in_chunk;
						state       <= S_APPEND;
					end else begin
						// Ostatnie słowo niepełne: przejście do trybu bitowego (dogranie końcówki + CRC + dopełnienie)
						serial_in_reg <= in_chunk;
						serial_crc_enable <= 1'b1;
						cnt <= 8'b0;
						serial_rst <= 1'b1;
						state 	<= S_SERIAL;
					end
				end else begin
					// Kolejne słowa danych: przepuszczenie 1:1 i aktualizacja CRC równolegle
					crc			<= crc_next;
					out_valid <= 1'b1;
					out_chunk   <= in_chunk;
					state  <= S_DATA;
				end
          end
        end

        // ------------------------------ S_DATA ---------------------------------
        // Przepuszczanie danych 1:1, równoległa aktualizacja CRC, przejście do APPEND lub SERIAL na końcu CB.
        S_DATA: begin
          if (in_valid) begin
				if (in_last) begin
					if (lcs_r==W) begin
						out_valid <= 1'b1;
						out_chunk   <= in_chunk;
						crc <= crc_next;
						state       <= S_APPEND;
					end else begin
						serial_in_reg <= in_chunk;
						serial_crc_enable <= 1'b1;
						cnt <= 8'b0;
						serial_rst <= 1'b1;
						state 	<= S_SERIAL;
					end
				end else begin
					out_valid <= 1'b1;
					out_chunk   <= in_chunk;
					crc <= crc_next;
				end
          end
        end

        // ------------------------------ S_APPEND ---------------------------------
        // Emisja CRC jako osobne słowo: CRC w najbardziej znaczących bitach, reszta dopełniona zerami.
        S_APPEND: begin
			 out_valid <= 1'b1;
			 out_last <= 1'b1;
			 out_chunk   <= {crc,{40{1'b0}}};
			 state    <= S_IDLE;
			 padding  <= 8'b0;
        end

		  // ------------------------------ S_SERIAL ---------------------------------
		  // Tryb bitowy: podawanie ważnych bitów ostatniego słowa danych i dopisanie CRC przez crc_serial.
		  // Końcowe słowo jest wyrównywane przez przesunięcie o padding_cnt_r.
		  S_SERIAL: begin

			if (cnt == lcs_r-1) begin
				serial_in_last <= 1'b1;
			end
			serial_in_valid <= 1'b1;
			serial_in <= serial_in_reg[W-1];

			if (cnt == W+2) begin
				out_valid <= 1'b1;
				out_chunk <= serial_out_reg;
			end
			if (serial_out_valid) begin
				serial_out_reg <= {serial_out_reg[W-2:0],serial_out_bit};
			end

			if (serial_out_last==1)begin
				serial_out_last_seen <= 1'b1;
			end

			if (serial_out_last_seen==1) begin
				out_valid <= 1'b1;
				out_last	 <= 1'b1;
				out_chunk <= serial_out_reg << padding_cnt_r;
				serial_crc_enable <= 1'b0;
				serial_in_valid <= 1'b0;
				serial_rst <= 1'b0;
				state <= S_IDLE;
			end
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
