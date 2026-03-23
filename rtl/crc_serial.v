// crc_serial
// Szeregowy moduł dopisujący sumę kontrolną CRC do bloku transportowego (TB) 
// - Dane są zawsze przepisywane na wyjście (out_bit=in_bit) przy in_valid.
// - out_last jest podawany tylko raz: na ostatnim bicie CRC.

`timescale 1ns/1ps

module crc_serial #(
  parameter integer L = 24                      // długość CRC: 16, 24 (24A) lub 248 (24B)
)(
  input  wire clk,
  input  wire rst,          // synchroniczny reset; aktywny poziom niski

  // Wejście strumieniowe (1 bit / cykl)
  input  wire in_valid,     // ważność bitu wejściowego w danym cyklu
  input  wire in_bit,       // bit danych (MSB-first) podawany do CRC i jednocześnie przepisywany na wyjście
  input  wire in_last,      // znacznik: ten bit jest ostatnim bitem danych

  // Opcjonalna inicjalizacja rejestru CRC wartością zewnętrzną
  input  wire [((L==248)? 24 : L)-1:0] custom_crc, // wartość startowa CRC (dla L=248 szerokość 24)
  input  wire  custom_crc_valid,                  // 1 => użyj custom_crc zamiast zera przy starcie ramki

  // Wyjście strumieniowe (payload + dopisane bity CRC)
  output reg  out_valid,    // ważność bitu wyjściowego w danym cyklu
  output reg  out_bit,      // bit wyjściowy: najpierw dane, potem bity CRC (MSB-first)
  output reg  out_last      // znacznik: ostatni bit całej ramki (ostatni bit CRC)
);

  // L_LEN: rzeczywista długość CRC w bitach.
  localparam integer  L_LEN = (L==248) ? 24 : L;

  // POLY: wielomian CRC zapisany jako maska XOR dla rejestru (bez wiodącego bitu x^L).
  // Dobór zależny od parametru L (16, 24A, 24B).
  localparam [L_LEN-1:0] POLY = (L==24)  ? 24'h864CFB :
                                (L==16)  ? 16'h1021   :
                                (L==248) ? 24'h800063 :
                                           {L_LEN{1'b0}};

  // ============================================================================
  // Automaty sterujące przepływem: dane -> dopisanie CRC
  //
  // S_IDLE   : oczekiwanie na początek ramki; inicjalizacja CRC (0 lub custom)
  // S_DATA   : przepisywanie danych na wyjście i aktualizacja CRC
  // S_APPEND : emisja bitów CRC po zakończeniu danych (MSB-first)
  // ============================================================================
  localparam [1:0] S_IDLE   = 2'd0;
  localparam [1:0] S_DATA   = 2'd1;
  localparam [1:0] S_APPEND = 2'd2;

  reg [1:0]       state;
  reg [L_LEN-1:0] crc;               // bieżący stan rejestru CRC w trakcie przyjmowania danych
  reg [L_LEN-1:0] crc_latched;       // stan CRC zatrzaśnięty po ostatnim bicie danych (źródło bitów CRC do emisji)
  reg [7:0]       cnt;               // licznik emitowanych bitów CRC

  // ============================================================================
  // Logika aktualizacji CRC dla pojedynczego bitu (MSB-first):
  // feedback = in_bit XOR MSB(crc)
  // crc_shift = przesunięcie rejestru (w lewo) z dopisaniem zera
  // next_crc  = crc_shift XOR POLY gdy feedback==1, inaczej crc_shift
  //
  // ============================================================================
  wire feedback            = in_valid && (in_bit ^ crc[L_LEN-1]);
  wire [L_LEN-1:0] crc_shift = {crc[L_LEN-2:0], 1'b0};
  wire [L_LEN-1:0] next_crc  = feedback ? (crc_shift ^ POLY) : crc_shift;

  // ============================================================================
  // Sekwencja synchroniczna
  // Reset: ustawia stan IDLE, czyści rejestry oraz wygasza wyjścia.
  //
  always @(posedge clk) begin
    if (~rst) begin
      state       <= S_IDLE;
      crc         <= {L_LEN{1'b0}};
      crc_latched <= {L_LEN{1'b0}};
      cnt         <= 8'd0;
      out_valid   <= 1'b0;
      out_bit     <= 1'b0;
      out_last    <= 1'b0;
    end else begin
      // Domyślnie: brak ważnego bitu na wyjściu w danym cyklu.
      out_valid <= 1'b0;
      out_bit   <= 1'b0;
      out_last  <= 1'b0;

      case (state)
        // --------------------------------------------------------------------
        // IDLE: przygotowanie startu ramki i oczekiwanie na pierwszy bit danych
        // --------------------------------------------------------------------
        S_IDLE: begin
          // Inicjalizacja CRC (0 lub wartość zewnętrzna) przed startem ramki
          if (custom_crc_valid) begin
            crc <= custom_crc;
          end else begin
            crc <= {L_LEN{1'b0}};
          end

          if (in_valid) begin
            // Emisja pierwszego bitu danych
            if (custom_crc_valid) begin
              crc <= custom_crc;
            end
            out_valid <= 1'b1;
            out_bit   <= in_bit;

            // Aktualizacja CRC o ten bit
            crc <= next_crc;

            // Jeżeli to był ostatni bit danych, przejście do emisji CRC
            if (in_last) begin
              state       <= S_APPEND;
              crc_latched <= next_crc;
              cnt         <= 8'd0;
            end else begin
              state <= S_DATA;
            end
          end
        end

        // --------------------------------------------------------------------
        // DATA: przepisywanie danych i aktualizacja CRC aż do in_last
        // --------------------------------------------------------------------
        S_DATA: begin
          if (in_valid) begin
            out_valid <= 1'b1;
            out_bit   <= in_bit;

            crc <= next_crc;

            if (in_last) begin
              state       <= S_APPEND;
              crc_latched <= next_crc;
              cnt         <= 8'd0;
            end
          end
        end

        // --------------------------------------------------------------------
        // APPEND: emisja bitów CRC (MSB-first) po zakończeniu danych
        // --------------------------------------------------------------------
        S_APPEND: begin
          out_valid <= 1'b1;

          // Emisja bitów CRC: p0..p(L-1) = MSB..LSB
          out_bit <= crc_latched[L_LEN-1-cnt];

          if (cnt == L_LEN-1) begin
            out_last <= 1'b1;    // ostatni bit całej ramki
            state    <= S_IDLE;  // gotowość na kolejną ramkę
          end
          cnt <= cnt + 8'd1;
        end

        default: begin
          state <= S_IDLE;
        end
      endcase
    end
  end

endmodule
