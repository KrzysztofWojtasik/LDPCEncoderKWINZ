// ============================================================================
// Ceil
// Moduł synchroniczny realizujący dzielenie całkowite metodą kolejnych odejmowań
// oraz wyznaczający iloraz typu "ceil" (zaokrąglenie w górę) wraz z resztą.
//
// Interfejs sterowania:
// - enable pełni rolę sygnału kroku/akceptacji: operacja wykonuje się tylko, gdy enable=1.
// - result_valid sygnalizuje dostępność wyniku (iloraz) w stanie RESULT.
//
// Uwagi funkcjonalne:
// - Obliczenia wykonywane są iteracyjnie (1 odejmowanie na cykl), co daje zmienną latencję.
// ============================================================================

module Ceil (
  input  wire [15:0]  num,           // licznik (numerator)
  input  wire [15:0]  den,           // mianownik (denominator)
  input  wire         rst,           // reset synchroniczny, aktywny niski (reset gdy rst==0)
  input  wire         enable,        // zezwolenie na rozpoczęcie i kolejne kroki obliczeń
  input  wire         clk,

  output reg  [15:0]  ceil,          // iloraz (zaokrąglenie w górę)
  output reg          result_valid,  // ważność wyjścia ceil
  output reg  [15:0]  reminder       // reszta (remainder)
);

  // ==========================================================================
  // Stany FSM:
  // S_IDLE   - oczekiwanie na start (enable=1) i zatrzaśnięcie danych wejściowych
  // S_CALC   - iteracyjne odejmowanie den od num_r, inkrementacja licznika ilorazu
  // S_RESULT - wystawienie wyniku (ceil) i result_valid, utrzymanie do czasu enable=0
  // ==========================================================================
  localparam [1:0] S_IDLE   = 2'd0;
  localparam [1:0] S_CALC   = 2'd1;
  localparam [1:0] S_RESULT = 2'd2;

  reg [15:0] num_r;     // rejestr roboczy licznika (wartość pozostała po odejmowaniach)
  reg [15:0] den_r;     // rejestr mianownika
  reg [15:0] result;    // licznik odejmowań (iloraz w ujęciu iteracyjnym)
  reg [1:0]  state;     // stan automatu

  // ==========================================================================
  // Sekwencja synchroniczna:
  // - reset czyści rejestry i przechodzi do IDLE
  // - domyślnie w każdym cyklu wygaszane są sygnały wyjściowe (brak utrzymania)
  // - enable steruje wykonywaniem kroków w stanach CALC/RESULT
  // ==========================================================================
  always @(posedge clk) begin
    if (~rst) begin
      state        <= S_IDLE;
      ceil         <= 16'b0;
      result       <= 16'b0;
      result_valid <= 1'b0;
      num_r        <= 16'b0;
      den_r        <= 16'b0;
    end else begin
      // Domyślnie: brak ważnego wyniku w danym cyklu
      ceil         <= 1'b0;
      result_valid <= 1'b0;

      case (state)

        // --------------------------------------------------------------------
        // IDLE: start transakcji (zatrzaśnięcie danych)
        // --------------------------------------------------------------------
        S_IDLE: begin
          if (enable) begin
            num_r  <= num;     // przejęcie wejść do rejestrów roboczych
            den_r  <= den;
            result <= 1'b0;    // zerowanie ilorazu
            state  <= S_CALC;  // przejście do iteracyjnych obliczeń
          end
        end

        // --------------------------------------------------------------------
        // CALC: iteracja odejmowania i zliczania ilorazu
        // - w każdym cyklu: num_r := num_r - den_r, result := result + 1
        // - zakończenie: gdy num_r <= den_r -> przejście do RESULT i wyznaczenie reszty
        // --------------------------------------------------------------------
        S_CALC: begin
          if (enable) begin
            num_r  <= num_r - den_r;
            result <= result + 1'b1;

            if (num_r <= den_r) begin
              state    <= S_RESULT;
              reminder <= (num_r == den_r) ? 16'b0 : num_r;
            end
          end
        end

        // --------------------------------------------------------------------
        // RESULT: wystawienie wyniku
        // - przy enable=1 podawany jest ceil=result oraz result_valid=1
        // - przy enable=0 moduł wraca do IDLE (gotowość na kolejne dane)
        // --------------------------------------------------------------------
        S_RESULT: begin
          if (enable) begin
            result_valid <= 1'b1;
            ceil         <= result;
          end else begin
            state <= S_IDLE;
          end
        end

        default: begin
          state <= S_IDLE;
        end

      endcase
    end
  end

endmodule
