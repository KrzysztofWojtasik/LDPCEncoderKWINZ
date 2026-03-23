// ============================================================================
// PDSCHTx (top-level)
// Tor przygotowania i kodowania danych dla kanału PDSCH (5G NR) w postaci
// strumienia równoległego 64-bit.
//
// Funkcja modułu (kolejność przetwarzania):
//  1) crc_tb_parallel  – dopisanie CRC do Transport Block (TB)
//  2) LDPC_prep        – segmentacja TB na bloki kodowe (CB), dopisanie CRC CB,
//                        wyznaczenie parametrów LDPC (k, Zc, BG) i liczby bitów wypełnienia
//  3) LDPC_encode      – obliczenie bloków parzystości LDPC i emisja zakodowanego strumienia
//
// Wejście – wymagania czasowe i semantyka:
//  - in_valid: kwalifikuje in_chunk w danym takcie; dane muszą być stabilne w cyklu, w którym in_valid=1.
//  - in_chunk[63:0]: słowo danych MSB-first.
//  - in_last: impuls oznaczający ostatnie słowo danych TB.
//  - tb_len: długość TB w bitach (bez CRC). Wartość musi odpowiadać aktualnej ramce i być
//            stabilna co najmniej od pierwszego słowa do końca TB.
//
// Wyjście – semantyka:
//  - out_valid: kwalifikuje out_chunk w danym takcie.
//  - out_chunk[63:0]: słowo zakodowanych danych (część systematyczna + bloki parzystości LDPC).
//  - out_last: impuls na ostatnim słowie całego TB po kodowaniu LDPC.
//  - out_pad_left: liczba bitów do pominięcia od strony MSB 
//  - out_pad_right: liczba bitów do pominięcia od strony LSB 
//
// Uwagi integracyjne:
//  - Moduł obsługuję tylko jeden TB, później trzeba użyć sygnału rst
//  - Opóźnienie od wejścia do pierwszego wyjścia zależy od długości TB i liczby CB.
//  - Pliki ROM (*.memh) używane przez LDPC_encode muszą być dostępne w środowisku syntezy/symulacji.
// ============================================================================

module PDSCHTx(
    input  wire        clk,
    input  wire        rst,

    // Wejście: strumień TB (64-bit) + długość TB
    input  wire [63:0] in_chunk,
    input  wire        in_valid,
    input  wire        in_last,
    input  wire [15:0] tb_len,

    // Wyjście: strumień po CRC(TB)+segmentacji+CRC(CB)+LDPC
    output wire [63:0] out_chunk,
    output wire        out_valid,
    output wire        out_last,
    output wire [5:0]  out_pad_left,
    output wire [5:0]  out_pad_right
);

  // ==========================================================================
  // 1) DOPISANIE CRC DO TB
  // - Przepuszcza słowa TB na wyjście 1:1
  // - Dopisuje CRC16 lub CRC24A zależnie od tb_len
  // - out_last_crc oznacza ostatnie słowo po dopisaniu CRC TB
  // ==========================================================================

  wire [63:0]  out_chunk_crc;
  wire         out_valid_crc;
  wire         out_last_crc;
  wire [15:0]  out_tb_len_crc;

  crc_tb_parallel crc (
    .clk        (clk),
    .rst        (rst),
    .in_tb_len  (tb_len),
    .in_valid   (in_valid),
    .in_chunk   (in_chunk),
    .in_last    (in_last),
    .out_valid  (out_valid_crc),
    .out_chunk  (out_chunk_crc),
    .out_last   (out_last_crc),
    .out_tb_len (out_tb_len_crc)
  );

  // ==========================================================================
  // 2) PRZYGOTOWANIE DO LDPC (SEGMENTACJA TB -> CB)
  // - Wyznacza parametry LDPC: k, Zc, BG oraz liczbę bitów wypełnienia
  // - Dzieli TB na CB, dopisuje CRC CB i emituje strumień CB (64-bit)
  // - out_last_chunk_prep: koniec danych danego CB 
  // - out_last_cb_prep:    znacznik, że bieżący CB jest ostatnim CB w TB
  // ==========================================================================

  wire [63:0]  out_chunk_prep;
  wire         out_valid_prep;
  wire         out_last_chunk_prep;
  wire         out_last_cb_prep;
  wire [15:0]  out_filler_cnt_prep;
  wire [15:0]  out_k_prep;
  wire [8:0]   out_zc_prep;
  wire         out_bg_prep;

  LDPC_prep prep (
    .clk             (clk),
    .rst             (rst),

    .tb_len          (out_tb_len_crc),

    .in_valid        (out_valid_crc),
    .in_chunk        (out_chunk_crc),
    .in_last         (out_last_crc),

    .out_valid       (out_valid_prep),
    .out_chunk       (out_chunk_prep),
    .out_last_chunk  (out_last_chunk_prep),
    .out_last_cb     (out_last_cb_prep),
    .out_filler_cnt  (out_filler_cnt_prep),
    .out_k           (out_k_prep),
    .out_zc          (out_zc_prep),
    .out_bg          (out_bg_prep)
  );

  // ==========================================================================
  // 3) KODOWANIE LDPC
  // - Oblicza bloki parzystości na podstawie BG i Zc
  // - Emisja: część systematyczna + parzystość dla każdego CB
  // - out_last: impuls końca całego TB po LDPC
  // - out_pad_left/out_pad_right: informacja o bitach do pominięcia na krańcach
  // ==========================================================================

  LDPC_encode encode (
    .clk           (clk),
    .rst           (rst),
    .in_k          (out_k_prep),
    .in_filler_cnt (out_filler_cnt_prep),
    .in_zc         (out_zc_prep),
    .in_bg         (out_bg_prep),
    .in_valid      (out_valid_prep),
    .in_chunk      (out_chunk_prep),
    .in_last       (out_last_chunk_prep),
    .in_last_cb    (out_last_cb_prep),
    .out_valid     (out_valid),
    .out_chunk     (out_chunk),
    .out_last      (out_last),
    .out_pad_left  (out_pad_left),
    .out_pad_right (out_pad_right)
  );

endmodule
