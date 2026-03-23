`timescale 1ns/1ps

// ============================================================================
// test_crc_serial
// Testbench weryfikujący moduł crc_serial (dopisywanie CRC do strumienia 1-bit).
//
// NALEŻY USTAWIĆ ODPOWIEDNI PARAMTER L (DŁUGOŚĆ CRC)
//
// Zasada działania testu:
// 1) Wczytuje z plików wektory z Matlaba:
//    - wejście:  CASE_in_bits.txt   (bity danych 0/1)
//    - golden:   CASE_out_bits.txt  (payload + bity CRC, MSB-first)
//    Uwaga: obecna implementacja czyta WYŁĄCZNIE PIERWSZĄ linię pliku i wybiera
//    tylko znaki '0'/'1' (ignoruje resztę).
//
// 2) Generuje zegar i reset synchroniczny (rst aktywny w stanie niskim).
//
// 3) Driver podaje dane do DUT: 1 bit na cykl przy in_valid=1.
//    in_last jest ustawiany na ostatnim bicie danych.
//
// 4) Checker monitoruje out_valid i porównuje każdy bit wyjściowy z goldenem.
//    - Nie przerywa symulacji przy błędach: wypisuje FAIL (diagnostyka) i jedzie dalej.
//    - Zapisuje:
//      * cały strumień DUT (payload+CRC) do *_dut_stream_out.txt
//      * same bity CRC do *_dut_crc_out.txt (od momentu gi >= data_q.size()).
//    Zakończenie testu następuje dopiero po out_last (ostatni bit CRC).
//
// 5) Na końcu wypisuje PASS/FAIL, jeśli FAIL to ile i jakich błędóW.
// ============================================================================

module test_crc_serial;

  // === KONFIG ===
  // Dwa katalogi: MATLAB (wejście + golden + meta) oraz katalog wyników DUT.
  localparam string VEC_DIR_MAT = "../vectors/TB_crc/Output_Matlab"; // Wektory wyjściowe do porównania
  localparam string VEC_DIR_RESULT = "../vectors/TB_crc/Output_DUT_serial"; //Miejsce do zapisu wektorów wyjściowych 
  localparam string CASE     = "A59761";
  localparam int		L		  = 24;

  // === SYGNAŁY DO DUT ===
  // Strumień 1-bit/cykl:
  // - in_valid: ważność bitu
  // - in_last : znacznik ostatniego bitu danych (payload)
  // Wyjście:
  // - out_valid: ważność bitu
  // - out_last : ostatni bit całej ramki (ostatni bit CRC)
  logic clk;
  logic rst;
  logic in_valid, in_bit, in_last;
  logic out_valid, out_bit, out_last;

  // === ZMIENNE POMOCNICZE ===
  // Ścieżki plików są składane z folderów + nazwy CASE.
  string in_file;
  string  golden_file;
  string out_stream_path;
  string out_crc_path;

  // Kolejki bitów (dynamiczne): wejście oraz golden.
  bit data_q[$];
  bit golden_q[$];

  // Deskryptory plików do zapisu:
  // - fd_stream: pełny strumień wyjściowy (payload+CRC)
  // - fd_crc   : wyłącznie bity CRC (wycięte wg indeksu gi >= data_q.size()).
  int fd_stream;         // deskryptor pliku: strumień DUT
  int fd_crc;            // deskryptor pliku: same bity CRC
  
  // === LICZNIKI BŁĘDÓW ===
  int err_mismatch;      // niezgodność bitów DUT vs GOLDEN
  int err_extra_bits;    // DUT wygenerował więcej bitów niż GOLDEN
  int err_missing_bits;  // DUT wygenerował mniej bitów niż GOLDEN

  // Zegar
  // Okres 10 ns: #5 półokres -> 100 MHz w symulacji.
  initial clk = 0;
  always #5 clk = ~clk;

  // DUT
  // Uwaga: ten TB używa tylko podstawowego interfejsu crc_serial (bez custom_crc).
  crc_serial #(.L(L)) dut (
    .clk, .rst, .in_valid, .in_bit, .in_last,
    .out_valid, .out_bit, .out_last
  );

  
  integer cyc;
  integer prev_in_valid;
  integer prev_in_valid_check;
  integer prev_in_last;
  integer prev_in_last_check;
  integer prev_out_valid;
  integer prev_out_valid_check;
  integer prev_out_last;
  integer prev_out_last_check;
  
  generate
      always @(posedge clk) begin
        // ====== DEBUGGER: ======
        if (!rst) begin
          cyc <= 0;
          prev_in_valid <= 0;
          prev_in_valid_check <= 0;
          prev_in_last <= 0;
          prev_in_last_check <= 0;
			 prev_out_valid <= 0;
			 prev_out_valid_check <= 0;
			 prev_out_last <= 0;
			 prev_out_last_check <= 0;

        end else begin
          cyc <= cyc + 1;
          // i/lub drukuj gdy zmieni się którykolwiek stan
          if (prev_in_valid != dut.in_valid && prev_in_valid_check == 0) begin
            $display("tin_first = %0d ",
              cyc,
            );
            prev_in_valid_check <= 1;
          end
			 if (prev_in_last != dut.in_last && prev_in_last_check == 0) begin
            $display("tin_last = %0d ",
              cyc,
            );
            prev_in_last_check <= 1;
          end
			 if (prev_out_valid != dut.out_valid && prev_out_valid_check == 0) begin
            $display("tout_fisrt = %0d ",
              cyc,
            );
            prev_out_valid_check <= 1;
          end
			 
			 if (prev_out_last != dut.out_last && prev_out_last_check == 0) begin
            $display("tout_last = %0d ",
              cyc,
            );
            prev_out_last_check <= 1;
          end
        end
      end
  endgenerate
  // Funkcja: czy znak to '0'/'1'
  // Służy do odfiltrowania danych z tekstowej linii pliku.
  function automatic bit is01(byte c);
    return (c=="0" || c=="1");
  endfunction

  // Wczytaj pierwszą linię pliku do kolejki bitów
  // Ograniczenie: czytana jest tylko 1 linia (pierwsza).
  // Format: dowolny tekst, istotne są tylko znaki '0' i '1'.
  task automatic read_bits_from_file(input string path, output bit q[$]);
    int fd; string line;
    q = {};
    fd = $fopen(path, "r");
    if (fd == 0) $fatal(1, "Nie moge otworzyc: %s", path);
    void'($fgets(line, fd));
    $fclose(fd);
    for (int i = 0; i < line.len(); i++) begin
      byte c = line[i];
      if (is01(c)) q.push_back( (c=="1") );
    end
  endtask

  // === STYMULACJA + CHECK ===
  initial begin
    // Reset
    // rst=0: aktywny reset (synchroniczny, aktywny niski)
    // po kilku cyklach przejście do rst=1 -> praca normalna DUT
    rst = 0;
    in_valid = 0; in_bit = 0; in_last = 0;
	 
	 // init liczników
    err_mismatch     = 0;
    err_extra_bits   = 0;
    err_missing_bits = 0;
	 
    repeat (5) @(posedge clk);
    rst = 1;
    @(posedge clk);

    // Ścieżki plików (przypisania, nie deklaracje!)
    // Konwencja nazw plików:
    //  - wejście  : <CASE>_in_bits.txt
    //  - golden   : <CASE>_out_bits.txt
    //  - wyniki   : <CASE>_dut_stream_out.txt  oraz  <CASE>_dut_crc_out.txt
    in_file  = {VEC_DIR_MAT,  "/", CASE, "_in_bits", ".txt"};
    golden_file = {VEC_DIR_MAT, "/", CASE, "_out_bits", ".txt"};
	 
	 out_stream_path = {VEC_DIR_RESULT,"/", CASE, "_dut_stream_out", ".txt"};
    out_crc_path    = {VEC_DIR_RESULT,"/", CASE, "_dut_crc_out", ".txt"};
	 
	 

    // Wczytaj dane i złoty wynik
    // data_q    : payload (bity wejściowe do DUT)
    // golden_q  : pełny oczekiwany strumień (payload + CRC)
    read_bits_from_file(in_file,  data_q);
    read_bits_from_file( golden_file, golden_q);
    if (data_q.size()   == 0) $fatal(1, "Plik wejsciowy pusty: %s", in_file);
    if (golden_q.size() == 0) $fatal(1, "Plik wyjsciowy pusty: %s",  golden_file);

    $display("Test: %s | data=%0d | golden=%0d", CASE, data_q.size(), golden_q.size());

	 // Otwórz pliki logów
	 // fd_stream: zawsze zapisuje każdy bit gdy out_valid
	 // fd_crc   : zapisuje tylko część CRC wyciętą na podstawie długości wejścia
    fd_stream = $fopen(out_stream_path, "w");
    if (fd_stream == 0) $fatal(1, "Nie moge otworzyc do zapisu: %s", out_stream_path);
    fd_crc    = $fopen(out_crc_path, "w");
    if (fd_crc == 0)    $fatal(1, "Nie moge otworzyc do zapisu: %s", out_crc_path);


    fork
      // DRIVER
      // Podaje 1 bit/cykl przy in_valid=1.
      // in_last ustawiany na ostatnim bicie payload (koniec danych).
      begin : drv
		int di;
		di = 0;
		
		
        while (di < data_q.size()) begin
          @(posedge clk);
          in_valid <= 1;
          in_bit   <= data_q[di];
          in_last  <= (di == data_q.size()-1);
          di++;
        end
        @(posedge clk);
        in_valid <= 0;
        in_last  <= 0;
      end

      // CHECKER 
      // gi = indeks bitu strumienia wyjściowego (liczony tylko przy out_valid).
      // - zapisuje bity do plików
      // - porównuje bit do bitu z golden_q
      // - wypisuje komunikaty FAIL, ale nie kończy testu
      // - kończy test dopiero na out_last
      begin : chk_blk
        int gi;
        gi = 0;
		  
        forever begin
          @(posedge clk);
          if (out_valid) begin
				 // Log całego strumienia wyjściowego DUT
				 $fwrite(fd_stream, "%0d", out_bit);

				 // Wycięcie CRC: po przejściu długości danych wejściowych (payload)
				 // zakładamy, że DUT wypuszcza payload 1:1, a potem dopisuje CRC.
				 if (gi >= data_q.size()) begin
              $fwrite(fd_crc, "%0d", out_bit);
             end

            // porównanie z goldenem (non-fatal)
            if (gi >= golden_q.size()) begin
              err_extra_bits++;
              $display("FAIL: DUT wygenerowal za duzo bitow (idx=%0d)", gi);
            end else begin
              if (out_bit !== golden_q[gi]) begin
                err_mismatch++;
                $display("FAIL @%0t: idx=%0d DUT=%0b GOLD=%0b", $time, gi, out_bit, golden_q[gi]);
              end
            end

            gi++;

            // Koniec ramki: out_last powinno wystąpić na ostatnim bicie CRC
             if (out_last) begin
              void'($fclose(fd_stream));
              void'($fclose(fd_crc));
              $display("Zapisano: %s", out_stream_path);
              $display("Zapisano: %s", out_crc_path);

              // DUT krótszy niż golden -> brakujące bity liczymy jako błąd
              if (gi < golden_q.size()) begin
                err_missing_bits = golden_q.size() - gi;
              end

              // PODSUMOWANIE
              if ((err_mismatch + err_extra_bits + err_missing_bits) == 0) begin
                $display("PASS: %s (sprawdzono %0d bitow)", CASE, gi);
              end else begin
                $display("FAIL: %s", CASE);
                $display("  mismatch_bits = %0d", err_mismatch);
                $display("  extra_bits    = %0d", err_extra_bits);
                $display("  missing_bits  = %0d", err_missing_bits);
                $display("  checked_bits  = %0d (dut) vs %0d (golden)", gi, golden_q.size());
              end

              $finish;
            end
          end
        end
      end
    join_none
  end

endmodule
