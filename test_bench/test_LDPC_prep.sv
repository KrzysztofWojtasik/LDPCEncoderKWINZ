// tb/tb_ldpc_prep.sv
`timescale 1ns/1ps

// ============================================================================
// TB: tb_LDPC_prep
//
// Cel:
// - Weryfikacja modułu LDPC_prep (stream 64-bit).
// - Wejście: TB+TB_CRC o długości B bitów (META.B) podane jako kolejne 64-bit chunk.
// - Parametr tb_len = A (META.A) czyli długość TB bez TB CRC.
// - Wyjście: codeblocki (CB) w chunkach 64-bit z sygnałami granic:
//     * out_last_chunk = koniec bieżącego CB
//     * out_last_cb    = koniec całego TB
//
// Dane z Matlaba (jeden katalog VEC_DIR_MAT):
// - <CASE>_in_bits.txt   : 1 linia bitów 0/1 (co najmniej B bitów)
// - <CASE>_out_bits.txt  : wiele linii, każda linia = golden dla jednego CB (0/1, opcjonalnie "-1")
// - <CASE>_meta.txt      : A, B, CB_COUNT w formacie KEY=VALUE
//
// Porównanie:
// - Porównujemy tylko tyle bitów w danym CB, ile jest w golden dla tego CB.
// - Dump zapisuje zawsze pełne 64 bity z każdego out_chunk (1 linia = 1 chunk).
// - Między CB w dumpie są 3 znaki nowej linii (1 po chunku + 2 puste linie).
//
// Obsługa błędów:
// - błędy są zliczane i wypisywane  do out_last_cb (albo timeout), zapisując cały strumień DUT do pliku.
// - Na końcu PASS/FAIL + podsumowanie
//
// Debug:
// - DBG_EN = 1 -> włączony
// - DBG_EN = 0 -> wyłączony
// ============================================================================

module test_LDPC_prep;

  // === KONFIG ===
  localparam string VEC_DIR_MAT    = "../vectors/LDPC_prep/Output_Matlab";
  localparam string VEC_DIR_RESULT = "../vectors/LDPC_prep/Output_DUT";
  localparam string CASE = "A59761";

  localparam int W = 64;
  localparam int TIMEOUT_CYCLES = 500000;

  // === PRZEŁĄCZNIK DEBUGGERA ===
  localparam bit DBG_EN = 1'b0;

  // Limit wydruków błędów na ekran
  localparam int MAX_ERR_PRINT = 200;

  // === SYGNAŁY DO DUT ===
  logic               clk;
  logic               rst;          // synch reset aktywny w '0'
  logic [15:0]        tb_len;       // TB length = A (bez TB CRC)

  logic               in_valid;
  logic [63:0]        in_chunk;
  logic               in_last;

  logic               out_valid;
  logic [63:0]        out_chunk;
  logic               out_last_chunk;
  logic               out_last_cb;
  logic [15:0]        out_filler_cnt;
  logic [15:0]        out_k;

  // === PLIKI ===
  string in_file;
  string golden_file;
  string meta_file;
  string out_dump_file;
  int fd_dump;

  // === BUFOR DANYCH ===
  bit tb_bits_q[$];                 // bity wejściowe (powinno być >= META_B)
  bit golden_cb[int][$];            // golden_cb[cb_idx] -> kolejka bitów dla danego CB (bez -1)

  // === META ===
  int META_A;        // TB length bez TB CRC
  int META_B;        // TB+TB_CRC length
  int META_CB_COUNT;

  // === STATYSTYKI / BŁĘDY ===
  int err_mismatch;          // bitowe niezgodności w zakresie porównania
  int err_short_cb;          // DUT zakończył CB zanim “dobił” do liczby bitów z goldena (w tym CB)
  int err_no_golden_cb;      // brak goldena dla danego cb_idx
  int err_lastcb_pos;        // out_last_cb w nieoczekiwanym miejscu względem META_CB_COUNT
  int err_extra_cb;          // DUT wygenerował więcej CB niż META_CB_COUNT (sygnalizowane granicami)
  int err_timeout;           // watchdog timeout

  int shown_err;             // ile błędów już wypisano (limit MAX_ERR_PRINT)

  // Liczniki przepływu
  int in_bits_expected;
  int out_chunks;
  int out_bits_total;
  int compared_bits_total;

  // Zegar
  initial clk = 0;
  always #5 clk = ~clk;

  // DUT
  LDPC_prep dut (
    .clk(clk),
    .rst(rst),
    .tb_len(tb_len),

    .in_valid(in_valid),
    .in_chunk(in_chunk),
    .in_last(in_last),

    .out_valid(out_valid),
    .out_chunk(out_chunk),
    .out_last_chunk(out_last_chunk),
    .out_last_cb(out_last_cb),
    .out_filler_cnt(out_filler_cnt),
    .out_k(out_k)
  );

  // ==========================================================================
  // DEBUGGER 
  // ==========================================================================
  int cyc;
  int prev_fsm1, prev_fsm2, prev_fsm3, prev_ceil_state;
  integer prev_in_valid;
  integer prev_in_valid_check;
  integer prev_in_last;
  integer prev_in_last_check;
  integer prev_out_valid;
  integer prev_out_valid_check;
  integer prev_out_last;
  integer prev_out_last_check;

  generate
    if (DBG_EN) begin : gen_dbg
      always @(posedge clk) begin
        if (!rst) begin
          cyc <= 0;
          prev_fsm1 <= -1;
          prev_fsm2 <= -1;
          prev_fsm3 <= -1;
          prev_ceil_state <= -1;
        end else begin
          cyc <= cyc + 1;

          if ((cyc % 1000) == 0) begin
            $display("cyc=%0d, t=%0t, fsm1=%0d, fsm2=%0d, fsm3=%0d, rd_addr=%0d, crc_out_valid=%0b, crc_out_last=%0b, crc.state=%0d",
              cyc, $time, dut.state_fsm1, dut.state_fsm2, dut.state_fsm3, dut.rd_addr, dut.crc_out_valid, dut.crc_out_last, dut.crc.state
            );
          end

          if (dut.state_fsm1 != prev_fsm1 || dut.state_fsm2 != prev_fsm2 || dut.state_fsm3 != prev_fsm3 || dut.u_Ceil.state != prev_ceil_state) begin
            $display("STATE CHANGE @cyc=%0d: fsm1=%0d fsm2=%0d fsm3=%0d ceil_state=%0d tb_stored=%0b params=%0b tb_ready=%0b",
              cyc, dut.state_fsm1, dut.state_fsm2, dut.state_fsm3, dut.u_Ceil.state,
              dut.tb_stored, dut.params_calc, dut.tb_ready
            );
            prev_fsm1 <= dut.state_fsm1;
            prev_fsm2 <= dut.state_fsm2;
            prev_fsm3 <= dut.state_fsm3;
            prev_ceil_state <= dut.u_Ceil.state;
          end
        end
      end
    end else begin
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
			 
			 if (prev_out_last != dut.out_last_chunk && dut.out_last_cb == 1 && prev_out_last_check == 0) begin
            $display("tout_last = %0d ",
              cyc,
            );
            prev_out_last_check <= 1;
          end
        end
      end
	 end
  endgenerate

  
  
  // === POMOCNICZE ===
  function automatic bit is01(byte c);
    return (c=="0" || c=="1");
  endfunction

  function automatic string trim_eol(input string s);
    while (s.len() > 0) begin
      byte c = s[s.len()-1];
      if (c == "\n" || c == "\r") s = s.substr(0, s.len()-2);
      else break;
    end
    return s;
  endfunction

  task automatic read_tb_bits_first_line(input string path, output bit q[$]);
    int fd;
    string line;
    q = {};
    fd = $fopen(path, "r");
    if (fd == 0) begin
      $display("ERROR: Nie moge otworzyc TB: %s", path);
      $finish;
    end

    if ($fgets(line, fd) == 0) begin
      $display("ERROR: TB pusty: %s", path);
      $finish;
    end
    line = trim_eol(line);
    $fclose(fd);

    for (int i = 0; i < line.len(); i++) begin
      byte c = line[i];
      if (is01(c)) q.push_back( (c=="1") );
    end
  endtask

  // Golden: linie = CB; token "-1" ignorowany.
  task automatic read_golden_codeblocks_lines(input string path);
    int fd;
    string line;
    int cb = 0;

    fd = $fopen(path, "r");
    if (fd == 0) begin
      $display("ERROR: Nie moge otworzyc golden CB: %s", path);
      $finish;
    end

    while ($fgets(line, fd)) begin
      bit tmp[$];
      tmp = {};
      line = trim_eol(line);

      for (int i = 0; i < line.len(); i++) begin
        byte c = line[i];

        if (c == "-" && (i+1) < line.len() && line[i+1] == "1") begin
          i++;
          continue;
        end

        if (is01(c)) tmp.push_back( (c=="1") );
      end

      if (tmp.size() > 0) begin
        golden_cb[cb] = tmp;
        cb++;
      end
    end

    $fclose(fd);

    if (cb == 0) begin
      $display("ERROR: Golden CB pusty: %s", path);
      $finish;
    end
  endtask

  task automatic read_meta(input string path);
    int fd;
    string line;
    string key;
    string val_s;
    int eq_pos;
    int v;

    META_A = -1;
    META_B = -1;
    META_CB_COUNT = -1;

    fd = $fopen(path, "r");
    if (fd == 0) begin
      $display("ERROR: Nie moge otworzyc META: %s", path);
      $finish;
    end

    while ($fgets(line, fd)) begin
      line = trim_eol(line);

      eq_pos = -1;
      for (int i = 0; i < line.len(); i++) begin
        if (line[i] == "=") begin
          eq_pos = i;
          break;
        end
      end
      if (eq_pos <= 0 || eq_pos >= line.len()-1) continue;

      key   = line.substr(0, eq_pos-1);
      val_s = line.substr(eq_pos+1, line.len()-1);

      if (key == "A" || key == "B" || key == "CB_COUNT") begin
        if ($sscanf(val_s, "%d", v) != 1) begin
          $display("ERROR: META: nie moge sparsowac liczby z: %s", line);
          $finish;
        end

        if      (key == "A")         META_A = v;
        else if (key == "B")         META_B = v;
        else if (key == "CB_COUNT")  META_CB_COUNT = v;
      end
    end

    $fclose(fd);

    if (META_A < 0 || META_B < 0 || META_CB_COUNT < 0) begin
      $display("ERROR: META niekompletna w %s (A=%0d B=%0d CB_COUNT=%0d)",
               path, META_A, META_B, META_CB_COUNT);
      $finish;
    end
  endtask

  // === GŁÓWNY TEST ===
  initial begin
    // init sygnałów
    rst      = 0;
    tb_len   = '0;
    in_valid = 0;
    in_chunk = '0;
    in_last  = 0;

    // init liczników
    err_mismatch     = 0;
    err_short_cb     = 0;
    err_no_golden_cb = 0;
    err_lastcb_pos   = 0;
    err_extra_cb     = 0;
    err_timeout      = 0;
    shown_err        = 0;

    out_chunks          = 0;
    out_bits_total      = 0;
    compared_bits_total = 0;

    // ścieżki plików (Matlab w jednym katalogu)
    in_file       = {VEC_DIR_MAT,  "/", CASE, "_in_bits",  ".txt"};
    golden_file   = {VEC_DIR_MAT,  "/", CASE, "_out_bits", ".txt"};
    meta_file     = {VEC_DIR_MAT,  "/", CASE, "_meta",     ".txt"};
    out_dump_file = {VEC_DIR_RESULT,"/", CASE, "_dut_out_cb", ".txt"};

    // reset (aktywny w 0)
    repeat (5) @(posedge clk);
    rst <= 1;
    @(posedge clk);

    // META
    read_meta(meta_file);
    $display("META: A=%0d B=%0d CB_COUNT=%0d", META_A, META_B, META_CB_COUNT);

    // NA WEJŚCIE TB_LEN IDZIE A (bez TB CRC)
    tb_len <= META_A[15:0];

    // wejście + golden
    read_tb_bits_first_line(in_file, tb_bits_q);
    read_golden_codeblocks_lines(golden_file);

    // wejściowy TB musi mieć >= B bitów (inaczej driver się wywali na indeksach)
    if (tb_bits_q.size() < META_B) begin
      $display("ERROR: TB za krotki: ma %0d bitow, a META.B=%0d", tb_bits_q.size(), META_B);
      $finish;
    end

    // jeżeli golden ma mniej CB niż meta, pozwalamy jechać (dump będzie), ale nie będzie pełnego compare
    if (golden_cb.num() < META_CB_COUNT) begin
      $display("UWAGA: Golden ma za malo codeblockow: %0d, a META.CB_COUNT=%0d (compare dla brakujacych CB bedzie pominiety).",
               golden_cb.num(), META_CB_COUNT);
    end

    // otwórz dump DUT
    fd_dump = $fopen(out_dump_file, "w");
    if (fd_dump == 0) begin
      $display("ERROR: Nie moge otworzyc do zapisu: %s", out_dump_file);
      $finish;
    end

    in_bits_expected = META_B;

    fork
      // =========================
      // DRIVER (TB -> DUT)
      // =========================
      begin : drv
        automatic int di = 0;
        automatic int send_bits = META_B;      // wysyłamy B (TB+TB_CRC)
        automatic logic [W-1:0] word;

        while (di < send_bits) begin
          for (int b = 0; b < W; b++) begin
            if ((di + b) < send_bits) word[W-1-b] = tb_bits_q[di + b];
            else                      word[W-1-b] = 1'b0;
          end

          @(posedge clk);
          in_valid <= 1'b1;
          in_chunk <= word;
          in_last  <= ((di + W) >= send_bits);
          di      += W;
        end

        @(posedge clk);
        in_valid <= 1'b0;
        in_last  <= 1'b0;
        in_chunk <= '0;
      end

      // =========================
      // CHECKER (DUT -> compare + dump)
      // =========================
      begin : chk
        automatic int cb_idx = 0;
        automatic int bit_in_cb = 0;
        automatic bit ob;
        automatic int compare_limit;

        forever begin
          @(posedge clk);

          if (out_valid) begin
            out_chunks++;
            out_bits_total += W;

            // jeżeli brak goldena dla tego CB -> nie porównujemy, ale dumpujemy dalej
            if (golden_cb.exists(cb_idx)) begin
              compare_limit = golden_cb[cb_idx].size();
            end else begin
              compare_limit = 0;
              err_no_golden_cb++;
              if (shown_err < MAX_ERR_PRINT) begin
                $display("WARN: Brak golden dla cb_idx=%0d (porownanie pominiete dla tego CB).", cb_idx);
                shown_err++;
              end
            end

            // Dump pełnego chunku + porównanie tylko w zakresie compare_limit
            for (int b = 0; b < W; b++) begin
              ob = out_chunk[W-1-b];

              // dump zawsze
              $fwrite(fd_dump, "%0d", ob);

              // compare tylko do compare_limit
              if (bit_in_cb < compare_limit) begin
                compared_bits_total++;
                if (ob !== golden_cb[cb_idx][bit_in_cb]) begin
                  err_mismatch++;
                  if (shown_err < MAX_ERR_PRINT) begin
                    $display("MISMATCH @%0t: CB=%0d bit=%0d DUT=%0b GOLD=%0b",
                             $time, cb_idx, bit_in_cb, ob, golden_cb[cb_idx][bit_in_cb]);
                    shown_err++;
                  end
                end
              end

              bit_in_cb++;
            end

            // nowa linia po KAŻDYM chunku
            $fwrite(fd_dump, "\n");

            // ===== granica codeblocka =====
            if (out_last_chunk) begin
              // jeżeli mamy golden, to wymagamy co najmniej compare_limit bitów “w CB”
              if (compare_limit > 0 && bit_in_cb < compare_limit) begin
                err_short_cb++;
                if (shown_err < MAX_ERR_PRINT) begin
                  $display("SHORT_CB: CB=%0d DUT_bits=%0d required=%0d", cb_idx, bit_in_cb, compare_limit);
                  shown_err++;
                end
              end

              // między CB: 3 newline (1 już po chunku, dodajemy 2 puste)
              $fwrite(fd_dump, "\n\n");

              // obsługa końca TB
              if (out_last_cb) begin
                // out_last_cb powinien wystąpić na ostatnim CB wg META
                if (cb_idx != (META_CB_COUNT - 1)) begin
                  err_lastcb_pos++;
                  if (shown_err < MAX_ERR_PRINT) begin
                    $display("BAD_LAST_CB: out_last_cb na CB=%0d, a oczekiwany ostatni=%0d (CB_COUNT=%0d)",
                             cb_idx, META_CB_COUNT-1, META_CB_COUNT);
                    shown_err++;
                  end
                end

                void'($fclose(fd_dump));
                $display("Zapisano: %s", out_dump_file);

                // PODSUMOWANIE
                if ((err_mismatch + err_short_cb + err_no_golden_cb + err_lastcb_pos + err_extra_cb + err_timeout) == 0) begin
                  $display("PASS: %s", CASE);
                end else begin
                  $display("FAIL: %s", CASE);
                end

                $display("  in_bits_expected   = %0d (META.B)", in_bits_expected);
                $display("  out_chunks         = %0d", out_chunks);
                $display("  out_bits_total     = %0d", out_bits_total);
                $display("  compared_bits      = %0d", compared_bits_total);
                $display("  cb_count_expected  = %0d (META.CB_COUNT)", META_CB_COUNT);
                $display("  errors:");
                $display("    mismatch         = %0d", err_mismatch);
                $display("    short_cb         = %0d", err_short_cb);
                $display("    no_golden_cb     = %0d", err_no_golden_cb);
                $display("    bad_last_cb_pos  = %0d", err_lastcb_pos);
                $display("    extra_cb         = %0d", err_extra_cb);
                $display("    timeout          = %0d", err_timeout);

                if (shown_err >= MAX_ERR_PRINT)
                  $display("  INFO: obcieto wypisywanie bledow (MAX_ERR_PRINT=%0d).", MAX_ERR_PRINT);

                $finish;
              end

              // kolejny CB
              cb_idx++;
              bit_in_cb = 0;

              if (cb_idx > META_CB_COUNT) begin
                err_extra_cb++;
                if (shown_err < MAX_ERR_PRINT) begin
                  $display("EXTRA_CB: DUT wygenerowal cb_idx=%0d > META_CB_COUNT=%0d", cb_idx, META_CB_COUNT);
                  shown_err++;
                end
              end
            end
          end
        end
      end

      // =========================
      // TIMEOUT WATCHDOG
      // =========================
      begin : wdog
        repeat (TIMEOUT_CYCLES) @(posedge clk);

        err_timeout++;
        void'($fclose(fd_dump));

        $display("FAIL: TIMEOUT (%0d cykli) - brak out_last_cb", TIMEOUT_CYCLES);
        $display("Zapisano (niepelny): %s", out_dump_file);
        $display("  in_bits_expected   = %0d (META.B)", in_bits_expected);
        $display("  out_chunks         = %0d", out_chunks);
        $display("  out_bits_total     = %0d", out_bits_total);
        $display("  compared_bits      = %0d", compared_bits_total);
        $display("  errors:");
        $display("    mismatch         = %0d", err_mismatch);
        $display("    short_cb         = %0d", err_short_cb);
        $display("    no_golden_cb     = %0d", err_no_golden_cb);
        $display("    bad_last_cb_pos  = %0d", err_lastcb_pos);
        $display("    extra_cb         = %0d", err_extra_cb);
        $display("    timeout          = %0d", err_timeout);

        $finish;
      end

    join_none
  end

endmodule
