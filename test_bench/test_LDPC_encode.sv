
`timescale 1ns/1ps

// ============================================================================
// TB: test_LDPC_encode
//
// Cel:
// - Weryfikacja modułu LDPC_encode (kodowanie LDPC w strumieniu równoległym 64-bit).
//
// Dane testowe (Matlab) – JEDEN katalog VEC_DIR_MAT:
// - <CASE>_in_bits.txt   : wejście jako linie CB; w każdej linii bity 0/1 do pierwszego "-1"
// - <CASE>_out_bits.txt  : golden jako linie CB; w każdej linii bity 0/1, tokeny "-1" ignorowane
// - <CASE>_meta.txt      : parametry (CB_COUNT, K, FILLER_CNT, ZC, BG)
//
// Przebieg:
// - Driver podaje do DUT kolejne CB z pliku wejściowego (chunk=64b, MSB-first).
//   in_last    = 1 na ostatnim słowie danego CB,
//   in_last_cb = 1 na ostatnim słowie ostatniego CB całego TB.
// - Monitor zbiera z DUT tylko WAŻNE bity z out_chunk, wycinając padding według:
//   out_pad_left  (nieważne bity od MSB),
//   out_pad_right (nieważne bity od LSB).
//   Następnie porównuje cały strumień DUT z konkatenacją goldenów.
//
// Obsługa błędów:
// - Niezgodności nie przerywają symulacji. Błędy są zliczane i raportowane.
// - Test kończy się na out_last (z DUT) albo na TIMEOUT.
// ============================================================================

module test_LDPC_encode;

  // === KONFIG ===
  // Matlab: in/golden/meta w jednym miejscu
  localparam string VEC_DIR_MAT    = "../vectors/LDPC_encode/Output_Matlab";
  localparam string VEC_DIR_RESULT = "../vectors/LDPC_encode/Output_DUT";

  localparam string CASE = "A59761";

  localparam int W = 64;
  localparam int TIMEOUT_CYCLES = 2_000_000;
  localparam int MAX_ERR_PRINT_DEFAULT = 1000;

  // === PRZEŁĄCZNIK DEBUGGERA ===
  localparam bit DBG_EN = 1'b0;

  logic        clk;
  logic        rst;

  logic [15:0] in_k;
  logic [15:0] in_filler_cnt;
  logic [8:0]  in_zc;
  logic        in_bg;

  logic        in_valid;
  logic [63:0] in_chunk;
  logic        in_last;
  logic        in_last_cb;

  logic        out_valid;
  logic [63:0] out_chunk;
  logic        out_last;

  // out_pad_left  - liczba nieważnych bitów od MSB (lewej)
  // out_pad_right - liczba nieważnych bitów od LSB (prawej)
  logic [5:0]  out_pad_left;
  logic [5:0]  out_pad_right;

  LDPC_encode dut (
    .clk(clk),
    .rst(rst),
    .in_k(in_k),
    .in_filler_cnt(in_filler_cnt),
    .in_zc(in_zc),
    .in_bg(in_bg),
    .in_valid(in_valid),
    .in_chunk(in_chunk),
    .in_last(in_last),
    .in_last_cb(in_last_cb),
    .out_valid(out_valid),
    .out_chunk(out_chunk),
    .out_last(out_last),
    .out_pad_left(out_pad_left),
    .out_pad_right(out_pad_right)
  );

  // === PLIKI ===
  string in_file;
  string golden_file;
  string meta_file;
  string out_dump_file;
  int    fd_dump;

  // === BUFOR DANYCH ===
  bit tb_cb[int][$];       // wejście: CB po CB
  bit golden_cb[int][$];   // golden: CB po CB (bez -1)

  // === META ===
  int META_CB_COUNT = 0;
  int META_K        = -1;
  int META_F        = -1;
  int META_ZC       = -1;
  int META_BG       = -1;

  // Zegar
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  initial begin
    if ($test$plusargs("DUMP")) begin
      $dumpfile("test_LDPC_encode.vcd");
      $dumpvars(0, test_LDPC_encode);
    end
  end

  // ==========================================================================
  // DEBUGGER
  // ==========================================================================
  integer cyc;
  integer prev_fsm1;
  integer prev_fsm2;
  integer prev_fsm3;
  integer prev_memdone;
  integer prev_pbdone;
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
          prev_memdone <= -1;
          prev_pbdone <= -1;
        end else begin
          cyc <= cyc + 1;

          if ((cyc % 1000) == 0) begin
            $display("cyc=%0d, fsm2=%0d, fsm3=%0d, rot=%0d, bshift=%0d, wshift=%0d, zc_mod_w=%0d, i0=%0d, i1=%0d, i2=%0d, zc=%0d, active_words=%0d, rowptr_addr=%0d",
              cyc, dut.state_fsm2, dut.state_fsm3, dut.rot, dut.bshift, dut.wshift, dut.zc_mod_w, dut.i0, dut.i1, dut.i2, dut.zc, dut.active_words, dut.rowptr_addr
            );
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
			 
			 if (prev_out_last != dut.out_last && prev_out_last_check == 0) begin
            $display("tout_last = %0d ",
              cyc,
            );
            prev_out_last_check <= 1;
          end
        end
      end
	 end
	 
  endgenerate

  
  // === POMOCNICZE: tekst / parsowanie ===
  function automatic string trim_eol(input string s);
    while (s.len() > 0) begin
      byte c = s[s.len()-1];
      if (c == "\n" || c == "\r") s = s.substr(0, s.len()-2);
      else break;
    end
    return s;
  endfunction

  function automatic bit is01(input byte c);
    return (c=="0" || c=="1");
  endfunction

  // INPUT: czytaj 0/1 do pierwszego "-1"
  task automatic parse_input_line(
    input  string line_in,
    output bit    q[$]
  );
    string line;
    int i;

    q = {};
    line = trim_eol(line_in);

    for (i = 0; i < line.len(); i++) begin
      if (line[i] == "-" && (i+1) < line.len() && line[i+1] == "1") begin
        break;
      end
      if (is01(line[i])) begin
        q.push_back(line[i] == "1");
      end
    end
  endtask

  // GOLDEN: ignoruj KAŻDE "-1
  task automatic parse_golden_line(
    input  string line_in,
    output bit    q[$]
  );
    string line;
    int i;

    q = {};
    line = trim_eol(line_in);

    i = 0;
    while (i < line.len()) begin
      if (line[i] == "-" && (i+1) < line.len() && line[i+1] == "1") begin
        i += 2; // zjedz "-1"
        continue;
      end

      if (is01(line[i])) begin
        q.push_back(line[i] == "1");
      end
      i++;
    end
  endtask

  task automatic read_input_lines(
    input  string path,
    output bit    cb[int][$]
  );
    int fd;
    string line;
    int idx;

    cb.delete();
    idx = 0;

    fd = $fopen(path, "r");
    if (fd == 0) begin
      $display("ERROR: Nie moge otworzyc INPUT: %s", path);
      $finish;
    end

    while ($fgets(line, fd)) begin
      bit q[$];
      line = trim_eol(line);
      if (line.len() == 0) continue;

      parse_input_line(line, q);
      cb[idx] = q;
      idx++;
    end

    $fclose(fd);
  endtask

  task automatic read_golden_lines(
    input  string path,
    output bit    cb[int][$]
  );
    int fd;
    string line;
    int idx;

    cb.delete();
    idx = 0;

    fd = $fopen(path, "r");
    if (fd == 0) begin
      $display("ERROR: Nie moge otworzyc GOLDEN: %s", path);
      $finish;
    end

    while ($fgets(line, fd)) begin
      bit q[$];
      line = trim_eol(line);
      if (line.len() == 0) continue;

      parse_golden_line(line, q);
      cb[idx] = q;
      idx++;
    end

    $fclose(fd);
  endtask

  task automatic read_meta(input string path);
    int fd;
    string line;
    int eq_pos;
    string key, val_s;
    int v;

    fd = $fopen(path, "r");
    if (fd == 0) begin
      $display("UWAGA: Nie moge otworzyc META: %s (jadę na domyślnych)", path);
      return;
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

      if ($sscanf(val_s, "%d", v) != 1) continue;

      if      (key == "CB_COUNT")    META_CB_COUNT = v;
      else if (key == "K")           META_K        = v;
      else if (key == "FILLER_CNT")  META_F        = v;
      else if (key == "ZC")          META_ZC       = v;
      else if (key == "BG")          META_BG       = v;
    end

    $fclose(fd);
  endtask

  // Driver: podaje jeden CB (bity pakowane MSB-first w 64-bit)
  task automatic drive_one_cb(
    input int cb_idx,
    input int cb_count_total
  );
    int nbits;
    int di;
    logic [W-1:0] word;

    if (!tb_cb.exists(cb_idx)) begin
      $display("ERROR: Brak tb_cb[%0d]", cb_idx);
      return;
    end
    nbits = tb_cb[cb_idx].size();

    $display("SEND CB=%0d bits=%0d", cb_idx, nbits);

    di = 0;
    while (di < nbits) begin
      for (int b = 0; b < W; b++) begin
        if ((di + b) < nbits) word[W-1-b] = tb_cb[cb_idx][di + b];
        else                  word[W-1-b] = 1'b0;
      end

      @(posedge clk);
      in_valid   <= 1'b1;
      in_chunk   <= word;
      in_last    <= ((di + W) >= nbits);
      in_last_cb <= ((di + W) >= nbits) && (cb_idx == (cb_count_total-1));

      di += W;
    end

    @(posedge clk);
    in_valid   <= 1'b0;
    in_chunk   <= '0;
    in_last    <= 1'b0;
    in_last_cb <= 1'b0;
  endtask

  // === Liczniki błędów/podsumowanie (non-fatal) ===
  int total_bit_errors = 0;
  int total_cb_failed  = 0;

  // Porównanie ciągłego strumienia bitów: dut_bits vs golden_flat.
  // Błędy zliczane, brak przerywania symulacji.
  task automatic check_stream_bits_nonfatal(
    input bit dut_bits[$],
    input bit golden_bits[$]
  );
    int glen, dlen;
    int ncmp;
    int err_cnt;
    int max_print;

    max_print = MAX_ERR_PRINT_DEFAULT;
    void'($value$plusargs("MAX_ERR_PRINT=%d", max_print));

    glen = golden_bits.size();
    dlen = dut_bits.size();
    ncmp = (dlen < glen) ? dlen : glen;

    err_cnt = 0;

    for (int i = 0; i < ncmp; i++) begin
      if (dut_bits[i] !== golden_bits[i]) begin
        err_cnt++;
        total_bit_errors++;

        if (err_cnt <= max_print) begin
          $display("BIT_ERR stream idx=%0d got=%0d exp=%0d", i, dut_bits[i], golden_bits[i]);
        end
      end
    end

    if (dlen != glen) begin
      $display("LEN_MISMATCH stream: dut_bits=%0d golden_bits=%0d", dlen, glen);
      total_cb_failed++;
    end else if (err_cnt != 0) begin
      $display("STREAM_ERRORS: mismatches=%0d (shown<=%0d)", err_cnt, max_print);
      total_cb_failed++;
    end else begin
      $display("STREAM_OK: bits=%0d", dlen);
    end
  endtask

  // === MAIN ===
  initial begin : main
    int tb_cnt;
    int gold_cnt;
    int cb_cnt;
    bit ob;

    rst        = 1'b0;
    in_valid   = 1'b0;
    in_chunk   = '0;
    in_last    = 1'b0;
    in_last_cb = 1'b0;

    in_k          = 16'd0;
    in_filler_cnt = 16'd0;
    in_zc         = 9'd0;
    in_bg         = 1'b0;

    // Matlab w jednym katalogu
    in_file       = {VEC_DIR_MAT,   "/", CASE, "_in_bits",  ".txt"};
    golden_file   = {VEC_DIR_MAT,   "/", CASE, "_out_bits", ".txt"};
    meta_file     = {VEC_DIR_MAT,   "/", CASE, "_meta",     ".txt"};
    out_dump_file = {VEC_DIR_RESULT,"/", CASE, "_dut_out_bits", ".txt"};

    repeat (5) @(posedge clk);
    rst <= 1'b1;
    @(posedge clk);

    // META -> parametry kodera
    read_meta(meta_file);
    if (META_K  >= 0) in_k          = META_K[15:0];
    if (META_F  >= 0) in_filler_cnt = META_F[15:0];
    if (META_ZC >= 0) in_zc         = META_ZC[8:0];
    if (META_BG >= 0) begin
      if (META_BG == 1) in_bg = 1'b1;
      else              in_bg = 1'b0;
    end

    $display("META: CB_COUNT=%0d K=%0d FILLER=%0d ZC=%0d BG(raw)=%0d | in_bg=%0b",
             META_CB_COUNT, META_K, META_F, META_ZC, META_BG, in_bg);

    read_input_lines(in_file, tb_cb);
    read_golden_lines(golden_file, golden_cb);

    tb_cnt   = tb_cb.num();
    gold_cnt = golden_cb.num();
    cb_cnt   = tb_cnt;
    if (META_CB_COUNT > 0) cb_cnt = META_CB_COUNT;

    if (tb_cnt < cb_cnt) begin
      $display("ERROR: Input ma za malo CB: %0d, oczekiwane %0d", tb_cnt, cb_cnt);
      $finish;
    end
    if (gold_cnt < cb_cnt) begin
      $display("ERROR: Golden ma za malo CB: %0d, oczekiwane %0d", gold_cnt, cb_cnt);
      $finish;
    end

    fd_dump = $fopen(out_dump_file, "w");
    if (fd_dump == 0) begin
      $display("ERROR: Nie moge otworzyc do zapisu: %s", out_dump_file);
      $finish;
    end

    fork
      // =========================
      // DRIVER
      // =========================
      begin : drv
        for (int cb = 0; cb < cb_cnt; cb++) begin
          drive_one_cb(cb, cb_cnt);
        end
      end

      // =========================
      // MONITOR + DUMP + CHECK
      // =========================
      begin : mon
        bit golden_flat[$];
        bit dut_bits[$];

        // flatten: CB0 || CB1 || ... (bez separatorów)
        golden_flat = {};
        for (int cb = 0; cb < cb_cnt; cb++) begin
          for (int i = 0; i < golden_cb[cb].size(); i++) begin
            golden_flat.push_back(golden_cb[cb][i]);
          end
        end
        $display("GOLDEN_FLAT bits=%0d (cb_cnt=%0d)", golden_flat.size(), cb_cnt);

        dut_bits = {};

        forever begin
          @(posedge clk);
          #1step;

          if (out_valid) begin
            int padL, padR;
            int vb;
            int msb_idx;
            int lsb_idx;

            padL = out_pad_left;
            padR = out_pad_right;

            // clamp
            if (padL < 0) padL = 0;
            if (padR < 0) padR = 0;
            if (padL > W) padL = W;
            if (padR > W) padR = W;

            vb = W - padL - padR;     // liczba WAŻNYCH bitów
            if (vb < 0) vb = 0;
            if (vb > W) vb = W;

            msb_idx = (W-1) - padL;   // start okna od lewej
            lsb_idx = padR;           // koniec okna od prawej

            // ważne bity: [msb_idx : lsb_idx], MSB-first
            for (int b = 0; b < vb; b++) begin
              int idx;
              idx = msb_idx - b;
              if (idx < 0 || idx >= W) continue;
              if (idx < lsb_idx) break;

              ob = out_chunk[idx];
              dut_bits.push_back(ob);
              $fwrite(fd_dump, "%0d", ob);
            end

            $fwrite(fd_dump, "\n");
          end

          if (out_last) begin
            $fwrite(fd_dump, "\n");

            // Non-fatal check: porównaj cały strumień
            check_stream_bits_nonfatal(dut_bits, golden_flat);

            // PASS/FAIL wg liczników z check_stream_bits_nonfatal
            if (total_cb_failed == 0 && total_bit_errors == 0) begin
              $display("PASS: %s | dut_bits=%0d golden_bits=%0d | cb_cnt=%0d",
                       CASE, dut_bits.size(), golden_flat.size(), cb_cnt);
            end else begin
              $display("FAIL: %s | dut_bits=%0d golden_bits=%0d | cb_failed=%0d bit_errors=%0d",
                       CASE, dut_bits.size(), golden_flat.size(), total_cb_failed, total_bit_errors);
            end

            $display("Zapisano: %s", out_dump_file);
            $fclose(fd_dump);
            $finish;
          end
        end
      end

      // =========================
      // WATCHDOG
      // =========================
      begin : wdog
        repeat (TIMEOUT_CYCLES) @(posedge clk);
        $display("TIMEOUT: brak zakończenia w %0d cyklach", TIMEOUT_CYCLES);
        $display("SUMMARY (timeout): cb_failed=%0d bit_errors=%0d", total_cb_failed, total_bit_errors);
        $fclose(fd_dump);
        $finish;
      end

    join_none
  end

endmodule
