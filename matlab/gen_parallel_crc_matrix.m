clc; clear; close;

% =========================================================================
% Generator macierzy równoległej aktualizacji CRC dla kroku N-bitowego.
%
% Skrypt wyznacza macierze liniowe H1 i H2 dla zadanych wielomianów CRC
% oraz generuje zestaw równań XOR (w postaci linii tekstu) opisujących
% zależność crc_out od din oraz crc_in.
%
% Parametry:
%   N  - liczba bitów danych w jednym kroku przetwarzania (równoległość)
%   M  - długość CRC (liczba bitów rejestru CRC)
%
% Macierze:
%   H1 (N×M) - wpływ wektora danych din[0..N-1] na wyjście crc_out[0..M-1]
%   H2 (M×M) - wpływ stanu początkowego crc_in[0..M-1] na wyjście crc_out[0..M-1]
%
%   Funkcja emit_crc_assigns emituje linie postaci:
%       crc_out[idx] = <xor-term>;
% =========================================================================

N    = 64;      % szerokość kroku danych
M24A = 24;      % długość CRC24A
M24B = 24;      % długość CRC24B
M16  = 16;      % długość CRC16

 outdir = fullfile('..','vectors/Parallel_Matrices');
 if ~exist(outdir, 'dir'), mkdir(outdir); end


% Wielomiany CRC zapisane w hex; hex2poly zwraca współczynniki z wiodącym wyrazem.
% Odcinamy pierwszy element, aby uzyskać maskę sprzężenia zwrotnego długości M.
poly24A = hex2poly('1864CFB');  
poly24A = poly24A(2:end);

poly24B = hex2poly('1800063');  
poly24B = poly24B(2:end);

poly16  = hex2poly('11021');
poly16  = poly16(2:end);

% Macierze wpływu danych (H1): pobudzenie bazą jednostkową din przy crc_in=0
H1_24A = gen_H1(N, poly24A);
H1_24B = gen_H1(N, poly24B);
H1_16  = gen_H1(N, poly16);

% Macierze wpływu stanu (H2): pobudzenie bazą jednostkową crc_in przy din=0
H2_24A = gen_H2(N, M24A, poly24A);
H2_24B = gen_H2(N, M24B, poly24B);
H2_16  = gen_H2(N, M16,  poly16);

% Emisja linii równań XOR do plików .vh
emit_crc_assigns(H1_24A, H2_24A, 'din', 'crc_in', 'crc_out', fullfile(outdir, 'crc_parallel_24A.vh'));
emit_crc_assigns(H1_24B, H2_24B, 'din', 'crc_in', 'crc_out', fullfile(outdir, 'crc_parallel_24B.vh'));
emit_crc_assigns(H1_16,  H2_16,  'din', 'crc_in', 'crc_out', fullfile(outdir, 'crc_parallel_16.vh'));


function H1 = gen_H1(N, poly)
% gen_H1: budowa macierzy H1 (N×M).
% Metoda: dla każdego i∈[1..N] wyznaczany jest crc_out dla wektora danych,
%         w którym ustawiony jest wyłącznie bit i (baza jednostkowa).
% Wejścia:
%   N    - liczba bitów danych w kroku
%   poly - maska wielomianu CRC (długość M)
% Wyjście:
%   H1   - macierz logiczna (N×M); wiersz i odpowiada wpływowi din(i)

    data = [];
    H1   = [];

    L = numel(poly);      % L = M (długość CRC)
    for i = 1:N
        lZeros = N - i;
        rZeros = i - 1;
        row = [zeros(1,lZeros) 1 zeros(1,rZeros)];  % wektor jednostkowy długości N
        data = [data; row];%#ok<AGROW>
    end

    crc = false(1, L);    % stan początkowy CRC = 0

    for i = 1:N
        row = crc_parallel(data(i,:), crc, poly);   % crc_out dla pobudzenia din(i)=1
        H1  = [H1; row]; %#ok<AGROW>
    end
end


function H2 = gen_H2(N, M, poly)
% gen_H2: budowa macierzy H2 (M×M).
% Metoda: dane wejściowe ustawione na 0; dla każdego i∈[1..M] wyznaczany
%         jest crc_out dla stanu wejściowego crc_in będącego wektorem jednostkowym.
% Wejścia:
%   N    - liczba bitów danych w kroku
%   M    - długość CRC
%   poly - maska wielomianu CRC (długość M)
% Wyjście:
%   H2   - macierz logiczna (M×M); wiersz i odpowiada wpływowi crc_in(i)

    data = zeros(1, N);   % din = 0...0
    H2   = [];
    crc  = [];
    L = numel(poly); %#ok<NASGU>  % formalnie L == M

    for i = 1:M
        lZeros = M - i;
        rZeros = i - 1;
        row = [zeros(1,lZeros) 1 zeros(1,rZeros)];  % wektor jednostkowy długości M
        crc = [crc; row]; %#ok<AGROW>
    end

    for i = 1:M
        row = crc_parallel(data, crc(i,:), poly);   % crc_out dla pobudzenia crc_in(i)=1
        H2  = [H2; row]; %#ok<AGROW>
    end
end


function crc_out = crc_serial(data, crc_in, POLY)
% crc_serial: pojedynczy krok aktualizacji CRC dla 1 bitu danych.
% Wejścia:
%   data   - pojedynczy bit danych (0/1)
%   crc_in - stan CRC (1×M)
%   POLY   - maska wielomianu (1×M)
% Wyjście:
%   crc_out - stan CRC po przetworzeniu 1 bitu

    poly = logical(POLY);

    feedback  = xor(data, crc_in(1));
    crc_shift = [crc_in(2:end), false];

    if feedback
        crc_out = xor(crc_shift, poly);
    else
        crc_out = crc_shift;
    end
end


function crc_out = crc_parallel(data, crc_in, POLY)
% crc_parallel: referencyjne przetworzenie wektora N bitów metodą bitową.
% Wejścia:
%   data   - wektor danych (1×N)
%   crc_in - stan CRC (1×M)
%   POLY   - maska wielomianu (1×M)
% Wyjście:
%   crc_out - stan CRC po przetworzeniu N bitów

    poly = logical(POLY);
    crc_out = crc_in;
    N = numel(data);

    for i = 1:N
        crc_out = crc_serial(data(i), crc_out, poly);
    end
end


function emit_crc_assigns(H1, H2, sig_din, sig_cin, sig_cout, fname)
% emit_crc_assigns: emisja równań XOR na podstawie H1 i H2.
%
% Wejścia:
%   H1       - macierz N×M (wkład din -> crc_out)
%   H2       - macierz M×M (wkład crc_in -> crc_out)
%   sig_din  - nazwa sygnału danych w docelowym kodzie (np. 'din')
%   sig_cin  - nazwa sygnału stanu CRC wejściowego (np. 'crc_in')
%   sig_cout - nazwa sygnału stanu CRC wyjściowego (np. 'crc_out')
%   fname    - ścieżka do pliku wyjściowego; gdy pusty, wypisuje na ekran
%
% Interpretacja:
%   Dla każdej kolumny i:
%     crc_out[lhs_index] jest XOR-em wszystkich din[k] i crc_in[j],
%     dla których H1(k,i)=1 oraz H2(j,i)=1.
%


    [N, M1] = size(H1);
    [M2, M3] = size(H2);
    assert(M1==M2 && M2==M3, 'H1 musi miec rozmiar N×M, H2 musi mieć rozmiar M×M; niezgodne M.');
    M = M2;

    lines = strings(M,1);

    for i = 1:M
        terms = strings(0);

        % Wkład crc_in: indeksy 0..M-1
        idx_cin = find(H2(:,i) ~= 0) - 1;
        for t = 1:numel(idx_cin)
            terms(end+1) = sprintf('%s[%d]', sig_cin, idx_cin(t)); %#ok<AGROW>
        end

        % Wkład din: indeksy 0..N-1
        idx_din = find(H1(:,i) ~= 0) - 1;
        for t = 1:numel(idx_din)
            terms(end+1) = sprintf('%s[%d]', sig_din, idx_din(t)); %#ok<AGROW>
        end

        if isempty(terms)
            rhs = '1\b0';
        else
            rhs = terms(1);
            for t = 2:numel(terms)
                rhs = rhs + " ^ " + terms(t);
            end
        end

        lhs_index = M - i;
        lines(i) = sprintf('%s[%d] = %s;', sig_cout, lhs_index, rhs);
    end

    if nargin>=6 && ~isempty(fname)
        fid = fopen(fname, 'w');
        assert(fid > 0, 'Nie mogę otworzyć do zapisu: %s', fname);
        for i = 1:M
            fprintf(fid, '%s\n', lines(i));
        end
        fclose(fid);
        fprintf('Zapisano %d linii do: %s\n', M, fname);
    else
        for i = 1:M, disp(lines(i)); end
    end
end
