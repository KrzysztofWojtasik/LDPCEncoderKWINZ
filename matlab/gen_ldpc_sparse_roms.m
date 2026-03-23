function gen_ldpc_sparse_roms()
% GEN_LDPC_SPARSE_ROMS
% Generator plików ROM dla rzadkiej reprezentacji bazowych macierzy LDPC (H_BG).
%
% Wejście:
%   - ../rtl/memory/base_matrices/NR_<bg>_<zc>.txt
%     gdzie <bg> ∈ {1,2}, <zc> = lifting size (Zc), a plik zawiera macierz bazową
%     o elementach: -1 (brak krawędzi) lub rotacja w zakresie [0..Zc-1].
%
% Wyjście (dla każdego BG osobno) w ../rtl/memory/:
%   - BG<bg>_zc_list.memh   : lista dostępnych Zc (uint16)
%   - BG<bg>_row_ptr.memh   : wskaźniki CSR per wiersz (uint16), długość ROWS+1
%   - BG<bg>_col_idx.memh   : indeksy kolumn (uint16), długość NNZ
%   - BG<bg>_shift.memh     : rotacje (uint16), długość NUM_ZC*NNZ (blokami po Zc)
%   - BG<bg>_info.txt       : parametry pomocnicze (ROWS, COLS, NNZ, NUM_ZC, SHIFT_DEPTH)
%
% Model danych:
%   - Struktura CSR budowana jest na podstawie wzorca elementów != -1.
%   - col_idx i row_ptr są wspólne dla wszystkich Zc w danym BG (wymagana zgodność maski != -1).
%   - shift zawiera wartości rotacji dla kolejnych Zc przy tej samej enumeracji krawędzi.

    base_dir = fullfile('..','rtl','memory','base_matrices');
    out_dir  = fullfile('..','rtl','memory');
    if ~exist(out_dir,'dir'), mkdir(out_dir); end

    % Wyszukanie plików bazowych macierzy: NR_<bg>_<zc>.txt
    files = dir(fullfile(base_dir,'NR_*_*.txt'));
    assert(~isempty(files), 'Nie znaleziono plików NR_<bg>_<zc>.txt w: %s', base_dir);

    % Podział na listy dla BG1 i BG2 na podstawie nazwy pliku
    [bg1_list, bg2_list] = collect_by_bg(files);

    % Generacja ROM osobno dla BG1 i BG2
    if ~isempty(bg1_list)
        gen_for_bg(1, bg1_list, base_dir, out_dir);
    else
        warning('Brak plików dla BG1.');
    end

    if ~isempty(bg2_list)
        gen_for_bg(2, bg2_list, base_dir, out_dir);
    else
        warning('Brak plików dla BG2.');
    end

    fprintf('\nGotowe. Pliki ROM w: %s\n', out_dir);
end


% =========================== BG processing ===========================

function gen_for_bg(bg, file_list, base_dir, out_dir)
% GEN_FOR_BG
% Buduje wspólną strukturę CSR (row_ptr, col_idx) oraz tablicę shift dla listy Zc.
%
% Wejście:
%   bg        - numer Base Graph (1 lub 2)
%   file_list - struct array: .name, .zc
%   base_dir  - katalog wejściowy z macierzami NR_<bg>_<zc>.txt
%   out_dir   - katalog wyjściowy dla ROM .memh oraz pliku info

    % Sortowanie rosnąco po Zc (stabilna kolejność w zc_list i shift)
    [~,ord] = sort([file_list.zc]);
    file_list = file_list(ord);

    zc_list = uint16([file_list.zc]);
    num_zc  = numel(zc_list);

    fprintf('\n=== BG%d: %d wartości ZC ===\n', bg, num_zc);

    % Odczyt pierwszej macierzy jako wzorca (rozmiar + maska != -1)
    first_path = fullfile(base_dir, file_list(1).name);
    B0 = read_bg_matrix(first_path);
    [ROWS, COLS] = size(B0);

    % Walidacja wymiarów bazowej macierzy dla BG
    if bg == 1
        exp_rows = 46; exp_cols = 68;
    else
        exp_rows = 42; exp_cols = 52;
    end
    assert(ROWS == exp_rows && COLS == exp_cols, ...
        'BG%d: zły rozmiar macierzy w %s (jest %dx%d, oczekiwane %dx%d)', ...
        bg, file_list(1).name, ROWS, COLS, exp_rows, exp_cols);

    % Maska krawędzi: element != -1 oznacza istniejące połączenie w grafie
    mask0 = (B0 ~= -1);

    % Budowa CSR:
    %   - row_ptr: wskaźnik początku listy kolumn dla kolejnych wierszy (ROWS+1)
    %   - col_idx: lista indeksów kolumn dla wszystkich wierszy (NNZ)
    % Enumeracja: wierszami, a w wierszu kolumny rosnąco.
    row_ptr = zeros(ROWS+1,1,'uint16');
    col_idx = zeros(nnz(mask0),1,'uint16');

    edge = 1;                 % indeks 1-based w MATLAB
    row_ptr(1) = uint16(0);   % 0-based licznik wpisów

    for r = 1:ROWS
        cols = find(mask0(r,:));     % rosnąco
        n = numel(cols);

        if n > 0
            col_idx(edge:edge+n-1) = uint16(cols-1); % 0-based dla docelowego RTL
            edge = edge + n;
        end

        % Liczba wpisów zsumowana do końca wiersza r (0-based count)
        row_ptr(r+1) = uint16(edge-1);
    end

    NNZ = numel(col_idx);
    fprintf('BG%d: NNZ = %d (liczba wpisów != -1)\n', bg, NNZ);

    % Walidacja spójności maski != -1 dla wszystkich Zc danego BG
    for i = 2:num_zc
        p = fullfile(base_dir, file_list(i).name);
        Bi = read_bg_matrix(p);
        if any((Bi ~= -1) ~= mask0, 'all')
            error('BG%d: wzór -1 różni się dla ZC=%d (plik %s). Nie można użyć jednego col_idx/row_ptr.', ...
                bg, file_list(i).zc, file_list(i).name);
        end
    end
    fprintf('BG%d: wzór != -1 zgodny dla wszystkich ZC.\n', bg);

    % Tablica shift: rotacje odpowiadające krawędziom w kolejności CSR.
    % Układ pamięci: bloki po Zc, każdy blok ma NNZ wpisów.
    shift = zeros(num_zc*NNZ, 1, 'uint16');

    for zi = 1:num_zc
        zc = double(file_list(zi).zc);
        p  = fullfile(base_dir, file_list(zi).name);
        B  = read_bg_matrix(p);

        base = (zi-1) * NNZ;   % offset bloku Zc w shift (0-based)
        edge0 = 0;

        for r = 1:ROWS
            e0 = double(row_ptr(r));     % 0-based
            e1 = double(row_ptr(r+1));   % 0-based

            if e1 > e0
                cols0 = double(col_idx(e0+1:e1)); % kolumny 0-based
                cols1 = cols0 + 1;                % kolumny 1-based do MATLAB
                vals  = B(r, cols1);

                % Walidacja: brak -1 oraz poprawny zakres rotacji
                if any(vals == -1)
                    error('BG%d: niespójność -1 dla ZC=%d w wierszu %d.', bg, zc, r-1);
                end
                if any(vals < 0 | vals >= zc)
                    error('BG%d: rotacja poza zakresem [0..ZC-1] dla ZC=%d (wiersz %d).', bg, zc, r-1);
                end

                shift(base + (e0+1:e1)) = uint16(vals);
                edge0 = e1;
            end
        end

        assert(edge0 == NNZ, ...
            'BG%d: błąd liczenia shift dla ZC=%d (edge0=%d, NNZ=%d)', bg, zc, edge0, NNZ);
    end

    % Zapis ROM (.memh): 16-bit HEX, 1 wartość na linię
    bg_tag = sprintf('BG%d', bg);

    write_memh16(fullfile(out_dir, sprintf('%s_zc_list.memh', bg_tag)), zc_list);
    write_memh16(fullfile(out_dir, sprintf('%s_row_ptr.memh', bg_tag)), row_ptr);
    write_memh16(fullfile(out_dir, sprintf('%s_col_idx.memh', bg_tag)), col_idx);
    write_memh16(fullfile(out_dir, sprintf('%s_shift.memh',   bg_tag)), shift);

    % Plik informacyjny (parametry pamięci)
    info_path = fullfile(out_dir, sprintf('%s_info.txt', bg_tag));
    fid = fopen(info_path,'w');
    fprintf(fid, 'BG%d sparse ROM\n', bg);
    fprintf(fid, 'ROWS=%d COLS=%d\n', ROWS, COLS);
    fprintf(fid, 'NNZ=%d\n', NNZ);
    fprintf(fid, 'NUM_ZC=%d\n', num_zc);
    fprintf(fid, 'SHIFT_DEPTH=%d (NUM_ZC*NNZ)\n', num_zc*NNZ);
    fclose(fid);

    fprintf('BG%d: zapisano ROM-y do %s\n', bg, out_dir);
end


% ============================== Helpers ==============================

function B = read_bg_matrix(path)
% READ_BG_MATRIX
% Odczyt macierzy bazowej z pliku tekstowego i konwersja do int16.
    B = readmatrix(path);
    if any(isnan(B), 'all')
        error('NaN w macierzy: %s (sprawdź format pliku)', path);
    end
    B = int16(B);
end


function [bg1_list, bg2_list] = collect_by_bg(files)
% COLLECT_BY_BG
% Klasyfikacja plików NR_<bg>_<zc>.txt do list BG1 i BG2 (parsowanie nazwy).
    bg1_list = struct('name',{},'zc',{});
    bg2_list = struct('name',{},'zc',{});

    for i = 1:numel(files)
        nm = files(i).name;
        tok = regexp(nm, '^NR_(\d)_(\d+)\.txt$', 'tokens', 'once');
        if isempty(tok), continue; end

        bg = str2double(tok{1});
        zc = str2double(tok{2});

        if bg == 1
            bg1_list(end+1) = struct('name', nm, 'zc', zc); %#ok<AGROW>
        elseif bg == 2
            bg2_list(end+1) = struct('name', nm, 'zc', zc); %#ok<AGROW>
        end
    end
end


function write_memh16(path, data_u16)
% WRITE_MEMH16
% Zapis danych 16-bit w formacie HEX (4 cyfry) jako 1 wartość na linię.
    fid = fopen(path,'w');
    if fid < 0
        error('Nie mogę otworzyć pliku do zapisu: %s', path);
    end
    for i = 1:numel(data_u16)
        fprintf(fid, '%04X\n', uint16(data_u16(i)));
    end
    fclose(fid);
end
