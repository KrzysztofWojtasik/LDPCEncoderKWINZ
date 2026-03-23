function gen_ldpc_prep_vec
% GEN_LDPC_PREP_VEC
% Generator wektorów testowych dla etapu przygotowania LDPC.
% (segmentacja TB na CB + dodanie CRC do CB + wyliczenie parametrów LDPC)
% Dla każdej długości A generuje:
%   - <name>_in_bits.txt   : B bitów (TB_data = dane + CRC TB)
%   - <name>_out_bits.txt  : CB_COUNT linii, każda linia = jeden Code Block
%   - <name>_meta.txt      : metadane obliczone wg reguł segmentacji LDPC
%
% Zależności:
%   - nrCRCEncode (5G Toolbox): dodanie CRC do TB
%   - nrCodeBlockSegmentLDPC (5G Toolbox): segmentacja TB na CB dla LDPC

    % Katalog docelowy dla plików testowych
    outdir = fullfile('..','vectors/LDPC_prep/Output_Matlab');

    % Utworzenie katalogu, jeśli nie istnieje
    if ~exist(outdir, 'dir'), mkdir(outdir); end

    % Zestaw długości TB (A)
   A_list = [ 57,   463,   983,  1477,  2193, ...
               3187,  4219,  5297,  6473,  7991, ...
               9839, 12107, 14963, 18371, 22159, ...
               26841, 31777, 38693, 46927, 59761 ];

    % Stałe ziarno RNG dla powtarzalności wektorów testowych
    rng(240924, 'twister');

    for A = A_list

        % Identyfikator przypadku: długość A
        name = sprintf('A%d', A);

        % Dane wejściowe TB: losowy wektor binarny (kolumna), długość A
        data = randi([0 1], A, 1);

        % Parametry TB CRC oraz wybór Base Graph wg progu A zgodnie z 3GPP TS 38.212
        if A > 3824
            BG = 1;
            L = 24;
            poly = '24A';
        else
            BG = 2;
            L = 16;
            poly = '16';
        end

        % TB_data: dane TB z dołączonym CRC (długość B = A + L)
        TB_data = nrCRCEncode(data, poly);
        B = A + L;

        % KCB_MAX: maksymalna ilość bitów TB w jednym CB zależna od Base Graph zgodnie z 3GPP TS 38.212
        if BG == 1
            KCB_MAX = 8448;
        else
            KCB_MAX = 3840;
        end

        % Wyznaczenie liczby CB i długości po dopisaniu CRC24B per CB
        % (BC = długość danych po segmentacji, CB_COUNT = ilość CB) zgodnie z 3GPP TS 38.212
        if (B < KCB_MAX)
            CB_COUNT = 1;
            BC = B;
        else
            CB_COUNT = ceil(B / (KCB_MAX - 24));
            BC = B + CB_COUNT * 24;
        end

        % KCB: ilość bitów TB przypadająca na jeden CB po segmentacji (ceil -> możliwe dopisanie zer)
        KCB = ceil(BC / CB_COUNT);

        % KB: ilość bloków informacyjnych zawierających bity pochodzące 
        % z TB, reszta zastępowana przez bity wypełnienia zgodnie z 3GPP TS 38.212
        if BG == 1
            KB = 22;
        elseif B > 640
            KB = 10;
        elseif B > 560
            KB = 9;
        elseif B > 192
            KB = 8;
        else
            KB = 6;
        end

        % ZC_MIN i ZC: minimalny lifting oraz najbliższy dozwolony lifting spełniający wymagania
        ZC_MIN = ceil(KCB / KB);
        ZC = find_zc(ZC_MIN);

        % K: docelowa długość informacji w CB przed LDPC zgodna z 3GPP TS 38.212
        if BG == 1
            K = 22 * ZC;
        else
            K = 10 * ZC;
        end

        % FILLER_COUNT: liczba bitów wypełnienia (filler) do osiągnięcia długości K
        FILLER_COUNT = K - KCB;

        % CBS: wynik segmentacji TB_data na CB (kolumny odpowiadają kolejnym CB)
        CBS = nrCodeBlockSegmentLDPC(TB_data, BG);

        % Nazwy plików wyjściowych (per przypadek)
        in_file   = fullfile(outdir, sprintf('%s_in_bits.txt',  name));
        out_file  = fullfile(outdir, sprintf('%s_out_bits.txt', name));
        meta_file = fullfile(outdir, sprintf('%s_meta.txt',     name));

        % Metadane przypadku (format key=value, po jednej linii)
        mf = fopen(meta_file, 'w');
        fprintf(mf, 'NAME=%s\n', name);
        fprintf(mf, 'A=%d\n', A);
        fprintf(mf, 'B=%d\n', B);
        fprintf(mf, 'BC=%d\n', BC);
        fprintf(mf, 'K=%d\n', K);
        fprintf(mf, 'CB_COUNT=%d\n', CB_COUNT);
        fprintf(mf, 'ZC=%d\n', ZC);
        fprintf(mf, 'BG=%d\n', BG);
        fprintf(mf, 'FILLER_CNT=%d\n', FILLER_COUNT);
        fclose(mf);

        % Zapis TB_data (jedna linia '0/1', długość B)
        write01(in_file, TB_data);

        % Zapis CBS do jednego pliku: jedna kolumna CBS -> jedna linia tekstu
        write01_vec(out_file, CBS);

    end

    fprintf('Wygenerowano wektory: %s\n', outdir);
end


function write01(filename, bits)
% WRITE01
% Zapisuje wektor bitów jako pojedynczą linię tekstu bez separatorów.
    bits = bits(:);
    fid = fopen(filename, 'w');
    assert(fid>0, 'Nie mogę otworzyć do zapisu: %s', filename);
    fprintf(fid, '%d', bits);
    fprintf(fid, '\n');
    fclose(fid);
end


function write01_vec(filename, CBS)
% WRITE01_VEC
% Zapisuje macierz do pliku tekstowego:
%   - każda kolumna zapisywana jest jako osobna linia '0/1'
    fid = fopen(filename, 'w');
    assert(fid>0, 'Nie mogę otworzyć do zapisu: %s', filename);

    for i = 1:size(CBS, 2)
        bits = CBS(:, i);
        fprintf(fid, '%d', bits);
        fprintf(fid, '\n');
    end

    fclose(fid);
end


