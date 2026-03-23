function gen_TB_crc_vec()
% GEN_TB_CRC_VEC
% Generator wektorów testowych dla modułu liczącego i dodającego CRC do bloku transportowego (TB).
%
% Dla każdej długości A  generuje:
%   - in_bits_<name>.txt   : A bitów danych wejściowych
%   - out_bits_<name>.txt  : A+L bitów (dane + CRC)
%   - crc_only_<name>.txt  : L bitów CRC
%   - meta_<name>.txt      : metadane (NAME, A, POLY, L)
%
% Dobór polinomu zgodnie z 3GPP TS 38.212 dla TB:
%   - A > 3824  -> CRC24A (L=24)
%   - A <= 3824 -> CRC16  (L=16)
%
% Zależności:
%   - nrCRCEncode (5G Toolbox): kodowanie CRC w referencyjnej implementacji MATLAB.

    % Katalog docelowy dla plików testowych
    outdir  = fullfile('..','vectors/TB_crc/Output_Matlab');

    % Utworzenie katalogów, jeśli nie istnieją
    if ~exist(outdir,  'dir'), mkdir(outdir);  end

    % Zestaw długości TB (A):
    A_list = [ 57,   463,   983,  1477,  2193, ...
               3187,  4219,  5297,  6473,  7991, ...
               9839, 12107, 14963, 18371, 22159, ...
               26841, 31777, 38693, 46927, 59761 ];

    % Stałe ziarno RNG dla powtarzalności wektorów testowych
    rng(240924, 'twister');


    for A = A_list

        % Identyfikator przypadku: długość A + numer sekwencji
        name = sprintf('A%d', A);

        % Dane wejściowe: losowy wektor binarny (kolumna logical), długość A
        data = logical(randi([0 1], A, 1));

        % Parametry CRC zależne od A (dobór polinomu i długości CRC)
        if A > 3824
            poly = '24A';
            L = 24;
        else
            poly = '16';
            L = 16;
        end

        % Referencyjne kodowanie CRC:
        % data_crc = [data; crc_bits] o długości A+L
        data_crc = nrCRCEncode(data, poly);

        % Wyodrębnienie samych bitów CRC (ostatnie L bitów)
        crc_only = data_crc(end-L+1:end);

        % Nazwy plików wyjściowych (per przypadek)
        in_file   = fullfile(outdir,  sprintf('%s_in_bits.txt',  name));
        out_file  = fullfile(outdir, sprintf('%s_out_bits.txt', name));
        crc_file  = fullfile(outdir, sprintf('%s_crc_only.txt', name));
        meta_file = fullfile(outdir,  sprintf('%s_meta.txt',     name));

        % Zapis sekwencji bitów jako pojedyncza linia znaków '0'/'1'
        write01(in_file,  data);
        write01(out_file, data_crc);
        write01(crc_file, crc_only);

        % Metadane przypadku (format key=value, po jednej linii)
        mf = fopen(meta_file, 'w');
        fprintf(mf, 'NAME=%s\n', name);
        fprintf(mf, 'A=%d\n', A);
        fprintf(mf, 'POLY=%s\n', poly);   % '16' lub '24A'
        fprintf(mf, 'L=%d\n', L);         % 16 lub 24
        fclose(mf);

    end

    fprintf('Wygenerowano wektory: %s\n', outdir);
end


function write01(filename, bits)
% WRITE01
% Zapisuje wektor bitów jako pojedynczą linię tekstu bez separatorów:
%   np. 001011010...
%
% Wejście:
%   bits - wektor (logical lub 0/1), wiersz lub kolumna
    bits = bits(:);  % wymuszenie postaci kolumnowej (A×1)
    fid = fopen(filename, 'w');
    assert(fid>0, 'Nie mogę otworzyć do zapisu: %s', filename);
    fprintf(fid, '%d', bits);   % zapis bez spacji i separatorów
    fprintf(fid, '\n');
    fclose(fid);
end
