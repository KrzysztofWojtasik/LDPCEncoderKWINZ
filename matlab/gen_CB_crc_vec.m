function gen_CB_crc_vec()
% GEN_CB_CRC_VEC
% Generator wektorów testowych dla modułu liczącego i dodającego CRC do bloku kodowego (CB).
%
% Dla każdej długości A generuje:
%   - <name>_in_bits.txt   : A bitów danych wejściowych
%   - <name>_out_bits.txt  : A+L bitów (dane + CRC)
%   - <name>_crc_only.txt  : L bitów CRC
%   - <name>_meta.txt      : metadane (NAME, A, POLY, L)
%
% Parametry CRC zgodnie z 3GPP TS 38.212 dla CB:
%   - POLY = CRC24B
%   - L    = 24
%
% Zależności:
%   - nrCRCEncode (5G Toolbox): kodowanie CRC w referencyjnej implementacji MATLAB.

    % Katalog docelowy dla plików testowych
    outdir = fullfile('..','vectors/CB_crc/Output_Matlab');

    % Utworzenie katalogu, jeśli nie istnieje
    if ~exist(outdir, 'dir'), mkdir(outdir); end

    % Zestaw długości CB (A)
    A_list = [ 57,   463,   983,  1477,  2193, ...
               3187,  4219,  5297,  6473,  7991, ...
               9839, 12107, 14963, 18371, 22159, ...
               26841, 31777, 38693, 46927, 59761 ];

    % Stałe ziarno RNG dla powtarzalności wektorów testowych
    rng(240924, 'twister');

    % Stałe parametry CRC dla CB
    poly = '24B';
    L = 24;

    for A = A_list

        % Identyfikator przypadku: długość A
        name = sprintf('A%d', A);

        % Dane wejściowe: losowy wektor binarny, długość A
        data = logical(randi([0 1], A, 1));

        % Referencyjne kodowanie CRC:
        % data_crc = [data; crc_bits] o długości A+L
        data_crc = nrCRCEncode(data, poly);

        % Wyodrębnienie samych bitów CRC (ostatnie L bitów)
        crc_only = data_crc(end-L+1:end);

        % Nazwy plików wyjściowych
        in_file   = fullfile(outdir,  sprintf('%s_in_bits.txt',   name));
        out_file  = fullfile(outdir,  sprintf('%s_out_bits.txt',  name));
        crc_file  = fullfile(outdir,  sprintf('%s_crc_only.txt',  name));
        meta_file = fullfile(outdir,  sprintf('%s_meta.txt',      name));

        % Zapis sekwencji bitów jako pojedyncza linia znaków '0'/'1'
        write01(in_file,  data);
        write01(out_file, data_crc);
        write01(crc_file, crc_only);

        % Metadane przypadku (format key=value, po jednej linii)
        mf = fopen(meta_file, 'w');
        fprintf(mf, 'NAME=%s\n', name);
        fprintf(mf, 'A=%d\n', A);
        fprintf(mf, 'POLY=%s\n', poly);   % '24B'
        fprintf(mf, 'L=%d\n', L);         % 24
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
