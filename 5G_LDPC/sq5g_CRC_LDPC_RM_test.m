% Skrypt MATLAB generujący słowa kodowe LDPC zgodnie ze specyfikacją 5G NR (3GPP TS 38.212).
% Skrypt tworzy dane użytkownika, dodaje CRC, koduje je za pomocą LDPC, a następnie wykonuje dopasowanie długości (rate matching).
% Wymaga 5G Toolbox.

clear;

% Parametry konfiguracji
BG = 1;                       % Base Graph (1 lub 2)
K = 220;                     % Liczba informacji (przed rozszerzeniem CRC)
L = 24;                       % Długość CRC (np. 24 dla LDPC 5G)
E = 384;                     % Długość słowa kodowego po enkodowaniu (z redundancją)
rv = 1;                       % Redundancy version (0–3)
modulation = 'QPSK';          % Typ modulacji (np. 'QPSK', '16QAM', '64QAM')

% Generacja danych wejściowych
rng(1);
dataIn = randi([0 1], K - L, 1);  % Dane losowe o długości K - CRC

% Oblicz CRC (CRC24A według standardu 3GPP)
dataCRC = nrCRCEncode(dataIn, '24A');

% LDPC kodowanie
codedLDPC = nrLDPCEncode(dataCRC, BG);

% LDPC kodowanie - ver. 2
% c = fqEncUrb(Hgf, dataCRC, ifi);

% Rate matching (dopasowanie długości słowa kodowego)
codedRM = nrRateMatchLDPC(codedLDPC, E, rv, modulation, 1);

% Wyświetl wynik
disp('Zakodowane dane LDPC (pierwsze 50 bitów):');
disp(codedRM(1:50)');
