function find_zc = find_zc(zc_min)
% FIND_ZC
% Mapowanie zc_min na najbliższą wartość Zc >= zc_min.
    if (zc_min <=   2) find_zc = 2;
    elseif (zc_min <=   3) find_zc = 3;
    elseif (zc_min <=   4) find_zc = 4;
    elseif (zc_min <=   5) find_zc = 5;
    elseif (zc_min <=   6) find_zc = 6;
    elseif (zc_min <=   7) find_zc = 7;
    elseif (zc_min <=   8) find_zc = 8;
    elseif (zc_min <=   9) find_zc = 9;
    elseif (zc_min <=  10) find_zc = 10;
    elseif (zc_min <=  11) find_zc = 11;
    elseif (zc_min <=  12) find_zc = 12;
    elseif (zc_min <=  13) find_zc = 13;
    elseif (zc_min <=  14) find_zc = 14;
    elseif (zc_min <=  15) find_zc = 15;
    elseif (zc_min <=  16) find_zc = 16;
    elseif (zc_min <=  18) find_zc = 18;
    elseif (zc_min <=  20) find_zc = 20;
    elseif (zc_min <=  22) find_zc = 22;
    elseif (zc_min <=  24) find_zc = 24;
    elseif (zc_min <=  26) find_zc = 26;
    elseif (zc_min <=  28) find_zc = 28;
    elseif (zc_min <=  30) find_zc = 30;
    elseif (zc_min <=  32) find_zc = 32;
    elseif (zc_min <=  36) find_zc = 36;
    elseif (zc_min <=  40) find_zc = 40;
    elseif (zc_min <=  44) find_zc = 44;
    elseif (zc_min <=  48) find_zc = 48;
    elseif (zc_min <=  52) find_zc = 52;
    elseif (zc_min <=  56) find_zc = 56;
    elseif (zc_min <=  60) find_zc = 60;
    elseif (zc_min <=  64) find_zc = 64;
    elseif (zc_min <=  72) find_zc = 72;
    elseif (zc_min <=  80) find_zc = 80;
    elseif (zc_min <=  88) find_zc = 88;
    elseif (zc_min <=  96) find_zc = 96;
    elseif (zc_min <= 104) find_zc = 104;
    elseif (zc_min <= 112) find_zc = 112;
    elseif (zc_min <= 120) find_zc = 120;
    elseif (zc_min <= 128) find_zc = 128;
    elseif (zc_min <= 144) find_zc = 144;
    elseif (zc_min <= 160) find_zc = 160;
    elseif (zc_min <= 176) find_zc = 176;
    elseif (zc_min <= 192) find_zc = 192;
    elseif (zc_min <= 208) find_zc = 208;
    elseif (zc_min <= 224) find_zc = 224;
    elseif (zc_min <= 240) find_zc = 240;
    elseif (zc_min <= 256) find_zc = 256;
    elseif (zc_min <= 288) find_zc = 288;
    elseif (zc_min <= 320) find_zc = 320;
    elseif (zc_min <= 352) find_zc = 352;
    elseif (zc_min <= 384) find_zc = 384;
    else                   find_zc = 0; % poza zakresem
    end
end
