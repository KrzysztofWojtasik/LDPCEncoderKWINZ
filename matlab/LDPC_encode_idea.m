clc;close all; clear;
%

cb_count = 3;
k = 6336;
zc = 288;
bg = 1;

filler_num = [325, 326, 326];
kcb = k-filler_num;
%A = 1:6400;
A = logical(randi([0 1], sum(kcb), 1))';
data = [];

idx = 0;

for i = 1:numel(kcb)
    if (mod(kcb(i),64)==0)
        data = [data;reshape([A(idx+1:idx+kcb(i))],64,[])'];
    else
        data = [data;reshape([A(idx+1:idx+kcb(i)),zeros(1,64-mod(kcb(i),64))],64,[])'];
    end
    idx = idx + kcb(i);
end




fname = sprintf('NR_%d_%d.txt', bg, zc);
BGM_path = fullfile('..','5G_LDPC','5G_LDPC_by_manuts','base_matrices', fname);
BGM = readmatrix(BGM_path);


syndromes = cell(4,1);

for j = 1:46 %42 dla BG2
    word_idx = 0;
    word_cnt = 1;
    xor_sum = zeros(1,zc);
    for i = 1:22    %10 dla bg2
        rotate = BGM(j,i);
        if (rotate ==- 1 || word_cnt > ceil(kcb(1)/64))
            t = word_idx + zc;
            word_cnt = word_cnt + floor(t/64);
            word_idx = mod(t, 64);
            continue
        else
            acc1 = [];
            acc2 = [];
            bits_left_1 = zc - rotate;
            while bits_left_1 > 0
                avail = 64 - word_idx;              % ile zostało w bieżącym wierszu
                take  = min(bits_left_1, avail);      % ile bierzemy teraz
                if (word_cnt > ceil(kcb(1)/64))
                    acc1 = [acc1,zeros(1,bits_left_1)];
                    bits_left_1 = 0;
                    word_idx = 0;
                else
                    acc1 = [acc1, data(word_cnt, word_idx+1 : word_idx+take)];
                    bits_left_1 = bits_left_1 - take;
                    word_idx  = word_idx + take;
                    if word_idx == 64
                        word_idx = 0;
                        word_cnt = word_cnt + 1;
                    end
                end    
            end
            
            bits_left_2 = rotate;
            while bits_left_2 > 0
                avail = 64 - word_idx;              % ile zostało w bieżącym wierszu
                take  = min(bits_left_2, avail);      % ile bierzemy teraz
                if (word_cnt > ceil(kcb(1)/64))
                    acc2 = [acc2,zeros(1,bits_left_2)];
                    bits_left_2 = 0;
                    word_idx = 0;
                else
                    acc2 = [acc2, data(word_cnt, word_idx+1 : word_idx+take)];
                    bits_left_2 = bits_left_2 - take;
                    word_idx  = word_idx + take;
                    if word_idx == 64
                        word_idx = 0;
                        word_cnt = word_cnt + 1;
                    end
                end
            end
            xor_sum = xor(xor_sum,[acc2,acc1]);
        end
    end
    if j <= 4
        syndromes{j} = xor_sum;
    end
    if j == 4
        parity_blocks = calc_first_four(bg,syndromes,BGM);
    end
    if j > 4
        acc = xor_sum;
        for k = 23:22+j-1   % 23:10+j-1 dla bg2
            if (BGM(j,k) ~= -1)
                acc = xor(acc,circshift(parity_blocks{k-22},BGM(j,k))); %k-10 dla bg2
            end
        end
        parity_blocks{j} = acc;
    end
end






function parity_blocks = calc_first_four(bg,syndromes,BGM)
    if (bg == 1)
        parity_blocks = cell(46,1);
        
        acc = xor(xor(xor(syndromes{1},syndromes{2}),syndromes{3}),syndromes{4});
        parity_blocks{1} = circshift(acc,-BGM(2,23));

        parity_blocks{2} = xor(syndromes{1},circshift(parity_blocks{1},BGM(1,23)));

        parity_blocks{4} = xor(syndromes{4},circshift(parity_blocks{1},BGM(4,23)));

        parity_blocks{3} = xor(syndromes{3},parity_blocks{4});
    else
        parity_blocks = cell(42,1);
        
        acc = xor(xor(xor(syndromes{1},syndromes{2}),syndromes{3}),syndromes{4});
        parity_blocks{1} = circshift(acc,-BGM(3,11));

        parity_blocks{2} = xor(syndromes{1},circshift(parity_blocks{1},BGM(1,11)));

        parity_blocks{4} = xor(syndromes{4},circshift(parity_blocks{1},BGM(4,11)));

        parity_blocks{3} = xor(syndromes{2},parity_blocks{2});
    end

end