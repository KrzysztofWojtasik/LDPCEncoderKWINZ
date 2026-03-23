
%% Option 2: explicit parity check matrix usage
% Parity check matrices are taken from /5G_LDPC_by_manuts/base_matrices

% Set code configuration
BG = 1; % base graph (1 / 2)
Z = 8;
K = 176;
if ~ismember(BG, [1 2]), error("Wrong BG."); end
if (Z<0), error("Wrong Z value."); end

SetsOfLiftSizes = [2, 4, 8, 16, 32, 64, 128, 256;    3, 6, 12, 24, 48, 96, 192, 384;   5, 10, 20, 40, 80, 160, 320, -1;...
                    7, 14, 28, 56, 112, 224, -1, -1;  9, 18, 36, 72, 144, 288, -1, -1;  11, 22, 44, 88, 176, 352, -1, -1;...
                    13, 26, 52, 104, 208, -1, -1, -1; 15, 30, 60, 120, 240, -1, -1, -1];

% Load PCMs
iLS = -1;
for k=1:8
    if ismember(Z, SetsOfLiftSizes(k,:))
        iLS = k-1;
    end
end
if iLS==-1, error("Given Z value not supported."); end
bmFileName = "5G_LDPC_by_manuts/base_matrices/NR_"+BG+"_"+iLS+"_"+Z+".txt";
B = load(bmFileName);

% Partition into High-Rate Code (HRC) part nad Incremental Redndancy (IR) part
if BG==1
    Hbase_HRC = double(B(1:4,1:26)>-1);
    H_HRC = fqSetCycshift(Hbase_HRC, Z, B(1:4,1:26));
    Hbase_IRC = double(B(5:46,1:26)>-1);
    H_IRC = fqSetCycshift(Hbase_IRC, Z, B(5:46,1:26));
    ifi_HRC = fqInvfi(H_HRC);
elseif BG==2
    Hbase_HRC = double(B(1:4,1:14)>-1);
    H_HRC = fqSetCycshift(Hbase_HRC, Z, B(1:4,1:14));
    Hbase_IRC = double(B(5:42,1:14)>-1);
    H_IRC = fqSetCycshift(Hbase_IRC, Z, B(5:42,1:14));
    ifi_HRC = fqInvfi(H_HRC);
end

% Save
% saveFileName = "H5G_BG"+BG+"_"+size(H_HRC,1)+"x"+size(H_HRC,2)+"_"+size(H_IRC,1)+"x"+size(H_IRC,2)+"_"+"P"+Z;
% save(saveFileName, 'H_HRC', 'Hbase_HRC', 'H_IRC', 'Hbase_IRC', 'ifi_HRC');

% Experimental coding
msg = randi([0 1],K,1); %generate random k-bit message
%Encoding 
cword = nrldpc_encode(B,Z,msg');
