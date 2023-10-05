%% Unwrap the time stamp from the FLIR camera
int32_min = -2^31;
int32_max =  2^31 - 1;
data_file = load("filename.csv");
data_file_pos_norm_2pi = (data_file(1,:) - int32_min) / (int32_max - int32_min) * 2*pi;
data_file_time = unwrap(data_file_pos_norm_2pi) / (2*pi) * (2^32-1);
data_file_time = (data_file_time - data_file_time(1) )/1e9; %time in seconds

