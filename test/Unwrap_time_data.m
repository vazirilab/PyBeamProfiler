clear 
close all 
clc

DataFile = load('data.csv'); %file with data from PyBeamProfile 

int32_min = -2^31;
int32_max =  2^31 - 1;
pos_norm_2pi = (DataFile(1,:) - int32_min) / (int32_max - int32_min) * 2*pi; 
time_sec = unwrap(pos_norm_2pi) / (2*pi) * (2^32-1); % unwrapping the time data from the camera
time_sec = (time_sec - time_sec(1) )/1e9; %time data in seconds
