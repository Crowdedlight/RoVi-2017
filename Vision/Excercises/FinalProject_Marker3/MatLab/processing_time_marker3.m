clear all; clc; format short;

load('../Outfiles/processing_time_easy.txt');
load('../Outfiles/processing_time_hard.txt');

figure ('Name','Marker 3 Easy');
plot(processing_time_easy)

figure ('Name','Marker 3 Hard');
plot(processing_time_hard)

mean_easy = mean(processing_time_easy);
mean_hard = mean(processing_time_hard);

var_easy = var(processing_time_easy);
var_hard = var(processing_time_hard);

disp("Mean for Easy: " + mean_easy);
disp("Mean for Hard: " + mean_hard);

disp("Variance for Easy: " + var_easy);
disp("Variance for Hard: " + var_hard);
