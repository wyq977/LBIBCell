% visualize connections
clear all
close all
clc
TIME = 0;

%%
outfolder = 'output/';
outfile = strcat('Connection_',num2str(TIME),'.txt');
R = importdata(strcat('../',outfolder,outfile));



for k=1:1:size(R,1)
    figure(1)
    line(R(k,1:2),R(k,3:4),'Color',[(R(k,5)-1)/max(R(:,5)-1) 0 0])
end

grid on