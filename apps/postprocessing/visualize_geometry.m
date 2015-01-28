clear all
close all
clc
fig=1;

%% geometry:

G = importdata('geoOut.txt');
G = G.data;


figure(fig); fig=fig+1;
plot(G(:,2),G(:,3),'ro')
grid on
axis equal