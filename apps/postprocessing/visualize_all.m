clear all
close all
clc
fig=1;
TIME = 7000;

%% geometry nodes:
outfolder = 'output/';
outfile = strcat('GeometryNodes_',num2str(TIME),'.txt');
G = importdata(strcat('../',outfolder,outfile));


%% velocity:
outfolder = 'output/';
outfile = strcat('Velocity_',num2str(TIME),'.txt');

% posX posY velX velY
V = importdata(strcat('../', outfolder, outfile) );

UX = zeros(max(V(:,1))-min(V:1),max(V(:,2))-min(V(:,2)));
UY = zeros(max(V(:,1))-min(V:1),max(V(:,2))-min(V(:,2)));

% put in matrix form:
for i=1:1:size(V,1)
   UX(V(i,1)+1,V(i,2)+1) = V(i,3);
   UY(V(i,1)+1,V(i,2)+1) = V(i,4);
end


%% HH protein concentration:
outfolder = 'output/';
outfile = strcat('HH_',num2str(TIME),'.txt');

CC = importdata(strcat('../',outfolder,outfile));

% put in matrix form:
for i=1:1:size(CC,1)
    HH(CC(i,1)+1,CC(i,2)+1) = CC(i,3);
end


%% domain ID:
outfolder = 'output/';
outfile = strcat('DomainID_',num2str(TIME),'.txt');
CC = importdata(strcat('../',outfolder,outfile));

% put in matrix form:
for i=1:1:size(CC,1)
    ID(CC(i,1)+1,CC(i,2)+1) = CC(i,3);
end

%% CellType:
outfolder = 'output/';
outfile = strcat('CellType_',num2str(TIME),'.txt');
CC = importdata(strcat('../',outfolder,outfile));

% put in matrix form:
for i=1:1:size(CC,1)
    CT(CC(i,1)+1,CC(i,2)+1) = CC(i,3);
end


%% plot:
figure(fig); fig=fig+1;

subplot(2,3,1)
imagesc(UX(2:end,2:end)')
hold on
plot(G(:,2),G(:,3),'r.')
quiver(G(:,2),G(:,3),G(:,4),G(:,5),0.1)
colorbar
axis equal
axis xy;
title('x velocity')

subplot(2,3,2)
imagesc(UY(2:end,2:end)')
hold on
plot(G(:,2),G(:,3),'r.')
quiver(G(:,2),G(:,3),G(:,4),G(:,5),0.1)
colorbar
axis equal
axis xy;
title('y velocity')

subplot(2,3,3)
imagescnan(HH(2:end,2:end)')
hold on
plot(G(:,2),G(:,3),'r.')
quiver(G(:,2),G(:,3),G(:,4),G(:,5),0.1)
colorbar
axis equal
axis xy;
title('HH protein concentration')

subplot(2,3,4)
imagesc(ID(2:end,2:end)')
hold on
plot(G(:,2),G(:,3),'r.')
quiver(G(:,2),G(:,3),G(:,4),G(:,5),0.1)
colorbar
axis equal
axis xy;
title('domain ID')

subplot(2,3,5)
imagesc(CT(2:end,2:end)')
hold on
plot(G(:,2),G(:,3),'r.')
quiver(G(:,2),G(:,3),G(:,4),G(:,5),0.1)
colorbar
axis equal
axis xy;
title('CellType')


