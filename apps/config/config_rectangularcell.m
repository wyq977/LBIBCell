%% create config files for rectangular growing cell

clear all
close all
clc
fig=1;

%% parameters:
LX = 40; % domain size x
LY = 400; % domain size y

BX = 10; % box size x
BY = 10; % box size y

MIDX = LX/2; % midpoint of the box
MIDY = LX/2; % midpoint of the box

XRES = 2*BX; % resolution of x side
YRES = 2*BY; % resolution of y side

NODES = [];
CONNECTIONS = [];


%% create PhysicalNodes #Nodes (id xPos yPos)

xstart = MIDX-BX/2;
xend = MIDX+BX/2;
xstep = BX/XRES;

ystart = MIDY-BY/2;
yend = MIDY+BY/2;
ystep = BY/YRES;

ID = 1;

% bottom:
for i=1:1:XRES
    xpos = xstart + i*xstep;
    ypos = ystart;
    NODES(ID,:) = [ID xpos ypos];
    ID=ID+1;
end

% right:
for i=1:1:YRES
   xpos = xend;
   ypos = ystart + i*ystep;
   NODES(ID,:) = [ID xpos ypos];
   ID=ID+1;
end

% top:
for i=1:1:XRES
    xpos = xend - i*xstep;
    ypos = yend;
    NODES(ID,:) = [ID xpos ypos];
    ID=ID+1;
end

% left:
for i=1:1:YRES
   xpos = xstart;
   ypos = yend - i*ystep;
   NODES(ID,:) = [ID xpos ypos];
   ID=ID+1;
end




%% create Connections #Connection (nodeId1 nodeId2 domainId bsolver cdesolver ...)

solver{1} = 'BoundarySolverNoFluxD2Q5';
solver{2} = 'CDESolverD2Q5';

COUNT = 1;

for i=1:1:size(NODES,1)-1
    CONNECTIONS(COUNT,:) = [COUNT COUNT+1 1];
    COUNT=COUNT+1;
end
CONNECTIONS(COUNT,:) = [COUNT 1 1];



%% plot:
figure(fig);fig=fig+1;
plot(NODES(:,2),NODES(:,3),'rx')
%axis equal
axis([0 LX 0 LY])
grid on

% for i=1:1:size(CONNECTIONS)
%     idx1 = find(NODES(:,1)==CONNECTIONS(i,1))
%     idx2 = find(NODES(:,1)==CONNECTIONS(i,2))
% end



%% write to file:

a{1} = sprintf('#Nodes (id\txPos\tyPos)');
a{2} = sprintf('#Connection (nodeId1\tnodeId2\tdomainId\tbsolver\tcdesolver\t...)');
sprintf('#Nodes (i\txPos yPos)')

FID = fopen('geometry.txt', 'w');
fprintf(FID, '%s\n', a{1});

for i = 1:size(NODES,1)
    fprintf(FID,'%g\t',NODES(i,:));
    fprintf(FID,'\n');
end

fprintf(FID, '%s\n', a{2});

for i=1:1:size(CONNECTIONS,1)
    fprintf(FID,'%g\t',CONNECTIONS(i,:));
        fprintf(FID,'%s\t%s',solver{1},solver{2});
    fprintf(FID,'\n');

    
end

fclose(FID);














