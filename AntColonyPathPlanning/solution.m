% Function: Ant Colony Algorithm for path panning
% Author: LiHongxin, FuLian
% Version: 2.0
% Summary: This function is a MATLAB-based fulfillment of
%          Ant Colony Algorithm, a route-search algorithm.
%          This algorithm is inspired by the ants, who shows
%          some kind of Swarm intelligence in finding.
%
 
 
%% Parameters Declaration
clf reset 
clear clc
% Map, 1 for walls, 0 for available space.
Map=[0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 1 0 0 0 1 1 1 0 0 0 0 0 0 0 1 1 0 0;
    0 1 1 0 0 0 1 1 1 0 0 0 0 0 0 0 1 1 0 0;
    0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 1 1 0;
    0 0 0 0 0 0 1 1 1 0 0 0 0 0 1 1 0 0 0 0;
    0 1 1 1 0 0 1 1 1 0 0 0 0 0 1 1 0 0 0 0;
    0 1 1 1 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 1 1 0 0 1 1 1 0 1 1 1 1 0 0 0 0 0 0;
    0 1 1 1 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 0;
    1 1 1 1 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 0;
    1 1 1 1 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 1 1 0;
    0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 1 1 0;
    0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0;];
 

MapSize=size(Map,1);
% Tabu：存储了网格之间互信息素的地图，因为两两网格之间有不同的信息素值，所以信息素地图大小为(MapSize*MapSize,MapSize*MapSize).
Tabu=ones(MapSize*MapSize,MapSize*MapSize);
Tabu=8.*Tabu;
% 迭代轮数
TotalTurns=100;
% 蚂蚁数量
opt=130;
converge=[];

for p=1:length(opt)
AntNumber=opt(p)
% 蚂蚁在地图上的起、终点

% StartPoint=2*(MapSize);
% EndPoint=(MapSize-1)*(MapSize-1);
% EndPoint=(MapSize-2)*(MapSize)+16;
StartPoint=1;
EndPoint=400;
%-------------------- 蚁群算法三个主要参数 -------------
% 信息素强度
% 每一轮计算之前，信息素挥发而减少；计算过程中，每一只蚂蚁在它们所经过的路径上释放信息素，使信息素增加
PheImportance=1;
% 启发因子
HeuImportance=8;
% 信息素挥发程度, 
PheEvapoEffi=0.4 ;
% --------------------------------------------------------
% 一些辅助变量
Q=1;
Minkl=inf;
Mink=0;
Minl=0;
CostMap=GenerateCostMap(Map);
N=size(CostMap,1);
a=1;

% 计算终点在禁忌表中的位置，减0.5是为了使终点在网格正中心（每一格边长为1）
Ex=a*(mod(EndPoint,MapSize)-0.5);
if Ex==-0.5
    Ex=MapSize-0.5;
end
Ey=a*(MapSize+0.5-ceil(EndPoint/MapSize));
% 启发式地图矩阵（类似于人工势场），网格距离距离终点越近，其启发值越高
HeuristicMap=zeros(N);
for i=1:N
    ix=a*(mod(i,MapSize)-0.5);
    if ix==-0.5
        ix=MapSize-0.5;
    end
    iy=a*(MapSize+0.5-ceil(i/MapSize));
    if i~=EndPoint
        HeuristicMap(i)=1/((ix-Ex)^2+(iy-Ey)^2)^0.5;
    else
        HeuristicMap(i)=100;
    end
end
% 存储每一轮每一只蚂蚁的路径
ROUTES=cell(TotalTurns,AntNumber);
% 存储每一轮每一只蚂蚁的路径的长度
PathLength=zeros(TotalTurns,AntNumber);
 
%% 蚁群算法迭代开始
% K轮, M只蚂蚁
tic
for k=1:TotalTurns
    for m=1:AntNumber
        W=StartPoint;          
        Path=StartPoint;        
        Pathlen=0;       
        Taboo=ones(N);% Taboo记录蚂蚁不能走或已经走过的地方，0表示蚂蚁不会去的地方，其参数N=地图长*宽
        Taboo(StartPoint)=0;   % 起点已经探索过了
        DD=CostMap;          %Initialization of adjacency matrix
        
        % 寻找下一个要前往的网格
        NextPoint=DD(W,:); % NextPoint存储从当前网格到其他所有网格的的路径成本，若成本为0，表示不可通行；若成本非零，表示可通行
        ReachableIndex=find(NextPoint); % 用find函数寻找可通行点
        % 把可通行点中走过的点剔除掉
        for j=1:length(ReachableIndex)
            if Taboo(ReachableIndex(j))==0
                NextPoint(ReachableIndex(j))=0;
            end
        end
        ReachableIndex=find(NextPoint);
        Len_ReachableIndex=length(ReachableIndex);
        
        % 蚂蚁未到达终点，且未走进死路
        while W~=EndPoint&&Len_ReachableIndex>=1
            % 计算蚂蚁从当前网格前往各个备选网格的转移概率
            TransferProbability=zeros(Len_ReachableIndex);
            for i=1:Len_ReachableIndex
                TransferProbability(i)=(Tabu(W,ReachableIndex(i))^PheImportance)*((HeuristicMap(ReachableIndex(i)))^HeuImportance);
            end
            sumpp=sum(TransferProbability);
            TransferProbability=TransferProbability/sumpp;
            % 计算概率累计直方图，用于挑选路径
            Pcum(1)=TransferProbability(1);
            for i=2:Len_ReachableIndex
                Pcum(i)=Pcum(i-1)+TransferProbability(i);
            end
            % 采用轮盘赌方法选择下一条要走的路
            Select=find(Pcum>=rand);
            to_visit=ReachableIndex(Select(1));
            
            % 更新路径
            Path=[Path,to_visit];       
            Pathlen=Pathlen+DD(W,to_visit);   
            % 更新蚂蚁所在的网格
            W=to_visit;
            % 更新禁忌表和成本地图，记录蚂蚁走过的网格
            for kk=1:N
                if Taboo(kk)==0
                    DD(W,kk)=0;
                    DD(kk,W)=0;
                end
            end
            
            Taboo(W)=0;    
            NextPoint=DD(W,:);
            ReachableIndex=find(NextPoint);
            for j=1:length(ReachableIndex)
                if Taboo(ReachableIndex(j))==0
                    NextPoint(j)=0;
                end
            end
            ReachableIndex=find(NextPoint);
            Len_ReachableIndex=length(ReachableIndex);
        end
        
        %记录第k轮，第m只蚂蚁走过的路径
        ROUTES{k,m}=Path;
        if Path(end)==EndPoint
            PathLength(k,m)=Pathlen;
            if Pathlen<Minkl
                Mink=k;Minl=m;Minkl=Pathlen;
            end
        else
            PathLength(k,m)=0;
        end
    end
    
    % 根据蚂蚁走过的路径，更新地图上的信息素
    Delta_Tau=zeros(N,N);
    for m=1:AntNumber
        if PathLength(k,m)
            ROUT=ROUTES{k,m};
            TS=length(ROUT)-1;
            PL_km=PathLength(k,m);
            for s=1:TS
                x=ROUT(s);
                y=ROUT(s+1);
                Delta_Tau(x,y)=Delta_Tau(x,y)+Q/PL_km;
                Delta_Tau(y,x)=Delta_Tau(y,x)+Q/PL_km;
            end
        end
    end
    % 信息素挥发
    Tabu=(1-PheEvapoEffi).*Tabu+Delta_Tau;
end

t=toc
%% 画出最短路径
IsToDraw=1;
if IsToDraw==1
    minPL=zeros(TotalTurns);
    for i=1:TotalTurns
        PLK=PathLength(i,:);
        Nonzero=find(PLK);
        PLKPLK=PLK(Nonzero);
        minPL(i)=min(PLKPLK);
    end
    figure(1)
    plot(minPL);
    hold on
    grid on
    str=sprintf('路径长度随迭代次数的变化趋势(用时:%2fs)',t);
    title('路径长度随迭代次数的变化趋势');
    
    xlabel('迭代次数');
    ylabel('每一轮中最短路径的长度');
    figure(2)
    axis([0,MapSize,0,MapSize])
    for i=1:MapSize
        for j=1:MapSize
            if Map(i,j)==1
                x1=j-1;y1=MapSize-i;
                x2=j;y2=MapSize-i;
                x3=j;y3=MapSize-i+1;
                x4=j-1;y4=MapSize-i+1;
                fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.2,0.2,0.2]);
                hold on
            else
                x1=j-1;y1=MapSize-i;
                x2=j;y2=MapSize-i;
                x3=j;y3=MapSize-i+1;
                x4=j-1;y4=MapSize-i+1;
                fill([x1,x2,x3,x4],[y1,y2,y3,y4],[1,1,1]);
                hold on
            end
        end
    end
    hold on
    
    title('最短路径');
    xlabel('x');
    ylabel('y');
    ROUT=ROUTES{Mink,Minl};
    LENROUT=length(ROUT);
    Rx=ROUT;
    Ry=ROUT;
    for ii=1:LENROUT
        Rx(ii)=a*(mod(ROUT(ii),MapSize)-0.5);
        if Rx(ii)==-0.5
            Rx(ii)=MapSize-0.5;
        end
        Ry(ii)=a*(MapSize+0.5-ceil(ROUT(ii)/MapSize));
    end
   plot(Rx,Ry)
end
%% 画出每一轮中所有蚂蚁的路径
 
IsToDraw2=0;
if IsToDraw2==1
    figure(3)
    axis([0,MapSize,0,MapSize])
    for i=1:MapSize
        for j=1:MapSize
            if Map(i,j)==1
                x1=j-1;y1=MapSize-i;
                x2=j;y2=MapSize-i;
                x3=j;y3=MapSize-i+1;
                x4=j-1;y4=MapSize-i+1;
                fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.2,0.2,0.2]);
                hold on
            else
                x1=j-1;y1=MapSize-i;
                x2=j;y2=MapSize-i;
                x3=j;y3=MapSize-i+1;
                x4=j-1;y4=MapSize-i+1;
                fill([x1,x2,x3,x4],[y1,y2,y3,y4],[1,1,1]);
                hold on
            end
        end
    end
    for k=1:TotalTurns
        PLK=PathLength(k,:);
        minPLK=min(PLK);
        pos=find(PLK==minPLK);
        m=pos(1);
        ROUT=ROUTES{k,m};
        LENROUT=length(ROUT);
        Rx=ROUT;
        Ry=ROUT;
        for ii=1:LENROUT
            Rx(ii)=a*(mod(ROUT(ii),MapSize)-0.5);
            if Rx(ii)==-0.5
                Rx(ii)=MapSize-0.5;
            end
            Ry(ii)=a*(MapSize+0.5-ceil(ROUT(ii)/MapSize));
        end
        plot(Rx,Ry)
        hold on
    end
end
for temp=length(minPL)-1:-1:1
    if minPL(temp)~=minPL(end)
        converge=[converge,temp];
        break
    end
end
end

%% 给地图的相邻网格之间设置路径成本
% 输入：Map 设置了起终点和障碍物的网格化地图矩阵（尺寸：MapSize*MapSize）
% 返回值： 成本地图 （尺寸：（MapSize*MapSize）*（MapSize*MapSize)）
function CostMap=GenerateCostMap(Map)
len=size(Map,1);
% CostMap的第(a,b)个网格表示实际地图的第a个元素（从上到下从左到右）与第b个元素之间的路径成本
% 如果实际地图的网格(i, j)可通行，即Map(i,j)==0，则计算地图上所有其他网格（包括自己共len*len个）到该网格的欧氏距离
% 所以CostMap的尺寸为实际地图的平方
CostMap=zeros(len*len,len*len);
for i=1:len
    for j=1:len
        if Map(i,j)==0
            for m=1:len
                for n=1:len
                    % 给地图上可行区域设置路径成本
                    if Map(m,n)==0 % 0表示可行区域
                        im=abs(i-m);jn=abs(j-n);
                        if im+jn==1||(im==1&&jn==1)
                            CostMap((i-1)*len+j,(m-1)*len+n)=(im+jn)^0.5;% 欧式距离
                        end
                    end
                end
            end
        end
    end
end

end