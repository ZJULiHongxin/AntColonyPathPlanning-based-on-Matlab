% @ 作者：李鸿鑫3170101271
% @ 最后一次修改：2019/10/24
%% 蚁群算法求解函数
function AntcolonyPathPlanning()
figure(1)
title('求解中...')
global width      % 地图边长
global TotalTurns % 迭代轮数
global AntNumber  % 蚂蚁数量
% 栅格化地图, 1为障碍, 0为可行空间
Map=zeros(width,width);
global obstacle
global StartPoint
global EndPoint
obstacle=unique(obstacle,'row'); % 删除重复的障碍物位置

for i=1:size(obstacle,1) % 把障碍物坐标中和起终点重合的点都去掉
    if (obstacle(i,1)==StartPoint(1) && obstacle(i,2)==StartPoint(2))||(obstacle(i,1)==EndPoint(1) && obstacle(i,2)==EndPoint(2))
        obstacle(i,:)=[];
    end
end

% 初始化地图网格，1表示障碍，0表示可行区域
for i=1:size(obstacle,1) 
    Map(width-obstacle(i,2)+1,obstacle(i,1))=1;
end


MapSize=size(Map,1);
% Tabu：存储了网格之间互信息素的地图，因为两两网格之间有不同的信息素值，所以信息素地图大小为(MapSize*MapSize,MapSize*MapSize).
Tabu=ones(MapSize*MapSize,MapSize*MapSize);
Tabu=8.*Tabu;

% 蚂蚁在地图上的起、终点
SPoint=(MapSize-StartPoint(2))*MapSize+StartPoint(1);
EPoint=(MapSize-EndPoint(2))*MapSize+EndPoint(1);

% 测试用------
% Map=[0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0;
%     0 1 1 0 0 0 1 1 1 0 0 0 0 0 0 0 1 1 0 0;
%     0 1 1 0 0 0 1 1 1 0 0 0 0 0 0 0 1 1 0 0;
%     0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 1 1 0;
%     0 0 0 0 0 0 1 1 1 0 0 0 0 0 1 1 0 0 0 0;
%     0 1 1 1 0 0 1 1 1 0 0 0 0 0 1 1 0 0 0 0;
%     0 1 1 1 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0;
%     0 1 1 1 0 0 1 1 1 0 1 1 1 1 0 0 0 0 0 0;
%     0 1 1 1 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 0;
%     0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 0;
%     1 1 1 1 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 0;
%     1 1 1 1 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 1 1 0;
%     0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 1 1 0;
%     0 0 0 0 0 0 0 0 0 0 1 1 0 0 1 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0;];
% 
% % Map=zeros(20);
% SPoint=1;
% EPoint=MapSize*MapSize;
% 以上是测试用地图------
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
Ex=a*(mod(EPoint,MapSize)-0.5);
if Ex==-0.5
    Ex=MapSize-0.5;
end
Ey=a*(MapSize+0.5-ceil(EPoint/MapSize));
% 启发式地图矩阵（类似于人工势场），网格距离距离终点越近，其启发值越高
HeuristicMap=zeros(N);
for i=1:N
    ix=a*(mod(i,MapSize)-0.5);
    if ix==-0.5
        ix=MapSize-0.5;
    end
    iy=a*(MapSize+0.5-ceil(i/MapSize));
    if i~=EPoint
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
        W=SPoint;          
        Path=SPoint;        
        Pathlen=0;       
        Taboo=ones(N);% Taboo记录蚂蚁不能走或已经走过的地方，0表示蚂蚁不会去的地方，其参数N=地图长*宽
        Taboo(SPoint)=0;   % 起点已经探索过了
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
        while W~=EPoint&&Len_ReachableIndex>=1
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
        if Path(end)==EPoint
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
    figure(2)
    cla
    plot(minPL);
    hold on
    grid on
    str=sprintf('路径长度随迭代次数的变化趋势(用时:%2fs)',t);
    title(str);
    xlabel('迭代次数');
    ylabel('每一轮中最短路径的长度');
    figure(1)
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
    
    % 画出起点
    hold on
    x1=StartPoint(1)-1;y1=StartPoint(2)-1;
    x2=StartPoint(1);y2=StartPoint(2)-1;
    x3=StartPoint(1);y3=StartPoint(2);
    x4=StartPoint(1)-1;y4=StartPoint(2);
    fill([x1,x2,x3,x4],[y1,y2,y3,y4],'g');
    [x1,x2,x3,x4]
    [y1,y2,y3,y4]
    % 画出终点
    x1=EndPoint(1)-1;y1=EndPoint(2)-1;
    x2=EndPoint(1);y2=EndPoint(2)-1;
    x3=EndPoint(1);y3=EndPoint(2);
    x4=EndPoint(1)-1;y4=EndPoint(2);
    fill([x1,x2,x3,x4],[y1,y2,y3,y4],'r');
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
IsToDraw2=1;
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
    % 画出起点
    hold on
    x1=StartPoint(1)-1;y1=StartPoint(2)-1;
    x2=StartPoint(1);y2=StartPoint(2)-1;
    x3=StartPoint(1);y3=StartPoint(2);
    x4=StartPoint(1)-1;y4=StartPoint(2);
    fill([x1,x2,x3,x4],[y1,y2,y3,y4],'g');
    [x1,x2,x3,x4]
    [y1,y2,y3,y4]
    % 画出终点
    x1=EndPoint(1)-1;y1=EndPoint(2)-1;
    x2=EndPoint(1);y2=EndPoint(2)-1;
    x3=EndPoint(1);y3=EndPoint(2);
    x4=EndPoint(1)-1;y4=EndPoint(2);
    fill([x1,x2,x3,x4],[y1,y2,y3,y4],'r');
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
        title('最后一次迭代所有蚂蚁的路径')
        hold on
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
                        dx=i-m; dy=j-n;
                        im=abs(dx);jn=abs(dy);
                        if im+jn==1||(im==1&&jn==1)
                            if Map(m+dx,n)==0 || Map(m,n+dy)==0 % 防止路径从障碍物之间的角落穿过去
                                CostMap((i-1)*len+j,(m-1)*len+n)=(im+jn)^0.5;% 欧式距离
                            end
                        end
                    end
                end
            end
        end
    end
end
end
