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
% Tabu���洢������֮�以��Ϣ�صĵ�ͼ����Ϊ��������֮���в�ͬ����Ϣ��ֵ��������Ϣ�ص�ͼ��СΪ(MapSize*MapSize,MapSize*MapSize).
Tabu=ones(MapSize*MapSize,MapSize*MapSize);
Tabu=8.*Tabu;
% ��������
TotalTurns=100;
% ��������
opt=130;
converge=[];

for p=1:length(opt)
AntNumber=opt(p)
% �����ڵ�ͼ�ϵ����յ�

% StartPoint=2*(MapSize);
% EndPoint=(MapSize-1)*(MapSize-1);
% EndPoint=(MapSize-2)*(MapSize)+16;
StartPoint=1;
EndPoint=400;
%-------------------- ��Ⱥ�㷨������Ҫ���� -------------
% ��Ϣ��ǿ��
% ÿһ�ּ���֮ǰ����Ϣ�ػӷ������٣���������У�ÿһֻ������������������·�����ͷ���Ϣ�أ�ʹ��Ϣ������
PheImportance=1;
% ��������
HeuImportance=8;
% ��Ϣ�ػӷ��̶�, 
PheEvapoEffi=0.4 ;
% --------------------------------------------------------
% һЩ��������
Q=1;
Minkl=inf;
Mink=0;
Minl=0;
CostMap=GenerateCostMap(Map);
N=size(CostMap,1);
a=1;

% �����յ��ڽ��ɱ��е�λ�ã���0.5��Ϊ��ʹ�յ������������ģ�ÿһ��߳�Ϊ1��
Ex=a*(mod(EndPoint,MapSize)-0.5);
if Ex==-0.5
    Ex=MapSize-0.5;
end
Ey=a*(MapSize+0.5-ceil(EndPoint/MapSize));
% ����ʽ��ͼ�����������˹��Ƴ����������������յ�Խ����������ֵԽ��
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
% �洢ÿһ��ÿһֻ���ϵ�·��
ROUTES=cell(TotalTurns,AntNumber);
% �洢ÿһ��ÿһֻ���ϵ�·���ĳ���
PathLength=zeros(TotalTurns,AntNumber);
 
%% ��Ⱥ�㷨������ʼ
% K��, Mֻ����
tic
for k=1:TotalTurns
    for m=1:AntNumber
        W=StartPoint;          
        Path=StartPoint;        
        Pathlen=0;       
        Taboo=ones(N);% Taboo��¼���ϲ����߻��Ѿ��߹��ĵط���0��ʾ���ϲ���ȥ�ĵط��������N=��ͼ��*��
        Taboo(StartPoint)=0;   % ����Ѿ�̽������
        DD=CostMap;          %Initialization of adjacency matrix
        
        % Ѱ����һ��Ҫǰ��������
        NextPoint=DD(W,:); % NextPoint�洢�ӵ�ǰ����������������ĵ�·���ɱ������ɱ�Ϊ0����ʾ����ͨ�У����ɱ����㣬��ʾ��ͨ��
        ReachableIndex=find(NextPoint); % ��find����Ѱ�ҿ�ͨ�е�
        % �ѿ�ͨ�е����߹��ĵ��޳���
        for j=1:length(ReachableIndex)
            if Taboo(ReachableIndex(j))==0
                NextPoint(ReachableIndex(j))=0;
            end
        end
        ReachableIndex=find(NextPoint);
        Len_ReachableIndex=length(ReachableIndex);
        
        % ����δ�����յ㣬��δ�߽���·
        while W~=EndPoint&&Len_ReachableIndex>=1
            % �������ϴӵ�ǰ����ǰ��������ѡ�����ת�Ƹ���
            TransferProbability=zeros(Len_ReachableIndex);
            for i=1:Len_ReachableIndex
                TransferProbability(i)=(Tabu(W,ReachableIndex(i))^PheImportance)*((HeuristicMap(ReachableIndex(i)))^HeuImportance);
            end
            sumpp=sum(TransferProbability);
            TransferProbability=TransferProbability/sumpp;
            % ��������ۼ�ֱ��ͼ��������ѡ·��
            Pcum(1)=TransferProbability(1);
            for i=2:Len_ReachableIndex
                Pcum(i)=Pcum(i-1)+TransferProbability(i);
            end
            % �������̶ķ���ѡ����һ��Ҫ�ߵ�·
            Select=find(Pcum>=rand);
            to_visit=ReachableIndex(Select(1));
            
            % ����·��
            Path=[Path,to_visit];       
            Pathlen=Pathlen+DD(W,to_visit);   
            % �����������ڵ�����
            W=to_visit;
            % ���½��ɱ�ͳɱ���ͼ����¼�����߹�������
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
        
        %��¼��k�֣���mֻ�����߹���·��
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
    
    % ���������߹���·�������µ�ͼ�ϵ���Ϣ��
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
    % ��Ϣ�ػӷ�
    Tabu=(1-PheEvapoEffi).*Tabu+Delta_Tau;
end

t=toc
%% �������·��
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
    str=sprintf('·����������������ı仯����(��ʱ:%2fs)',t);
    title('·����������������ı仯����');
    
    xlabel('��������');
    ylabel('ÿһ�������·���ĳ���');
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
    
    title('���·��');
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
%% ����ÿһ�����������ϵ�·��
 
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

%% ����ͼ����������֮������·���ɱ�
% ���룺Map ���������յ���ϰ�������񻯵�ͼ���󣨳ߴ磺MapSize*MapSize��
% ����ֵ�� �ɱ���ͼ ���ߴ磺��MapSize*MapSize��*��MapSize*MapSize)��
function CostMap=GenerateCostMap(Map)
len=size(Map,1);
% CostMap�ĵ�(a,b)�������ʾʵ�ʵ�ͼ�ĵ�a��Ԫ�أ����ϵ��´����ң����b��Ԫ��֮���·���ɱ�
% ���ʵ�ʵ�ͼ������(i, j)��ͨ�У���Map(i,j)==0��������ͼ�������������񣨰����Լ���len*len�������������ŷ�Ͼ���
% ����CostMap�ĳߴ�Ϊʵ�ʵ�ͼ��ƽ��
CostMap=zeros(len*len,len*len);
for i=1:len
    for j=1:len
        if Map(i,j)==0
            for m=1:len
                for n=1:len
                    % ����ͼ�Ͽ�����������·���ɱ�
                    if Map(m,n)==0 % 0��ʾ��������
                        im=abs(i-m);jn=abs(j-n);
                        if im+jn==1||(im==1&&jn==1)
                            CostMap((i-1)*len+j,(m-1)*len+n)=(im+jn)^0.5;% ŷʽ����
                        end
                    end
                end
            end
        end
    end
end

end