% @ ���ߣ������3170101271
% @ ���һ���޸ģ�2019/10/24
%% ������
function AntPathPlanning()
figure(1)
% ȫ�ֱ�����ʼ��
% Ĭ�ϣ������ϰ�ģʽ������ڵ�ͼ���Ͻǣ��յ��ڵ�ͼ���½�
global obstacle
obstacle=[];

global width      % ��ͼ�߳�
global TotalTurns % ��������
global AntNumber  % ��������

global StartPoint
StartPoint=1;
global setstartpoint
setstartpoint=0;

global EndPoint
global setendpoint
setendpoint=0;

global setobstacle
setobstacle=1;

backcolor = get(gcf,'Color');
uicontrol(gcf,'style','text','units','normalized','pos',[0.22 0.9 0.56 0.08],...
    'string','��Ⱥ�㷨·���滮','fontsize',14,'fontweight','bold',...
    'fontunits','normalized','Hor','center','ForegroundColor',[0,0,0],...
    'bac',backcolor)
uicontrol(gcf,'style','push','units','normalized','pos',[0.05 0.02 0.127 0.08],'string','�������',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','set_startpoint')
uicontrol(gcf,'style','push','units','normalized','pos',[0.25 0.02 0.127 0.08],'string','�����յ�',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','set_endpoint')
uicontrol(gcf,'style','push','units','normalized','pos',[0.45 0.02 0.127 0.08],'string','�����ϰ�',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','set_obstacle')
uicontrol(gcf,'style','push','units','normalized','pos',[0.65 0.02 0.127 0.08],'string','�� ��',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','CLR')
uicontrol(gcf,'style','push','units','normalized','pos',[0.85 0.02 0.127 0.08],'string','�� ��',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','AntcolonyPathPlanning')

axes('Position',[0.17 0.18 0.67 0.68],'Box','on','ButtonDownFcn',{@MouseAction 'start'})

answer = inputdlg({'��ͼ�߳�','��������','��������'},...
              '�������', [1 50; 1 50; 1 50],{'20','20','80'}); 
width=str2double(answer{1});
TotalTurns=str2double(answer{2});
AntNumber=str2double(answer{3});
EndPoint=width*width;


axis([0 width 0 width])
axis square
set(gca,'XTick',0:1:width, 'YTick',0:1:width)
grid on
zoom off

end



