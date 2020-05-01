% @ 作者：李鸿鑫3170101271
% @ 最后一次修改：2019/10/24
%% 主函数
function AntPathPlanning()
figure(1)
% 全局变量初始化
% 默认：设置障碍模式，起点在地图左上角，终点在地图右下角
global obstacle
obstacle=[];

global width      % 地图边长
global TotalTurns % 迭代轮数
global AntNumber  % 蚂蚁数量

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
    'string','蚁群算法路径规划','fontsize',14,'fontweight','bold',...
    'fontunits','normalized','Hor','center','ForegroundColor',[0,0,0],...
    'bac',backcolor)
uicontrol(gcf,'style','push','units','normalized','pos',[0.05 0.02 0.127 0.08],'string','设置起点',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','set_startpoint')
uicontrol(gcf,'style','push','units','normalized','pos',[0.25 0.02 0.127 0.08],'string','设置终点',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','set_endpoint')
uicontrol(gcf,'style','push','units','normalized','pos',[0.45 0.02 0.127 0.08],'string','设置障碍',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','set_obstacle')
uicontrol(gcf,'style','push','units','normalized','pos',[0.65 0.02 0.127 0.08],'string','重 置',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','CLR')
uicontrol(gcf,'style','push','units','normalized','pos',[0.85 0.02 0.127 0.08],'string','求 解',...
    'fontsize',12,'fontweight','bold','fontunits','normalized','callback','AntcolonyPathPlanning')

axes('Position',[0.17 0.18 0.67 0.68],'Box','on','ButtonDownFcn',{@MouseAction 'start'})

answer = inputdlg({'地图边长','迭代次数','蚂蚁数量'},...
              '输入参数', [1 50; 1 50; 1 50],{'20','20','80'}); 
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



