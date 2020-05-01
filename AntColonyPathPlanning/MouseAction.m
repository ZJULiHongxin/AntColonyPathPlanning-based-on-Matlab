% @ 作者：李鸿鑫3170101271
% @ 最后一次修改：2019/10/24
%% 鼠标事件回调函数
function MouseAction(hObject, eventdata, action)
switch action
    case 'start'
        point = get(gca,'CurrentPoint');
        setappdata(gcf,'point_old_new',point(1,:));
        line(point(1,1),point(1,2),'clipping','on',...
            'erasemode','background','linewidth',2);
        set(gcbf,'WindowButtonMotionFcn',{@MouseAction 'move'});
        set(gcbf,'WindowButtonUpFcn',{@MouseAction 'stop'});   
        
    case 'move'
        point_old_new = getappdata(gcf,'point_old_new');
        point = get(gca,'CurrentPoint');
        
        setappdata(gcf,'point_old_new',point(1,:));
        
        global setstartpoint
        global setendpoint
        global setobstacle
        % 用画笔设置障碍
        if setobstacle==1
            line([point_old_new(1) point(1,1)],[point_old_new(2) point(1,2)],'color','black',...
                'clipping','on','erasemode','background','linewidth',2);
            
            global obstacle
            obstacle=[obstacle;floor(point(1,1))+1,floor(point(1,2))+1]  % 记录画出的障碍点
        % 设置起始点   
        elseif setstartpoint==1
            line([point_old_new(1) point(1,1)],[point_old_new(2) point(1,2)],'color','green',...
                'clipping','on','erasemode','background','linewidth',2);
            global StartPoint
            StartPoint=[floor(point(1,1))+1,floor(point(1,2))+1]
        % 设置终点
        elseif setendpoint==1
            line([point_old_new(1) point(1,1)],[point_old_new(2) point(1,2)],'color','red',...
                'clipping','on','erasemode','background','linewidth',2);
            global EndPoint
            EndPoint=[floor(point(1,1))+1,floor(point(1,2))+1]
        end
        
    case 'stop'
        set(gcbf,'WindowButtonMotionFcn','');
        set(gcbf,'WindowButtonUpFcn','');
end 
end