% @ ���ߣ������3170101271
% @ ���һ���޸ģ�2019/10/24
%% ����¼��ص�����
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
        % �û��������ϰ�
        if setobstacle==1
            line([point_old_new(1) point(1,1)],[point_old_new(2) point(1,2)],'color','black',...
                'clipping','on','erasemode','background','linewidth',2);
            
            global obstacle
            obstacle=[obstacle;floor(point(1,1))+1,floor(point(1,2))+1]  % ��¼�������ϰ���
        % ������ʼ��   
        elseif setstartpoint==1
            line([point_old_new(1) point(1,1)],[point_old_new(2) point(1,2)],'color','green',...
                'clipping','on','erasemode','background','linewidth',2);
            global StartPoint
            StartPoint=[floor(point(1,1))+1,floor(point(1,2))+1]
        % �����յ�
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