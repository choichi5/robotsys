
n   = size(Q);
n(1);

aviobj = avifile('mymovie.avi','fps',50);      % 関数 avifile は、AVIフ?@イルを作成し、AVIフ?@イルオブジェクトに対するハンドルを出力します
                                               % frames per second：fps 
%  aviobj = avifile('mymovie.avi'); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:n(1)                                    % サンプルコ?[ドは、???[ビ?[に含まれる一連のグラフをキャプ?`ャ?[するために、for ル?[プを使います。
                                                % AVI???[ビ?[で一連のグラフをキャプ?`ャ?[するには、通例、関数addframeを使用します。
                                                % しかし、この特別なMATLABアニメ?[ションは、XOR グラフックスを使っているので、先ずグラフをキャプ?`ャ?[する関数getframeを呼び出し、
                                                % その後に関数addframe を使って、キャプ?`ャ?[したフレ?[??を???[ビ?[に追加する作業を行う必要があります

	i;
    x11     = Link11*cos( Q(i,1) );
    y11     = Link11*sin( Q(i,1) );
    x12     = Link11*cos( Q(i,1) )+Link12*cos( Q(i,1)+Q(i,2) );
    y12     = Link11*sin( Q(i,1) )+Link12*sin( Q(i,1)+Q(i,2) );
    %x13     = Link11*cos( q(i,1) )+Link12*cos( q(i,1)+q(i,2) )+Link13*cos( q(i,1)+q(i,2)+q(i,3) );
    %y13     = Link11*sin( q(i,1) )+Link12*sin( q(i,1)+q(i,2) )+Link13*sin( q(i,1)+q(i,2)+q(i,3) );
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    plot([x12 x11],[y12 y11],'k','LineWidth',2.);
    hold on
    plot([x11 0],[y11 0],'k','LineWidth',2.);
    hold on
    plot([-0.5 -0.5],[2.5 0],'k','LineWidth',3.);
    hold on
    plot([-0.5 -1],[0.25 -0.25],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[0.5 0],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[0.75 0.25],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[1 0.5],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[1.25 0.75],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[1.5 1],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[1.75 1.25],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[2 1.5],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[2.25 1.75],'k','LineWidth',1.);
    hold on
    plot([-0.5 -1],[2.5 2],'k','LineWidth',1.);
    hold on
    plot([x12],[y12],'ko');
    hold on
    plot([x11],[y11],'ko');
    hold on
    plot([0],[0],'ko');
    hold on
    %plot([obj1_x obj2_x obj3_x obj4_x obj1_x], [obj1_y obj2_y  obj3_y obj4_y obj1_y],'k','LineWidth',2.5);
    %plot([x01_d x02_d],[y01_d y02_d],'--k','LineWidth',2.);

	axis ([-1.5 1.5 0.0 3.0]);
    axis square
    
	grid on;

    frame = getframe(gca); 
    
    aviobj = addframe(aviobj,frame); 
	
    hold off
    
end
aviobj = close(aviobj);
