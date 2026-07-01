function animate_maneuver(tag, varargin)
% animate_maneuver  Animated GIF of the 3-axis camera-pose emulation, with
% attitude and control-effort panels.
%
%   animate_maneuver('loop')                 whole-trajectory view
%   animate_maneuver('roll','mode','follow') vehicle close-up
%
% Layout:
%   Left   : 3D scene. Red STL fixed-wing flies the reference; blue quad glyph
%            tracks the camera position with its own (banked/inverted) attitude;
%            green boresight triad = gimbal camera frame q_M (x) q_G locked on q_A.
%   Right-top : drone body roll / pitch / yaw + tilt (body-z from vertical).
%   Right-mid : thrust / weight, with hover (1) and a feasibility limit (T/W=4).
%   Right-bot : body-rate magnitude, with the feasibility limit (1500 deg/s).
%             Shaded red bands mark physically-infeasible regions.
%
% Options: 'nframes'(80) 'dpi'(96) 'delay'(0.06) 'mode'('whole'|'follow')
%          'TWmax'(4) 'ratemax'(1500)

    p = inputParser;
    addParameter(p,'nframes',80); addParameter(p,'dpi',96);
    addParameter(p,'delay',0.06); addParameter(p,'mode','whole');
    addParameter(p,'TWmax',4);    addParameter(p,'ratemax',1500);
    parse(p,varargin{:});
    nframes=p.Results.nframes; dpi=p.Results.dpi; delay=p.Results.delay;
    follow=strcmpi(p.Results.mode,'follow'); TWmax=p.Results.TWmax; ratemax=p.Results.ratemax;

    here=fileparts(mfilename('fullpath')); repo=fileparts(here); addpath(here);
    S=load(fullfile(repo,'results',['results_v3_' tag '.mat'])); out=S.out;
    t=out.t; state=out.state; quad=state(:,1:18); fw=state(:,19:31); n=numel(t);

    qC=zeros(n,4); qA=zeros(n,4); qM=zeros(n,4);
    rpy=zeros(n,3); tilt=zeros(n,1); geo=zeros(n,1);
    for k=1:n
        q=quad(k,4:7).'; q=q/norm(q+1e-12); qM(k,:)=q.';
        phi=quad(k,14); th=quad(k,15); ps=quad(k,16);
        qGx=[cos(phi/2);sin(phi/2);0;0]; qGy=[cos(th/2);0;sin(th/2);0]; qGz=[cos(ps/2);0;0;sin(ps/2)];
        qG=quat_mul(quat_mul(qGx,qGy),qGz); qC(k,:)=quat_mul(q,qG).';
        qa=fw(k,7:10).'; qa=qa/norm(qa+1e-12); qA(k,:)=qa.';
        rpy(k,:)=q2rpy(q);
        R=quat_to_R(q); tilt(k)=acosd(max(-1,min(1,R(3,3))));
        qe=quat_mul(quat_conj(qC(k,:).'),qA(k,:).'); qe=qe/norm(qe+1e-12);
        geo(k)=2*acosd(min(1,abs(qe(1))));
    end
    thr = quad(:,17);  W = thr(1);  TW = thr/max(W,1e-9);    % thr(1) = m*g (hover)
    brate = vecnorm(quad(:,11:13),2,2)*180/pi;

    % net world acceleration a = R*[0;0;T]/m - g e3 (thrust + gravity) and the
    % world velocity, for the in-scene velocity (cyan) and accel (magenta)
    % arrows. When the body inverts, a points strongly DOWN -> the drone drops.
    gacc = 9.81; mq = W/gacc;
    aN = zeros(n,3); vW = quad(:,8:10);
    for k = 1:n
        Rk = quat_to_R(qM(k,:));
        aN(k,:) = (Rk*[0;0;thr(k)]/mq - [0;0;gacc]).';
    end
    aref = max(vecnorm(aN,2,2)) + 1e-9; vref = max(vecnorm(vW,2,2)) + 1e-9;

    Xq=[quad(:,1),quad(:,2),-quad(:,3)]; Xf=[fw(:,1),fw(:,2),-fw(:,3)];
    allp=[Xq;Xf]; span=max(allp)-min(allp)+1e-6; pad=0.12*span; lo=min(allp)-pad; hi=max(allp)+pad;
    scene=max(span);

    [F,V]=stlread(fullfile(repo,'CAD','aero.stl')); V=V-mean(V);
    V=V/max(max(V)-min(V))*(0.13*scene);
    Ryaw=[cos(-pi) -sin(-pi) 0; sin(-pi) cos(-pi) 0; 0 0 1]; V=(Ryaw*V')';
    gscale=0.05*scene;

    outdir=fullfile(repo,'plots','gifs'); if ~exist(outdir,'dir'), mkdir(outdir); end
    giffile=fullfile(outdir,[tag '.gif']);

    fig=figure('Visible','off','Color','w','Position',[0 0 1300 680]);
    idx=unique(round(linspace(1,n,nframes)));
    TWtop=2*TWmax; rtop=3*ratemax;   % fixed caps; spikes clip at ceiling (reads as off-scale/infeasible)
    for jj=1:numel(idx)
        i=idx(jj); clf(fig);

        % ---- 3D scene
        ax=axes('Parent',fig,'Position',[0.035 0.07 0.45 0.88]); hold(ax,'on');
        plot3(ax,Xf(:,1),Xf(:,2),Xf(:,3),':','Color',[1 0.6 0.6],'LineWidth',0.5);
        plot3(ax,Xf(1:i,1),Xf(1:i,2),Xf(1:i,3),'r-','LineWidth',2.0);
        plot3(ax,Xq(1:i,1),Xq(1:i,2),Xq(1:i,3),'b-','LineWidth',1.2);
        Rf=quat_to_R(qA(i,:)); Vw=(Rf*V')';
        Vp=[Vw(:,1)+fw(i,1),Vw(:,2)+fw(i,2),-(Vw(:,3)+fw(i,3))];
        patch(ax,'Faces',F,'Vertices',Vp,'FaceColor',[0.85 0.2 0.2],'EdgeColor','none','FaceAlpha',0.35,'FaceLighting','gouraud');
        RM=quat_to_R(qM(i,:)); RC=quat_to_R(qC(i,:)); RA=quat_to_R(qA(i,:));
        % --- drone (at its position): light airframe + body axes + motion arrows
        draw_quad(ax,Xq(i,:),RM,gscale*0.9);
        draw_frame(ax,Xq(i,:),RM,gscale*1.3,4.0);            % drone body axes (short, thick)
        vv=vW(i,:)/vref*gscale*2.0;
        quiver3(ax,Xq(i,1),Xq(i,2),Xq(i,3),vv(1),vv(2),-vv(3),0,'Color',[0 0.6 0.75],'LineWidth',2.0,'MaxHeadSize',0.6);
        aa=aN(i,:)/aref*gscale*2.4;
        quiver3(ax,Xq(i,1),Xq(i,2),Xq(i,3),aa(1),aa(2),-aa(3),0,'Color',[0.9 0 0.7],'LineWidth',2.2,'MaxHeadSize',0.6);
        % --- camera, offset BELOW the drone for clarity: camera axes + FOV
        camoff=Xq(i,:)-[0 0 3.0*gscale];
        plot3(ax,[Xq(i,1) camoff(1)],[Xq(i,2) camoff(2)],[Xq(i,3) camoff(3)],'--','Color',[.55 .55 .55],'LineWidth',1.0);
        draw_frame(ax,camoff,RC,gscale*1.3,4.0);             % camera axes (short, thick)
        draw_fov_rect(ax,camoff,RC,gscale*4.5,28,20,[0 0.7 0],0.16,'fill');         % multicopter camera FOV
        % --- fixed-wing camera FOV (at FW pose), red wireframe
        draw_fov_rect(ax,Xf(i,:),RA,gscale*4.5,28,20,[0.85 0.15 0.15],0,'wire');    % plane camera FOV
        hold(ax,'off'); grid(ax,'on'); box(ax,'on'); daspect(ax,[1 1 1]);
        if follow
            Wd=1.7*(0.13*scene); c=Xf(i,:);
            xlim(ax,[c(1)-Wd c(1)+Wd]); ylim(ax,[c(2)-Wd c(2)+Wd]); zlim(ax,[c(3)-Wd c(3)+Wd]);
        else
            xlim(ax,[lo(1) hi(1)]); ylim(ax,[lo(2) hi(2)]); zlim(ax,[lo(3) hi(3)]);
        end
        view(ax,40,18); xlabel(ax,'N (m)'); ylabel(ax,'E (m)'); zlabel(ax,'Alt (m)');
        title(ax,sprintf('%s   t = %.2f s',upper(tag),t(i)),'Interpreter','none');
        annotation(fig,'textbox',[0.025 0.955 0.50 0.04],'EdgeColor','none','FontSize',7, ...
            'Interpreter','tex','String',['RGB triads = drone-body axes (top) & camera axes (below).   ' ...
            '\color[rgb]{0.85,0.15,0.15}red box: plane FOV   \color[rgb]{0,0.7,0}green box: camera FOV   ' ...
            '\color[rgb]{0,0.6,0.75}cyan: velocity   \color[rgb]{0.9,0,0.7}magenta: net accel']);
        camlight(ax,'headlight'); lighting(ax,'gouraud');

        % ---- attitude panel
        axA=axes('Parent',fig,'Position',[0.565 0.70 0.40 0.25]); hold(axA,'on');
        plot(axA,t,rpy(:,1),'-','Color',[0.85 0.1 0.1],'LineWidth',1.0);
        plot(axA,t,rpy(:,2),'-','Color',[0.1 0.5 0.1],'LineWidth',1.0);
        plot(axA,t,rpy(:,3),'-','Color',[0.1 0.1 0.85],'LineWidth',1.0);
        plot(axA,t,tilt,'-','Color',[0.5 0.5 0.5],'LineWidth',1.0);
        cursor(axA,t(i)); grid(axA,'on'); box(axA,'on'); xlim(axA,[0 t(end)]); ylim(axA,[-185 185]);
        ylabel(axA,'angle (deg)');
        legend(axA,{'roll','pitch','yaw','tilt'},'Orientation','horizontal','Location','northoutside','Box','off','FontSize',7);
        title(axA,'drone body attitude','FontSize',9);

        % ---- thrust panel
        axB=axes('Parent',fig,'Position',[0.565 0.40 0.40 0.21]); hold(axB,'on');
        area(axB,[0 t(end)],[TWtop TWtop],TWmax,'FaceColor',[1 0.85 0.85],'EdgeColor','none','BaseValue',TWmax);
        plot(axB,[0 t(end)],[1 1],'k:','LineWidth',0.8);
        plot(axB,[0 t(end)],[TWmax TWmax],'r--','LineWidth',1.0);
        plot(axB,t,TW,'-','Color',[0 0 0.7],'LineWidth',1.4);
        cursor(axB,t(i)); plot(axB,t(i),TW(i),'ko','MarkerFaceColor',[0 0 0.7],'MarkerSize',5);
        grid(axB,'on'); box(axB,'on'); xlim(axB,[0 t(end)]); ylim(axB,[0 TWtop]);
        ylabel(axB,'thrust / weight'); title(axB,sprintf('thrust  (hover=1, limit=%g)',TWmax),'FontSize',9);

        % ---- body-rate panel (feasibility)
        axC=axes('Parent',fig,'Position',[0.565 0.085 0.40 0.21]); hold(axC,'on');
        area(axC,[0 t(end)],[rtop rtop],ratemax,'FaceColor',[1 0.85 0.85],'EdgeColor','none','BaseValue',ratemax);
        plot(axC,[0 t(end)],[ratemax ratemax],'r--','LineWidth',1.0);
        plot(axC,t,brate,'-','Color',[0.5 0 0.5],'LineWidth',1.4);
        cursor(axC,t(i)); plot(axC,t(i),brate(i),'ko','MarkerFaceColor',[0.5 0 0.5],'MarkerSize',5);
        grid(axC,'on'); box(axC,'on'); xlim(axC,[0 t(end)]); ylim(axC,[0 rtop]);
        xlabel(axC,'t (s)'); ylabel(axC,'body rate (deg/s)');
        title(axC,sprintf('control effort  (limit=%g deg/s)',ratemax),'FontSize',9);

        drawnow;
        frame=print(fig,'-RGBImage',sprintf('-r%d',dpi));
        [A,map]=rgb2ind(frame,256);
        if jj==1, imwrite(A,map,giffile,'gif','LoopCount',Inf,'DelayTime',delay);
        else, imwrite(A,map,giffile,'gif','WriteMode','append','DelayTime',delay); end
    end
    close(fig);
    fprintf('wrote %s (%d frames)\n',giffile,numel(idx));
end

% ---------------------------------------------------------------------
function cursor(ax,tt)
    yl=ylim(ax); plot(ax,[tt tt],yl,'-','Color',[0.4 0.4 0.4],'LineWidth',0.8,'HandleVisibility','off');
end

function rpy=q2rpy(q)
    q=q(:)/max(norm(q),1e-12); q0=q(1);q1=q(2);q2=q(3);q3=q(4);
    roll=atan2(2*(q0*q1+q2*q3),1-2*(q1^2+q2^2));
    sp=2*(q0*q2-q3*q1); if abs(sp)>=1, pit=sign(sp)*pi/2; else, pit=asin(sp); end
    yaw=atan2(2*(q0*q3+q1*q2),1-2*(q2^2+q3^2));
    rpy=[roll pit yaw]*180/pi;
end

function draw_quad(ax,pos,R,L)
    arms=[1 0 0;-1 0 0;0 1 0;0 -1 0].'*L; Wv=R*arms; Wp=[Wv(1,:);Wv(2,:);-Wv(3,:)];
    for c=1:4
        plot3(ax,pos(1)+[0 Wp(1,c)],pos(2)+[0 Wp(2,c)],pos(3)+[0 Wp(3,c)],'-','Color',[0.3 0.3 0.55],'LineWidth',1.4);
        plot3(ax,pos(1)+Wp(1,c),pos(2)+Wp(2,c),pos(3)+Wp(3,c),'o','Color',[0.3 0.3 0.55],'MarkerSize',5,'MarkerFaceColor',[0.6 0.6 0.85]);
    end
end

function draw_frame(ax,pos,R,L,lw)
    % short, thick body triad: x=red, y=green, z=blue (z = body up / thrust axis)
    cols=[0.85 0.1 0.1; 0.1 0.7 0.1; 0.1 0.1 0.95];
    for a=1:3
        v=R(:,a)*L; vp=[v(1);v(2);-v(3)];
        quiver3(ax,pos(1),pos(2),pos(3),vp(1),vp(2),vp(3),0,'Color',cols(a,:),'LineWidth',lw,'MaxHeadSize',1.0);
    end
end

function draw_fov_rect(ax,pos,R,L,hdeg,vdeg,col,alpha,mode)
    % Rectangular camera FOV frustum: apex at pos, optical axis = R(:,1)
    % (forward), image right = R(:,2), image up = R(:,3). Rolls with the frame.
    P=@(v)[v(1);v(2);-v(3)];
    fp=P(R(:,1)); rp=P(R(:,2)); upp=P(R(:,3));
    apex=pos(:); hw=L*tand(hdeg); vh=L*tand(vdeg);
    c1=apex+L*fp+hw*rp+vh*upp; c2=apex+L*fp-hw*rp+vh*upp;
    c3=apex+L*fp-hw*rp-vh*upp; c4=apex+L*fp+hw*rp-vh*upp;
    C=[c1 c2 c3 c4];
    if strcmp(mode,'fill')
        V=[apex.'; c1.'; c2.'; c3.'; c4.'];
        patch(ax,'Faces',[1 2 3;1 3 4;1 4 5;1 5 2],'Vertices',V,'FaceColor',col,'EdgeColor',col*0.6,'FaceAlpha',alpha,'LineWidth',0.8);
        patch(ax,'Faces',[1 2 3 4],'Vertices',C.','FaceColor',col,'EdgeColor',col*0.6,'FaceAlpha',alpha*0.6,'LineWidth',0.8);
    else
        for c=1:4, plot3(ax,[apex(1) C(1,c)],[apex(2) C(2,c)],[apex(3) C(3,c)],'-','Color',col,'LineWidth',1.8); end
        cc=[C C(:,1)]; plot3(ax,cc(1,:),cc(2,:),cc(3,:),'-','Color',col,'LineWidth',1.8);
    end
end

function draw_fov_cone(ax,pos,Rcam,L,halfdeg)
    % Camera field-of-view cone: apex at the camera (= drone CoM, lever arm 0),
    % axis along the camera forward boresight Rcam(:,1) (body-aligned camera).
    % Shows where the emulated camera looks; tracks the fixed-wing view even as
    % the drone body flips. NED -> plot frame negates z.
    bore=Rcam(:,1); bp=[bore(1);bore(2);-bore(3)]; bp=bp/norm(bp);
    tmp=[0;0;1]; if abs(dot(tmp,bp))>0.9, tmp=[1;0;0]; end
    e1=cross(bp,tmp); e1=e1/norm(e1); e2=cross(bp,e1); e2=e2/norm(e2);
    r=L*tand(halfdeg); N=22; th=linspace(0,2*pi,N);
    apex=pos(:); ring=apex+L*bp+r*(e1*cos(th)+e2*sin(th));   % 3xN
    V=[apex.'; ring.']; Fc=zeros(N-1,3);
    for j=1:N-1, Fc(j,:)=[1 j+1 j+2]; end
    patch(ax,'Faces',Fc,'Vertices',V,'FaceColor',[0.1 0.8 0.1],'EdgeColor','none','FaceAlpha',0.18);
    tip=apex+L*bp;
    plot3(ax,[apex(1) tip(1)],[apex(2) tip(2)],[apex(3) tip(3)],'-','Color',[0 0.65 0],'LineWidth',2.2);
    plot3(ax,ring(1,:),ring(2,:),ring(3,:),'-','Color',[0 0.6 0],'LineWidth',0.8);
    plot3(ax,[apex(1) ring(1,1)],[apex(2) ring(2,1)],[apex(3) ring(3,1)],'-','Color',[0 0.6 0],'LineWidth',0.8);
end

function R=quat_to_R(q)
    q=q(:)/max(norm(q),1e-12); q0=q(1);q1=q(2);q2=q(3);q3=q(4);
    R=[q0^2+q1^2-q2^2-q3^2,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2);
       2*(q1*q2+q0*q3),q0^2-q1^2+q2^2-q3^2,2*(q2*q3-q0*q1);
       2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0^2-q1^2-q2^2+q3^2];
end
