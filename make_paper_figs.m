function make_paper_figs()
% make_paper_figs  Publication figures (PDF) from the governed (feasible) runs.
%   Outputs to ECC_2026/figures/results/:
%     track_<m>.pdf     position + camera-orientation error vs time
%     overlay_<m>.pdf   3D overlay: FW (transparent STL) + drone + camera frame
%     inputs_<m>.pdf    thrust/weight, body rate, gimbal rate vs feasibility
%     feasibility.pdf   peak T/W per maneuver: full speed vs governed
    here = fileparts(mfilename('fullpath')); cd(here); addpath('utilities');
    outdir = fullfile('ECC_2026','figures','results');
    if ~exist(outdir,'dir'), mkdir(outdir); end
    mq=5.0; Ix=0.15; Iy=0.15; Iz=0.25; g=9.81;            % 5 kg platform
    ratemax=600; gimbmax=400; TWmax=3;                     % feasibility limits

    track_set   = {'loop','roll'};
    overlay_set = {'loop','roll','splits','cuban8'};
    input_set   = {'loop','roll'};

    for f=1:numel(track_set)
        nm=track_set{f}; [t,quad,fw,n]=loadrun(nm);
        [~,~,geo]=camera_quats(quad,fw,n);
        perr=vecnorm(quad(:,1:3)-fw(:,1:3),2,2);
        fig=figure('Visible','off','Color','w','Position',[0 0 560 340]);
        subplot(2,1,1); plot(t,perr,'b-','LineWidth',1.6); grid on;
        ylabel('position err (m)'); title(sprintf('%s: camera-pose tracking',upper(nm)));
        subplot(2,1,2); semilogy(t,max(geo,1e-4),'-','Color',[0 0.55 0],'LineWidth',1.6); grid on;
        ylabel('orient. err (deg)'); xlabel('t (s)'); ylim([1e-4 10]);
        exportgraphics(fig,fullfile(outdir,['track_' nm '.pdf']),'ContentType','vector'); close(fig);
        fprintf('track_%s.pdf\n',nm);
    end

    % --- 3D overlays with transparent aircraft ----------------------------
    [F,V]=stlread(fullfile('CAD','aero.stl')); V=V-mean(V);
    for f=1:numel(overlay_set)
        nm=overlay_set{f}; [t,quad,fw,n]=loadrun(nm);
        [qC,qA,~]=camera_quats(quad,fw,n);
        Xf=[fw(:,1) fw(:,2) -fw(:,3)]; Xq=[quad(:,1) quad(:,2) -quad(:,3)];
        span=max([Xq;Xf])-min([Xq;Xf])+1e-6; scene=max(span);
        Vs=V/max(max(V)-min(V))*(0.10*scene);
        Ry=[cos(-pi) -sin(-pi) 0; sin(-pi) cos(-pi) 0; 0 0 1]; Vs=(Ry*Vs')'; gs=0.05*scene;
        fig=figure('Visible','off','Color','w','Position',[0 0 520 460]);
        ax=axes('Parent',fig); hold(ax,'on');
        plot3(ax,Xf(:,1),Xf(:,2),Xf(:,3),'r-','LineWidth',1.8);
        plot3(ax,Xq(:,1),Xq(:,2),Xq(:,3),'b--','LineWidth',1.2);
        idx=unique(round(linspace(1,n,7)));
        for i=idx
            Rf=q2R(qA(i,:)); Vw=(Rf*Vs')'; Vp=[Vw(:,1)+fw(i,1),Vw(:,2)+fw(i,2),-(Vw(:,3)+fw(i,3))];
            patch(ax,'Faces',F,'Vertices',Vp,'FaceColor',[0.85 0.2 0.2],'EdgeColor','none','FaceAlpha',0.18,'FaceLighting','gouraud');
            triad(ax,Xq(i,:),q2R(qC(i,:)),gs*1.3);   % camera frame (boresight green)
        end
        hold(ax,'off'); grid(ax,'on'); box(ax,'on'); daspect(ax,[1 1 1]);
        cen=(max([Xq;Xf])+min([Xq;Xf]))/2; hs=0.6*scene;
        xlim(ax,[cen(1)-hs cen(1)+hs]); ylim(ax,[cen(2)-hs cen(2)+hs]); zlim(ax,[cen(3)-hs cen(3)+hs]);
        view(ax,40,20);
        camlight(ax,'headlight'); lighting(ax,'gouraud');
        xlabel(ax,'North (m)'); ylabel(ax,'East (m)'); zlabel(ax,'Altitude (m)');
        legend(ax,{'fixed-wing camera path','multicopter camera path'},'Location','northeast','FontSize',8);
        title(ax,sprintf('%s: 3D camera-pose overlay',upper(nm)));
        exportgraphics(fig,fullfile(outdir,['overlay_' nm '.pdf']),'ContentType','image','Resolution',300); close(fig);
        fprintf('overlay_%s.pdf\n',nm);
    end

    % --- input / feasibility plots ----------------------------------------
    for f=1:numel(input_set)
        nm=input_set{f}; [t,quad,fw,n]=loadrun(nm); dt=mean(diff(t));
        TW=quad(:,17)/(mq*g);
        brate=vecnorm(quad(:,11:13),2,2)*180/pi;
        gang=quad(:,14:16); grate=max(abs(gradient(gang',dt)'),[],2)*180/pi;
        fig=figure('Visible','off','Color','w','Position',[0 0 560 460]);
        subplot(3,1,1); hold on; area([0 t(end)],[6 6],TWmax,'FaceColor',[1 .88 .88],'EdgeColor','none','BaseValue',TWmax);
        plot(t,TW,'b-','LineWidth',1.5); yline(TWmax,'r--'); yline(1,'k:'); grid on; ylim([0 6]);
        ylabel('thrust / weight'); title(sprintf('%s: control inputs vs feasibility limits',upper(nm)));
        subplot(3,1,2); hold on; area([0 t(end)],[1.5*ratemax 1.5*ratemax],ratemax,'FaceColor',[1 .88 .88],'EdgeColor','none','BaseValue',ratemax);
        plot(t,brate,'-','Color',[.5 0 .5],'LineWidth',1.5); yline(ratemax,'r--'); grid on; ylim([0 1.5*ratemax]);
        ylabel('body rate (deg/s)');
        subplot(3,1,3); hold on; area([0 t(end)],[1.5*gimbmax 1.5*gimbmax],gimbmax,'FaceColor',[1 .88 .88],'EdgeColor','none','BaseValue',gimbmax);
        plot(t,grate,'-','Color',[0 .55 0],'LineWidth',1.5); yline(gimbmax,'r--'); grid on; ylim([0 1.5*gimbmax]);
        ylabel('gimbal rate (deg/s)'); xlabel('t (s)');
        exportgraphics(fig,fullfile(outdir,['inputs_' nm '.pdf']),'ContentType','vector'); close(fig);
        fprintf('inputs_%s.pdf\n',nm);
    end

    % --- feasibility bar ---------------------------------------------------
    mans={'loop','roll','rollsoft','barrelroll','splits','cuban8'};
    tw_full=[4.0 8.4 8.3 8.4 33.5 38.8]; tw_gov=[3 3 3 3 3 3];
    fig=figure('Visible','off','Color','w','Position',[0 0 560 300]);
    b=bar([tw_full(:) tw_gov(:)]); b(1).FaceColor=[0.85 0.3 0.3]; b(2).FaceColor=[0.3 0.5 0.85];
    set(gca,'XTickLabel',mans,'YScale','log'); grid on; hold on; yline(3,'k--','T/W limit','LineWidth',1.2);
    ylabel('peak thrust / weight'); legend({'full speed','governed'},'Location','northwest');
    title('Feasibility: governor confines demand to T/W \leq 3');
    exportgraphics(fig,fullfile(outdir,'feasibility.pdf'),'ContentType','vector'); close(fig);
    fprintf('feasibility.pdf\n'); disp('=== FIGS DONE ===');
end

function [t,quad,fw,n]=loadrun(nm)
    S=load(fullfile('results',['results_v3_' nm '.mat'])); out=S.out;
    t=out.t; st=out.state; quad=st(:,1:18); fw=st(:,19:31); n=numel(t);
end
function [qC,qA,geo]=camera_quats(quad,fw,n)
    qC=zeros(n,4); qA=zeros(n,4); geo=zeros(n,1);
    for k=1:n
        qM=quad(k,4:7).'; qM=qM/norm(qM+1e-12);
        ph=quad(k,14); th=quad(k,15); ps=quad(k,16);
        qGx=[cos(ph/2);sin(ph/2);0;0]; qGy=[cos(th/2);0;sin(th/2);0]; qGz=[cos(ps/2);0;0;sin(ps/2)];
        qG=quat_mul(quat_mul(qGx,qGy),qGz); qC(k,:)=quat_mul(qM,qG).';
        qa=fw(k,7:10).'; qa=qa/norm(qa+1e-12); qA(k,:)=qa.';
        qe=quat_mul(quat_conj(qC(k,:).'),qA(k,:).'); qe=qe/norm(qe+1e-12);
        geo(k)=2*acosd(min(1,abs(qe(1))));
    end
end
function triad(ax,pos,R,L)
    cols=[0.9 0.3 0;0 0.6 0.6;0 0.75 0];
    for a=1:3, v=R(:,a)*L; vp=[v(1);v(2);-v(3)];
        quiver3(ax,pos(1),pos(2),pos(3),vp(1),vp(2),vp(3),0,'Color',cols(a,:),'LineWidth',2.0,'MaxHeadSize',0.7); end
end
function R=q2R(q)
    q=q(:)/max(norm(q),1e-12); q0=q(1);q1=q(2);q2=q(3);q3=q(4);
    R=[q0^2+q1^2-q2^2-q3^2,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2);
       2*(q1*q2+q0*q3),q0^2-q1^2+q2^2-q3^2,2*(q2*q3-q0*q1);
       2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0^2-q1^2-q2^2+q3^2];
end
