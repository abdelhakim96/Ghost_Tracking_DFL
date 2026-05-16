function inspect_run(tag)
    S = load(['results_' tag '.mat']); out = S.out;
    t = out.t; s = out.state;
    fprintf('--- %s ---\n', tag);
    fprintf('final t: %.3fs (diverged=%d)\n', t(end), out.diverged);
    fprintf('FW final pos: %8.2f %8.2f %8.2f\n', s(end,18), s(end,19), s(end,20));
    fprintf('Drone final pos: %8.2f %8.2f %8.2f\n', s(end,1), s(end,2), s(end,3));
    fprintf('Drone final quat: %.3f %.3f %.3f %.3f\n', s(end,4:7));
    fprintf('FW final quat:    %.3f %.3f %.3f %.3f\n', s(end,24:27));
    fprintf('phi_g range:   %8.2f -> %8.2f deg\n', min(s(:,14))*180/pi, max(s(:,14))*180/pi);
    fprintf('theta_g range: %8.2f -> %8.2f deg\n', min(s(:,15))*180/pi, max(s(:,15))*180/pi);
    fprintf('drone roll/pitch/yaw final (ZYX): ');
    q = s(end,4:7); q = q/norm(q);
    [yaw, pitch, roll] = quat_to_ypr(q);
    fprintf('roll=%.1f pitch=%.1f yaw=%.1f deg\n', roll*180/pi, pitch*180/pi, yaw*180/pi);
    fprintf('FW roll/pitch/yaw final (ZYX):    ');
    qa = s(end,24:27); qa = qa/norm(qa);
    [yaw, pitch, roll] = quat_to_ypr(qa);
    fprintf('roll=%.1f pitch=%.1f yaw=%.1f deg\n', roll*180/pi, pitch*180/pi, yaw*180/pi);

    csc = abs(cos(s(:,14)).*cos(s(:,15)));
    i_sing = find(csc < 0.3, 1, 'first');
    if ~isempty(i_sing)
        fprintf('cosc dropped below 0.3 at t=%.3fs (phi_g=%.1f deg, theta_g=%.1f deg)\n', ...
            t(i_sing), s(i_sing,14)*180/pi, s(i_sing,15)*180/pi);
    else
        fprintf('cosc never dropped below 0.3 — gimbal stayed safely reachable.\n');
    end
    fprintf('First 10 sample times: %.3fs ... ', t(1));
    for k=2:min(10,length(t))
        fprintf('%.3fs ', t(k));
    end
    fprintf('\n');
end

function [yaw, pitch, roll] = quat_to_ypr(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    sinp = 2*(q0*q2 - q3*q1);
    if abs(sinp) >= 1
        pitch = sign(sinp)*pi/2;
    else
        pitch = asin(sinp);
    end
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    yaw  = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
end
