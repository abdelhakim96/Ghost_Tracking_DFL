function inspect_loop()
    load(fullfile('results','results_v3_loop.mat'));
    s = out.state; t = out.t;
    fprintf('Loop t_end=%.3fs samples=%d\n', t(end), length(t));
    n = length(t);
    pitch = zeros(n,1);
    for k = 1:n
        qa = s(k,25:28); qa = qa/(norm(qa)+1e-12);
        sinp = 2*(qa(1)*qa(3) - qa(4)*qa(2));
        pitch(k) = asin(max(-1,min(1,sinp)));
    end
    fprintf('FW pitch start = %.1f deg, end = %.1f deg\n', pitch(1)*180/pi, pitch(end)*180/pi);
    q_rate = s(:,30);
    fprintf('Body q-rate end = %.3f rad/s\n', q_rate(end));
    total_pitch = cumtrapz(t, q_rate);
    fprintf('Integrated body pitch rotation = %.1f deg (%.2f loops)\n', total_pitch(end)*180/pi, total_pitch(end)/(2*pi));
end
