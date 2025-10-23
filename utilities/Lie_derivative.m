%% Lie derivative function

function l = Lie_derivative(h, x, f, k)
    if k == 0
        l = h;
    else
        h_new = Lie_derivative(h, x, f, k - 1);
        dh = sym(zeros(1, length(x)));
        for i = 1:length(x)
            dh(i) = diff(h_new, x(i));
        end
        l = dh * f;
    end
end
