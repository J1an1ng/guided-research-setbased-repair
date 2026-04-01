function y = scaled_tanh(x, alpha_pos, alpha_neg, beta)
    y = alpha_pos * (1 + tanh(beta * x)) / 2 .* tanh(x) + ...
        alpha_neg * (1 - tanh(beta * x)) / 2 .* tanh(x);
end
