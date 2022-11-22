function alpha = Corrigir(alpha)
alpha = mod(alpha,360);

if alpha > 180 
    alpha = alpha-360;
end

end