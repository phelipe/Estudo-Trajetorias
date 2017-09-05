module Trajetorias

export minimumjerk
export functionform

# Retorna os coeficientes do polinômio de trajetória com base no princípio do minimum jerk.
# Entradas
# x0 -> posição inicial
# v0 -> velocidade inicial
# a0 -> aceleração inicial
# xf -> posição final
# vf -> velocidade final
# af -> aceleração final
# T -> tempo para chegar ao ponto final
function minimumjerk(x0,v0,a0,xf,vf,af,T)
    T2 = T^2
    T3 = T^3
    T4 = T^4
    T5 = T^5
    a = zeros(6)
    a[1] = x0
    a[2] = v0
    a[3] = a0/2
    b = [T3 T4 T5; 3*T2 4*T3 5*T4; 6*T 12*T2 20*T3]
    c = [xf-a[1]-a[2]*T-a[3]*T2, vf-a[2]-2*a[3]*T, af-2*a[3] ]
    a[4:6] = pinv(b)*c
    return a
end


function functionform(a)
    return (t -> a[1] + a[2]*t + a[3]*t^2 + a[4]*t^3 + a[5]*t^4 + a[6]*t^5, t-> a[2] + a[3]*t^1 + a[4]*t^2 + a[5]*t^3 + a[6]*t^4, t-> a[3] + a[4]*t^1 + a[5]*t^2 + a[6]*t^3)
end

end
