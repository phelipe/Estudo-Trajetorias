module Trajetorias

export minimumjerk
export functionform

# Retorna os coeficientes do polinômio de trajetória com base no princípio do minimum jerk considerando o tempo inicial sendo 0(zero).
# Entradas
# x0 -> posição inicial
# v0 -> velocidade inicial
# a0 -> aceleração inicial
# xf -> posição final
# vf -> velocidade final
# af -> aceleração final
# T -> tempo para chegar ao ponto final
function minimumjerk(x0,v0,a0,xf,vf,af,T)
    const t0 = 0
    const tf = T
    b=[ 1 t0 t0^2 t0^3 t0^4 t0^5;
    0 1 2*t0 3*(t0^2) 4*(t0^3) 5*(t0^4);
    0 0 2 6*t0 12*(t0^2) 20*(t0^3);
    1 tf tf^2 tf^3 tf^4 tf^5;
    0 1 2*tf 3*(tf^2) 4*(tf^3) 5*(tf^4);
    0 0 2 6*tf 12*(tf^2) 20*(tf^3)]
    c = [x0, v0, a0, xf, vf, af]
    a = inv(b)*c
    return a
end

# Retorna os coeficientes do polinômio de trajetória com base no princípio do minimum jerk considerando o tempo inicial sendo 0(zero).
# Entradas
# x0 -> posição inicial
# v0 -> velocidade inicial
# a0 -> aceleração inicial
# t0 -> tempo inicial
# xf -> posição final
# vf -> velocidade final
# af -> aceleração final
# tf -> tempo para chegar ao ponto final
function minimumjerk(x0,v0,a0,t0,xf,vf,af,tf)
    b=[ 1 t0 t0^2 t0^3 t0^4 t0^5;
    0 1 2*t0 3*(t0^2) 4*(t0^3) 5*(t0^4);
    0 0 2 6*t0 12*(t0^2) 20*(t0^3);
    1 tf tf^2 tf^3 tf^4 tf^5;
    0 1 2*tf 3*(tf^2) 4*(tf^3) 5*(tf^4);
    0 0 2 6*tf 12*(tf^2) 20*(tf^3)]
    c = [x0, v0, a0, xf, vf, af]
    a = inv(b)*c
    return a
end

# Retorna as funções de posição, velocidade e aceleração a parti dos coeficientes do polinômio do mínimo jerk
# Entradas
# a -> vetor com os coeficientes do polinômio
function functionform(a::Vector)
    return (t -> a[1] + a[2]*t + a[3]*t^2 + a[4]*t^3 + a[5]*t^4 + a[6]*t^5, t-> a[2] + 2*a[3]*t + 3*a[4]*(t^2) + 4*a[5]*(t^3) + 5*a[6]*(t^4), t-> 2*a[3] + 6*a[4]*t + 12*a[5]*(t^2) + 20*a[6]*(t^3))
end


# TODO: fazer a geração de trajetórias pelo método tradicional do livro do craig

end
