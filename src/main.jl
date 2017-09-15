push!(LOAD_PATH, pwd()*"/modules")

using Trajetorias
using Plots
pyplot()

tempo_final = 12
coeficientes = minimumjerk(0,0,0,0,10,0,0,tempo_final)
posicao, velocidade, aceleracao = functionform(coeficientes)
time = 0:0.01:tempo_final
p1 = plot(time, map(posicao,time), xlabel = "tempo", ylabel ="posição")
p2 = plot(time, map(velocidade,time), xlabel = "tempo", ylabel ="velocidade")
p3 = plot(time, map(aceleracao,time), xlabel = "tempo", ylabel ="aceleração")
plot(p1, p2, p3, layout=(3,1))
