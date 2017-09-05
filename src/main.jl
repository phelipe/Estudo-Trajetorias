push!(LOAD_PATH, pwd()*"/modules")

using Trajetorias
using PyPlot

tempo = 3
coeficientes = minimumjerk(0,0,0,20,0,0,tempo)
posicao, velocidade, aceleracao = functionform(coeficientes)

x = 0:0.1:tempo
y1 = map(posicao,x)
y2 = map(velocidade,x)
y3 = map(aceleracao,x)
close("all")
plot(x,y1)
xlabel("tempo")
ylabel("posição")
