push!(LOAD_PATH, pwd()*"/modules")

using Trajetorias
using PyPlot

tempo_final = 0.5
coeficientes = minimumjerk(0,0,0,0,10,0,0,tempo_final)
posicao, velocidade, aceleracao = functionform(coeficientes)

x = 0:0.01:tempo_final
y1 = map(posicao,x)
y2 = map(velocidade,x)
y3 = map(aceleracao,x)
close("all")
#plot(x,y1)
#plot(x,y2)
plot(x,y3)
xlabel("tempo")
ylabel("posição")
