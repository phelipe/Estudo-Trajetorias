# aqui vou fazer o mesmo procedimento para simulação porém montando a equação para um pendulo simples
#
#
using DifferentialEquations
using Plots
pyplot()
type InputRobot{T} <: DEDataVector{T}
    x::Array{T,1}
    tau::Array{T,1}
end

#=
Função que realiza a simulação
entradas:
kp -  ganho proporcional
kv - ganho derivativo
r  - referência
tend - tempo de simulação
x_0 - vetor de posições iniciais
=#
function simulation(kp::Float64,kv::Float64,r::Float64,tend::Float64,x_0::Vector{Float64})
    t0 = 0.0
    v_0 = [0.,0.]
    Ts = 0.02

    #função contendo o sistema, robô sendo utilizado
    function robot(du,u,p,t)
        m1 = 0.2866
        L0 = 0.201
        L1 = 0.30997
        l1 = 0.154985
        I0 = 0.0052
        I1 = 0.0023
        g = 9.81


        θ = u[1:2]
        dθ = u[3:4]
        M = [I0 + m1*(L0^2)+(l1^2)*m1*(sin(θ[2])^2) L0*l1*m1*cos(θ[2]);
        L0*l1*m1*cos(θ[2]) I1+m1*(l1^2)]

        C = [2*(l1^2)*m1*sin(θ[2])*cos(θ[2])*dθ[2]  -L0*l1*m1*sin(θ[2])*dθ[2];
        -(l1)^2*m1*sin(θ[2])*cos(θ[2])*dθ[1]  0]

        G = [0;
        -g*l1*m1*sin(θ[2])]
        du[1:2] = dθ
        du[3:4] = inv(M)*(u.tau - C*dθ -G)
    end

    function controlador(integrator)
        for c in user_cache(integrator)
            q = c.x[1:2]
            dotq = c.x[3:4]
            e = r - q[2]
            #println(integrator.t)
            #println(e)
            c.tau =[(kp*e - kv*dotq[2]), 0]
        end
    end

    cbs = PeriodicCallback(controlador,Ts)
    tspan = (t0,tend)
    start = InputRobot(vcat(x_0,v_0), zeros(7))
    prob = ODEProblem(robot,start,tspan)
    sol = solve(prob,Tsit5(),callback = cbs,saveat = Ts/2)
    out_x = map(x -> x[1:2],sol.u) #posição
    out_dx = map(x -> x[3:end],sol.u)
    #velocidade
    #Aqui estou fazendo uma aproximação da aceleração e do jerk
    #out_d2x = diff(out_dx)/Ts #aceleração
    #out_d3x = diff(out_d2x)/Ts #jerk

    θ1 = map(x -> x[1],out_x)
    θ2 = map(x -> x[2],out_x)
    ω1 = map(x -> x[1],out_dx)
    ω2 = map(x -> x[2],out_dx)
    #α1 = map(x -> x[1],out_d2x)
    #α2 = map(x -> x[2],out_d2x)
    #𝚥1 = map(x -> x[1],out_d3x)
    #𝚥2 = map(x -> x[2],out_d3x)
    [θ1, θ2], [ω1, ω2], sol.t
end

function plotdeg(solution)
    p1 = plot(solution[3],rad2deg.(solution[1][1]), xlabel = "tempo (s)", ylabel = "Posição braço (graus)")
    p2 = plot(solution[3],rad2deg.(solution[1][2]), xlabel = "tempo (s)", ylabel = "Posição pêndulo (graus)")
    plot(p1,p2)
end

function plotrad(solution)
    p1 = plot(solution[3],solution[1][1], xlabel = "tempo (s)", ylabel = "Posição braço (rad)")
    p2 = plot(solution[3],solution[1][2], xlabel = "tempo (s)", ylabel = "Posição pêndulo (rad)")
    plot(p1,p2)
end

out = simulation(10.,0.,0.,2.,[0., 0.00001]);


plotdeg(out)
