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
kp - vetor de ganhos proporcionais
kv - vetor de ganhos derivativos
r  - vetor de referência
saídas :(as que irei precisar)
e - somatório do erro de posição
J - somatório do jerk
=#
function simulation(kp,kv,r,tend)
    t0 = 0.
    x_0 = [0.,0.]
    v_0 = [0.,0.]
    if length(kp) != 2
        error("O vetor de ganhos Kp deve ter mesma dimensão que a quantidade de juntas do robô ($(size))")
    elseif length(kv) != 2
        error("O vetor de ganhos Kv deve ter mesma dimensão que a quantidade de juntas do robô ($(size))")
    elseif length(r) != 2
        error("O vetor de referência r deve ter mesma dimensão que a quantidade de juntas do robô ($(size))")
    end
    KP = diagm(kp)
    KV = diagm(kv)
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
            e = r - q
            #println(integrator.t)
            #println(dotq)
            c.tau = KP*e - KV*dotq
            println(c.tau)
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
    out_d2x = diff(out_dx)/Ts #aceleração
    out_d3x = diff(out_d2x)/Ts #jerk

    out_x, out_dx, out_d2x, out_d3x, sol.t
end

θ, ω, α, 𝚥, t = simulation([0., 0.],[0.,0.],[1.,1.],2);

θ1 = map(x -> x[1],θ)
θ2 = map(x -> x[2],θ)
ω1 = map(x -> x[1],ω)
ω2 = map(x -> x[2],ω)

plot(t,teta2, xlabel = "tempo (s)", ylabel = "Posição (rad)", title="Simulação pêndulo simples")
