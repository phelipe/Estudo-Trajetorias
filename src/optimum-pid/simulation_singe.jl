# aqui vou fazer o mesmo procedimento para simulação porém montando a euqação para um pendulo simples
#
#

#NOTE: tem que modificar tudo aqui para uma simulação de um pendulo simles

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
function simulation(kp,kv,r)
    if length(kp) != size
        error("O vetor de ganhos Kp deve ter mesma dimensão que a quantidade de juntas do robô ($(size))")
    elseif length(kv) != size
        error("O vetor de ganhos Kv deve ter mesma dimensão que a quantidade de juntas do robô ($(size))")
    elseif length(r) != size
        error("O vetor de referência r deve ter mesma dimensão que a quantidade de juntas do robô ($(size))")
    end
    KP = diagm(kp)
    KV = diagm(kv)
    Ts = 0.02

    #função contendo o sistema, robô sendo utilizado
    function robot(t,u,du)
        #TODO: adicionar valores para esses termos
        m1 =
        L0 =
        L1 =
        l1 =
        I0 =
        I1 =
        g =


        θ = u[1:2]
        dθ = u[3:4]
        M = [I0 + m1*(L0^2)+(l1^2)*m1*(sin(θ[2])^2) L0*l1*m1*cos(θ[2]);
        L0*l1*m1*cos(θ[2]) I1+m1*(l1^2)]

        C = [2*(l1^2)*m1*sin(θ[2])*cos(θ[2])*dθ[2]  -L0*l1*m1*sin(θ[2])*dθ[2];
        -(l1)^2*m1*sin(θ[2])*cos(θ[2])*dθ[1]  0]

        G = [0;
        -g*l1*m1*sin(θ[2])]

        du[1:2] = dθ
        du[3:4] = inv(M)*(tau - C*dθ -G)
    end

    function controlador(integrator)
        # TODO: preciso ver como pegar os valores de erro de dentro da equação
        e = q - r
        for c in user_cache(integrator)
            c.tau = kp*e - kv*dotq
        end
    end

    cbs = PeriodicCallback(controlador,Ts)
    tspan = (t0,2)
    start = InputRobot(vcat(x_0,v_0), zeros(7))
    prob = ODEProblem(robot,start,tspan)
    sol = solve(prob,Tsit5(),callback = cbs)

    out_x = map(x -> x[1:size],sol.u) #posição
    out_dx = map(x -> x[(size+1):end],sol.u) #velocidade
    #NOTE:  Aqui estou fazendo uma aproximação da aceleração e do jerk
    out_d2x = diff(out_dx)/Ts #aceleração
    out_d3x = diff(out_d2x)/Ts #jerk

    #NOTE: agoa preciso escolher os valores a serem retornados para utiizar no otimizador

end
