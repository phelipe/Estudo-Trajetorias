#=
Arquivo contendo o código do algoritmo de otimização
aqui serão feitas as buscas pelo melhores valores
=#

using Evolutionary
include(simulation.jl)

function objetivo()
    # Aqui vou colocar as entradas necessárias(ganhos) e executar a simulação
    # Simulação retorna os valores de erros, JERK e demais coisas necessárias
    # Aqui aplico os valores retornados da simulação propriamente na função objetivo elaborada
    # Retorno os escores
end

# Parametros
N = 2 # -> quantidade de ganhos a serem buscados, no caso serão 2 para cada junta, estou fazendo somente PD
terminate(σ) = σ < 1e-10

result, fitness, cnt = es(rosenbrock, N; initStrategy = strategy(σ = 1.0), recombination = average, mutation = isotropic, μ = 15, ρ = 5, λ = 100, iterations = 1000)
