using Evolutionary
using Base.Test

# Objective function
rosenbrock(x::Vector{Float64}) = (1.0 - x[1])^2 + 100.0 * (x[2] - x[1]^2)^2

# Parameters
N = 2
terminate(σ) = σ < 1e-10

# Testing: (15/5+100)-ES
# with isotropic mutation operator y' := y + σ(N_1(0, 1), ..., N_N(0, 1))
result, fitness, cnt = es(rosenbrock, N; initStrategy = strategy(σ = 1.0), recombination = average, mutation = isotropic, μ = 15, ρ = 5, λ = 100, iterations = 1000)
println("(15/5+100)-ES => F: $(fitness), C: $(cnt), OBJ: $(result)")
