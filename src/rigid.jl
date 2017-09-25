# Estudo do pacote RigidBodyDynamics
using RigidBodyDynamics
using StaticArrays
using Plots

# Carregando um arquivo udrf com as características do pendulo duplo
urdf = "urdf/doublependulum.urdf"
doublependulum = parse_urdf(Float64, urdf)

# Aqui fica visível os corpos que compões o pendulo duplo
@show collect(bodies(doublependulum))

# Aqui fica visível as juntas, normalmente temos uma junta a mais, esta é utilizada como base do mecanismo.
@show collect(joints(doublependulum))

# O ESTADO DO MECANISMO
# O  tipo Mechanism guarda o layout das juntas/corpos mas não guarda o estado destes. A informação de estado fica contida em um tipo separado chamado MechanismState
state = MechanismState(doublependulum)
# opcionalmente eu posso escolher o tipo fazendo MechanismState{Float64} (doublependulum)

# Vamos agora modificar a configuração e velocidade das juntas
fixedjoint, shoulder, elbow = (joints(doublependulum)...)
configuration(state, shoulder)[:] = 0.3
configuration(state, elbow)[:] = 0.4
velocity(state, shoulder)[:] = 1.
velocity(state, elbow)[:] = 2.

# O tipo MechanismState contém variáveis de cache que dependem de configurações e velocidades das juntas. Estas precisam ser invalidadas quando as configurações e velocidades são modificadas, isto pose ser feito como é mostrado a seguir
setdirty!(state)

# uma forma de modificar o estado sem que seja necessário utilizar o setdirty! é através das funções apresentadas abaixo, elas mudam o estado e velocidade já realizando essa operação.
set_configuration!(state,[0.3, 0.4])
set_velocity!(state,[1.,1.])
# é possível também fazer isso escolhendo as juntas específicas
set_configuration!(state, shoulder, [0.3])
set_configuration!(state, elbow, [0.4])
set_velocity!(state, shoulder, [1.])
set_velocity!(state, elbow, [2.])

# As configurações e velocidades são armazenadas como vetores dentro do objeto  MechanismState
q = configuration(state)
v = velocity(state)
# é possível pegar o estado e velocidade de uma junta específica utilizando velocity(state, joint) ou configuration(state, joint)



# CINEMÁTICA

# DINÂMICA
# O  tipo Mechanism guarda o layout das juntas/corpos mas não guarda a dinâmica nem a cinemática destes. Esta informação fica contida em um tipo separado chamado DynamicsResult.
result = DynamicsResult(doublependulum)

# calcula o centro de massa do mecanismo no estado(state)
center_of_mass(state)

# Calcula o vetor de aceleração (dinâmica direta) e o multiplicador de lagrange da equação de Euler-Lagrange
# a = dynamics!(result, state, [1.0,1.0])

# Calcula o termo de viés dinâmico da equação de Euler Lagrange no estado(state)
dynamics_bias(state)

# Calcula a massa total do mecanismo
mass(doublependulum)

# Calcula a matriz de massa do macanismo no estado atual(state)
mass_matrix(state)

# Calcula a dinâmica inversa do mecanismo a aprti do estado atual e da aceleração atual
v̇ = [2.; 3.] # the joint acceleration vector, i.e., the time derivative of the joint velocity vector v
inverse_dynamics(state, v̇)
