# Estudo do pacote RigidBodyDynamics
using RigidBodyDynamics
using StaticArrays
using Plots

# Carregando um arquivo udrf com as características do pendulo duplo
urdf = "../doblependulum.urdf"
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

# O tipo MechanismState contém variáveis de cache que dependem de configurações e veocidades das juntas. Estas precisam ser invalidadas quando as configurações e velocidades são modificadas, isto pose ser feito como é mostrado a seguir
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
