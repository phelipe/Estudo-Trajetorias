using RigidBodyDynamics
using StaticArrays
# Nesse código será mostrado como modelar mecanismos utilizando o pacote RigidBodyDynamics.
# O primeiro passo é ter um modelo em mente, neste caso utilizaremos um mecanismo de quatro barras
# apresentado no desenho do caderno.


# Agora iremos definir algumas constantes importantes
# - aceleação da gravidade
g = 9.81
# - comprimento das barras
l_0 = 1.10
l_1 = 0.5
l_2 = 1.20
l_3 = 0.75
# - massa das barras
m_1 = 0.5
m_2 = 1.0
m_3 = 0.75
# - centro de massa
c_1 = 0.25
c_2 = 0.60
c_3 = 0.375
# - momentos de inércia com relação ao centro de massa
I_1 = 0.333
I_2 = 0.537
I_3 = 0.4

# Definimos agora o eixo de rotação.
# Como todas as barras giram em torno do eixo -y será criado um único eixo de rotação.
# Será utilizado o pacote StaticArrays para questão de velocidade
T = Float64
axis = SVector(zero(T),-one(T),zero(T))
# Agora temos um vetor com rotação 0 em x -1 em y e 0 em z

# Construiremos o corpo rígido mundo e criaremos um novo mecanismo
world = RigidBody{T}("world")
fourbar = Mechanism(world; gravity = SVector(0., 0., g))

# Partiremos agora para a criação da árvore cinemática em si
# primeiro criaremos a junta 1
joint1 = Joint("joint1", Revolute(axis))
#agora criamos o referencial inercial do corpo
inertia1 = SpatialInertia(CartesianFrame3D("inertia1_centroidal"), I_1*axis*axis', m_1*zeros(SVector{3, T}), m_1)
# basicamente definimos um frame 3D, informamos a matriz de momento de inércia do corpo, a posição do centro de massa com relação ao sistema de referência local(no caso, o sistema de referência local está tendo origem no próprio centro de massa, assim será um vetor de zeros) multiplicado pela massa e informamos a massa.
link1 = RigidBody(inertia1)

# basicamente será necessário fazer duas transformações aqui já que não adotei o eixo no iníio do corpo e sim no meio dele, assim tenho uma transformação entre o eixo global e a junta 1 e depois entre a junta 1 e o eixo de gravidade. Esta basicamente será uma translação do eixo
before_joint1_to_world = eye(Transform3D, frame_before(joint1), default_frame(world))
c1_to_joint = Transform3D(inertia1.frame, frame_after(joint1), SVector(c_1, 0, 0))
attach!(fourbar, world, link1, joint1, joint_pose = before_joint1_to_world, successor_pose = c1_to_joint)
