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
# agora criamos o referencial inercial do corpo
# basicamente definimos um frame 3D, informamos a matriz de momento de inércia do corpo, a posição do centro
#  de massa com relação ao sistema de referência local(no caso, o sistema de referência local está
#  tendo origem no próprio centro de massa, assim será um vetor de zeros) multiplicado pela massa e informamos a massa.

joint1 = Joint("joint1", Revolute(axis))
inertia1 = SpatialInertia(CartesianFrame3D("inertia1_centroidal"), I_1*axis*axis', m_1*SVector(c_1, 0., 0.), m_1)
link1 = RigidBody(inertia1)
joint1_to_world = eye(Transform3D, frame_before(joint1), default_frame(world))
attach!(fourbar, world, link1, joint1, joint_pose = joint1_to_world)

joint2 = Joint("joint2", Revolute(axis))
inertia2 = SpatialInertia(CartesianFrame3D("inertia2_centroidal"), I_2*axis*axis', m_2*SVector(c_2, 0., 0.), m_2)
link2 = RigidBody(inertia2)
joint2_to_link1 = Transform3D(frame_before(joint2), default_frame(link1), SVector(l_1,0.,0.))
attach!(fourbar, link1, link2, joint2, joint_pose = joint2_to_link1)

joint3 =Joint("joint3", Revolute(axis))
inertial3 = SpatialInertia(CartesianFrame3D("inertia3_centroidal"), I_3*axis*axis', m_3*SVector(c_3, 0., 0.),m_3)
link3 = RigidBody(inertial3)
joint3_to_world = Transform3D(frame_before(joint3), default_frame(world), SVector(l_0, 0., 0.))
attach!(fourbar, world, link3, joint3, joint_pose = joint3_to_world)


joint4 = Joint("joint4", Revolute(axis))
