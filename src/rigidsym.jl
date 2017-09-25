using RigidBodyDynamics
using StaticArrays
using SymPy

#=
Create symbolic parameters

    Masses: m1,m2

Mass moments of inertia (about center of mass): I1,I2
Link lengths: l1,l2
Center of mass locations (w.r.t. preceding joint axis): c1,c2
Gravitational acceleration: g

=#

inertias = @syms m_1 m_2 I_1 I_2 positive = true
lengths = @syms l_1 l_2 c_1 c_2 real = true
gravitational_acceleration = @syms g real = true
params = [inertias..., lengths..., gravitational_acceleration...]
params.'


#=
Create double pendulum Mechanism

A Mechanism contains the joint layout and inertia parameters, but no state information.

=#

T = Sym # the 'scalar type' of the Mechanism we'll construct
axis = SVector(zero(T), one(T), zero(T)) # axis of rotation for each of the joints
double_pendulum = Mechanism(RigidBody{T}("world"); gravity = SVector(zero(T), zero(T), g))
world = root_body(double_pendulum) # the fixed 'world' rigid body

# Attach the first (upper) link to the world via a revolute joint named 'shoulder'
inertia1 = SpatialInertia(CartesianFrame3D("upper_link"), I_1 * axis * axis.', m_1 * SVector(zero(T), zero(T), c_1), m_1)
body1 = RigidBody(inertia1)
joint1 = Joint("shoulder", Revolute(axis))
joint1_to_world = eye(Transform3D{T}, frame_before(joint1), default_frame(world))
attach!(double_pendulum, world, body1, joint1, joint_pose = joint1_to_world)

# Attach the second (lower) link to the world via a revolute joint named 'elbow'
inertia2 = SpatialInertia(CartesianFrame3D("lower_link"), I_2 * axis * axis.', m_2 * SVector(zero(T), zero(T), c_2), m_2)
body2 = RigidBody(inertia2)
joint2 = Joint("elbow", Revolute(axis))
joint2_to_body1 = Transform3D(frame_before(joint2), default_frame(body1), SVector(zero(T), zero(T), l_1))
attach!(double_pendulum, body1, body2, joint2, joint_pose = joint2_to_body1)

#=

Create MechanismState associated with the double pendulum Mechanism

A MechanismState stores all state-dependent information associated with a Mechanism.
=#
x = MechanismState(double_pendulum);

# Set the joint configuration vector of the MechanismState to a new vector of symbolic variables
configuration(x)[:] = [symbols("q_$i", real = true) for i = 1 : num_positions(x)]

# Set the joint velocity vector of the MechanismState to a new vector of symbolic variables
velocity(x)[:] = [symbols("v_$i", real = true) for i = 1 : num_positions(x)]

#=
Compute dynamical quantities in symbolic form

=#

# Mass matrix
M = mass_matrix(x)
map!(simplify, M.data, M.data) # Note: M is a Symmetric matrix type; need to simplify the underlying data
C = dynamics_bias(x)
map!(simplify, C, C)
