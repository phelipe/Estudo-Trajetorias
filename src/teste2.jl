
using RigidBodyDynamics

urdf = "urdf/doublependulum.urdf"
mechanism = parse_urdf(Float64, urdf)
state = MechanismState{Float64}( mechanism)
times, joint_angles, joint_velocities = simulate(state, 5.)
