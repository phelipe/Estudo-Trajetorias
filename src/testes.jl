using RigidBodyDynamics
using Plots
pyplot()
urdf = "../iiwa14.urdf"
mechanism = parse_urdf(Float64, urdf)
const state = MechanismState(Float64, mechanism)
times, joint_angles, joint_velocities = simulate(state, 5.)
