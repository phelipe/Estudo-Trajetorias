using RigidBodyDynamics
using StaticArrays

#using CoordinateTransformations
#using GeometryTypes
#using DrakeVisualizer
#using RigidBodyTreeInspector

#delete!(Visualizer());
#DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window();

urdf = "../doblependulum.urdf"
mechanism = parse_urdf(Float64, urdf);

#vis = Visualizer()[:doublependulum]
#setgeometry!(vis, mechanism);
state = MechanismState(mechanism)
#inspect!(state, vis)
times, joint_angles, joint_velocities = simulate(state, 5.);
