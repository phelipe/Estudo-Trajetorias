using RigidBodyDynamics
using StaticArrays
using CoordinateTransformations
using GeometryTypes
using DrakeVisualizer
using RigidBodyTreeInspector

urdf = "urdf/kuka.urdf"
mechanism = parse_urdf(Float64, urdf)
state = MechanismState{Float64}( mechanism)
DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window();
vis = Visualizer(mechanism);
set_configuration!(state,[10, 20, 120,20,120,120,120.])
settransform!(vis, state)
