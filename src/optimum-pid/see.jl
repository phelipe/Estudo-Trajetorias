using RigidBodyDynamics
using StaticArrays
using CoordinateTransformations
using GeometryTypes
using DrakeVisualizer
using RigidBodyTreeInspector

urdf = "../urdf/pendulum.urdf"
mechanism = parse_urdf(Float64, urdf)
state = MechanismState{Float64}( mechanism)
DrakeVisualizer.any_open_windows() || DrakeVisualizer.new_window();
vis = Visualizer(mechanism);
