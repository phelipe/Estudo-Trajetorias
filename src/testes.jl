

# a = minimumjerk(0,0,0,20,0,0,30)
# s = functionform(a)

function minimumjerk(x0,v0,a0,t0,xf,vf,af,tf)
    b=[t0^5 t0^4 t0^3 t0^2 t0 1;
     tf^5 tf^4 tf^3 tf^2 tf 1;
     5*(t0^4) 4*(t0^3) 3*(t0^2) 2*(t0) 1 0;
     5*(tf^4) 4*(tf^3) 3*(tf^2) 2*(tf) 1 0;
     20*(t0^3) 12*(t0^2) 6*(t0) 2 0 0;
     20*(tf^3) 12*(tf^2) 6*(tf) 2 0 0]
     c = [x0, v0, a0, xf, vf, af]
     a = inv(b)*c
    return a
end

function functionform(a)
    return (t -> a[1] + a[2]*t + a[3]*t^2 + a[4]*t^3 + a[5]*t^4 + a[6]*t^5, t-> a[2] + a[3]*t^1 + a[4]*t^2 + a[5]*t^3 + a[6]*t^4, t-> a[3] + a[4]*t^1 + a[5]*t^2 + a[6]*t^3)
end


minimumjerk(0,0,0,0,20,0,0,3)
