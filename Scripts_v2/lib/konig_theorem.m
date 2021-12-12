function T = konig_theorem(m_i, vi_ci, omegai_i, Ii_ci)
    m = m_i;
    vc = vi_ci;
    omega = omegai_i;
    Ic = Ii_ci;
    
    vc_squared = (vc')*vc;
    
    if class(vc_squared) ~= "double"
        vc_squared = simplify(vc_squared);
    end
    
    T = 0.5*m*vc_squared + 0.5*(omega')*Ic*omega;
    T = simplify(T);
end
