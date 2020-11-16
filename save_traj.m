function save_traj(time,z,u,name,dt_interp)

    t_interp = time(1):dt_interp:time(end);
    
    z_interp = interp1(time,z',t_interp)';
    u_interp = interp1(time,u',t_interp)';
    
    traj.time = time;
    traj.q = z_interp(3:5,:);
    traj.qd = z_interp(8:10,:);
    traj.torques = u_interp;
    save(name,'traj')
    
end