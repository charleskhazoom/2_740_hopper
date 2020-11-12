function save_traj(time,z,u,name)

    traj.time = time;
    traj.q = z(3:5,:);
    traj.qd = z(8:10,:);
    traj.torques = u;
    save(name,'traj')
    
end