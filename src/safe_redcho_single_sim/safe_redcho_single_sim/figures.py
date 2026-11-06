"""Saved scientific figures for actual single-bank signals; no invented banks."""
from pathlib import Path
import numpy as np


def make_plots(h,meta,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from . import scenario
    output=Path(output);t=h['time'];colors=plt.get_cmap('tab10').colors[:6];saved=[]
    def axis(ax,title,ylabel):
        ax.set(title=title,xlabel='Time [s]',ylabel=ylabel);ax.grid(alpha=.3)
        ax.axvline(meta['fault_time'],color='k',ls=':',lw=1,label='cf2 fault')
    def save(fig,name):
        fig.tight_layout();path=output/name;fig.savefig(path,dpi=160);saved.append(path.name);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,5))
    for i,c in enumerate(colors):
        axs[0].plot(h['p_hist'][0,i],h['p_hist'][1,i],color=c,label=f'cf{i+1}')
        axs[0].plot(h['formation_target_hist'][0,i],h['formation_target_hist'][1,i],':',color=c,lw=.6)
        axs[1].plot(t,h['p_hist'][2,i],color=c)
    for center in scenario.OBSTACLE_CENTERS.T:
        for r,fill,style in [(scenario.OBSTACLE_RADIUS,True,'-'),(scenario.OBSTACLE_RADIUS+scenario.OBSTACLE_CLEARANCE,False,'--'),(scenario.OBSTACLE_RADIUS+scenario.OBSTACLE_DESIGN_CLEARANCE,False,':')]:
            axs[0].add_patch(Circle(center[:2],r,fill=fill,ls=style,color='orange',alpha=.6))
    axs[0].set(title='Actual paths and nominal targets',xlabel='x [m]',ylabel='y [m]');axs[0].axis('equal');axs[0].grid();axs[0].legend(fontsize=8)
    axis(axs[1],'Takeoff and common height','z [m]');axs[1].axhline(scenario.HOVER_HEIGHT,color='gray',ls='--')
    save(fig,'figure_1_trajectory.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8));axs=axs.ravel()
    for mu in range(3):
        axs[mu].plot(t,h['U_hist'][0,mu,1],label='cf2 local SVF')
        axis(axs[mu],['Filtered local velocity','Local acceleration','Local jerk'][mu],['m/s','m/s²','m/s³'][mu]);axs[mu].legend(fontsize=8)
    axs[0].plot(t,h['raw_ref_hist'][0,1],':',label='Raw battery input');axs[0].legend(fontsize=8)
    axs[3].plot(t,h['svf_regularizer_hist'],label='Max local SVF snap norm')
    axs[3].axhline(meta['L_svf'],color='r',ls='--',label='SVF bound');axis(axs[3],'Reference regularity','m/s⁴');axs[3].legend(fontsize=8)
    save(fig,'figure_2_local_reference.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8));axs=axs.ravel()
    for mu in range(3):
        for i,c in enumerate(colors):axs[mu].plot(t,h['pose_redcho_estimate_hist'][0,mu,i],color=c,lw=.8,label=f'cf{i+1}')
        axs[mu].plot(t,h['position_reference_average_hist'][0,mu],'k--',lw=1.4,label='True reference average (logging only)')
        axis(axs[mu],['Position estimate, x','Velocity derivative output, x','Acceleration derivative output, x'][mu],['m','m/s','m/s²'][mu])
    axs[0].legend(fontsize=7,ncol=2)
    for mu,label in enumerate(['Position [m]','Velocity [m/s]','Acceleration [m/s²]']):
        axs[3].semilogy(t,np.maximum(h['position_redcho_order_error_hist'][mu],1e-12),label=label)
    axis(axs[3],'One bank: mean reconstruction error','Order-dependent units');axs[3].legend(fontsize=8)
    save(fig,'figure_3_single_position_redcho.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8));axs=axs.ravel()
    axs[0].plot(t,h['min_dist_hist']);axs[0].axhline(scenario.INTERAGENT_DISTANCE,color='r',ls='--',label='Physical requirement');axs[0].axhline(scenario.INTERAGENT_DESIGN_DISTANCE,color='gray',ls=':',label='Design');axis(axs[0],'Minimum pair separation','m');axs[0].legend(fontsize=8)
    for o in range(2):axs[1].plot(t,h['min_phys_obs_dist'][o],label=f'Obstacle {o+1}')
    axs[1].axhline(scenario.OBSTACLE_CLEARANCE,color='r',ls='--',label='Physical requirement');axis(axs[1],'Obstacle surface clearance','m');axs[1].legend(fontsize=8)
    for key,label in [('qp_violation_hist','Hard-QP violation'),('qp_phase1_slack_hist','Phase-I slack')]:axs[2].semilogy(t,np.maximum(h[key].max(axis=0),1e-14),label=label)
    axis(axs[2],'Hard feasibility diagnostics','Normalized residual');axs[2].legend(fontsize=8)
    for key,label in [('accel_utilization_hist','Acceleration'),('slew_utilization_hist','Slew'),('motor_utilization_hist','Motor')]:axs[3].plot(t,h[key].max(axis=0),label=label)
    axs[3].axhline(1,color='k',ls='--');axis(axs[3],'Actuation utilization','Fraction of limit');axs[3].legend(fontsize=8)
    save(fig,'figure_4_safety.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8));axs=axs.ravel()
    for i,c in enumerate(colors):
        axs[0].plot(t,h['SoC_hist'][i],color=c,label=f'cf{i+1}');axs[1].plot(t,h['V_batt_hist'][i],color=c)
        axs[2].plot(t,h['v_safe_hist'][i],color=c,ls=':',lw=.7)
        axs[2].plot(t,np.linalg.norm(h['pose_redcho_estimate_hist'][:,1,i],axis=0),color=c)
    axis(axs[0],'Local charge state','Fraction');axs[0].legend(fontsize=8)
    axis(axs[1],'Terminal voltage','V');axis(axs[2],'Local governor (dotted) and bank velocity','m/s')
    axs[3].plot(t,h['P_elec_hist'][1],label='cf2 power');right=axs[3].twinx();right.plot(t,h['I_batt_hist'][1],'r--',label='cf2 current');right.set_ylabel('A');axis(axs[3],'cf2 electrical response','W')
    save(fig,'figure_5_battery.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8));axs=axs.ravel()
    for ax,key,title,units in zip(axs,['formation_shape_error_hist','height_spread_hist','vel_consensus_err_hist','plant_tracking_err_hist'],['Shape recovery','Common-height spread','Velocity consensus','Plant velocity tracking'],['m','m','m/s','m/s']):
        ax.plot(t,h[key]);axis(ax,title,units)
    save(fig,'figure_6_formation.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8));axs=axs.ravel()
    for motor in range(4):axs[0].plot(t,h['u_motors_hist'][motor,1],label=f'cf2 motor {motor+1}')
    axs[0].plot(t,h['F_max_hist'][1],'k--',label='Voltage cap');axis(axs[0],'Motor force allocation','N');axs[0].legend(fontsize=8)
    axs[1].plot(t,h['model_mismatch_hist'].max(axis=0));axs[1].axhline(.4,color='r',ls='--');axis(axs[1],'Acceleration residual (includes contact)','m/s²')
    axs[2].plot(t,h['attitude_error_hist'].T);axis(axs[2],'Geometric attitude error','||e_R||')
    axs[3].plot(t,h['qp_correction_hist'].T);axis(axs[3],'QP nominal correction','m/s²')
    save(fig,'figure_7_motor_plant.png')
    fig=plt.figure(figsize=(11,8));ax=fig.add_subplot(111,projection='3d')
    for i,c in enumerate(colors):ax.plot(*h['p_hist'][:,i],color=c,label=f'cf{i+1}')
    u,v=np.meshgrid(np.linspace(0,2*np.pi,30),np.linspace(0,np.pi,15))
    for center in scenario.OBSTACLE_CENTERS.T:
        r=scenario.OBSTACLE_RADIUS;ax.plot_surface(center[0]+r*np.cos(u)*np.sin(v),center[1]+r*np.sin(u)*np.sin(v),center[2]+r*np.cos(v),color='orange',alpha=.4)
    ax.set(xlabel='x [m]',ylabel='y [m]',zlabel='z [m]',title='Native/numerical plant paths (see metadata backend)');ax.legend()
    save(fig,'figure_8_3d_paths.png')
    return saved
