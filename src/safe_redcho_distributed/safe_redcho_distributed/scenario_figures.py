"""All static figures and the interactive 3D playback."""
import numpy as np


def make_plots(h, meta, output):
    import matplotlib.pyplot as plt
    t, tf = h['time'], meta['fault_time']
    colors = list(plt.get_cmap('tab10').colors[:6]); colors[1]=(0.9,0.1,0.1); colors[3]=(1.,.6,.1)
    figs = []
    def style(ax, title, ylabel, log=False):
        ax.set(title=title,xlabel='Time [s]',ylabel=ylabel)
        ax.grid(True,alpha=.3); ax.axvline(tf,color='k',ls=':',lw=1)
        if log: ax.set_yscale('log')
    def finish(fig,name):
        fig.tight_layout();fig.savefig(output/name,dpi=180);figs.append(fig)
    fig,axs=plt.subplots(1,2,figsize=(12,5),num='Figure 1: ground takeoff and obstacle bypass')
    from . import scenario
    for i in range(6):
        axs[0].plot(h['p_hist'][0,i],h['p_hist'][1,i],color=colors[i],label=f'cf{i+1}')
        axs[1].plot(t,h['p_hist'][2,i],color=colors[i])
    from matplotlib.patches import Circle
    for centre in scenario.OBSTACLE_CENTERS.T:
        axs[0].add_patch(Circle(centre[:2],scenario.OBSTACLE_RADIUS,color='orange',alpha=.6))
        axs[0].add_patch(Circle(centre[:2],scenario.OBSTACLE_RADIUS+scenario.OBSTACLE_CLEARANCE,fill=False,color='red',ls='--'))
        axs[0].add_patch(Circle(centre[:2],scenario.OBSTACLE_RADIUS+scenario.OBSTACLE_DESIGN_CLEARANCE,fill=False,color='gray',ls=':'))
    axs[0].set(xlabel='x [m]',ylabel='y [m]',title='Top projection: reactive 3D avoidance');axs[0].axis('equal');axs[0].grid(True);axs[0].legend(fontsize=8)
    axs[1].set(xlabel='Time [s]',ylabel='Altitude [m]',title='Ground start, takeoff and hover gate')
    axs[1].axhline(scenario.HOVER_HEIGHT,color='k',ls='--',label='Hover target')
    axs[1].axvline(scenario.FORWARD_START,color='r',ls=':',label='Forward starts after hover gate');axs[1].legend();axs[1].grid(True)
    finish(fig,'figure_1_ground_trajectory.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8),num='Figure 2: SVF and Levant')
    axs=axs.ravel()
    axs[0].plot(t,h['raw_ref_hist'][0,1],':',color='k',label='Raw battery reference')
    for mu in range(3):
        axs[mu].plot(t,h['U_hist'][0,mu,1],color='b',label='SVF')
        axs[mu].plot(t,h['U_levant_hist'][0,mu,1],'r--',lw=.8,label='Levant')
        style(axs[mu],['(a) Velocity supplied to REDCHO','(b) Acceleration input','(c) Jerk input'][mu],['m/s','m/s²','m/s³'][mu])
        axs[mu].set_xlim(max(0,tf-1),min(meta['duration'],tf+5));axs[mu].legend(fontsize=8)
    axs[3].semilogy(t,np.maximum(h['svf_regularizer_hist'],1e-12),label='Measured max ||u⁽³⁾||')
    axs[3].axhline(meta['L_svf'],color='r',ls='--',label=f'Analytical L_svf={meta["L_svf"]:.2f}')
    style(axs[3],'(d) SVF third-derivative certificate','m/s⁴');axs[3].legend(fontsize=8)
    finish(fig,'figure_2_svf_levant.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8),num='Figure 3: REDCHO reconstruction');axs=axs.ravel()
    for mu in range(3):
        for i in range(6):axs[mu].plot(t,h['Y_hist'][0,mu,i],color=colors[i],lw=1,label=f'Agent {i+1}')
        axs[mu].plot(t,h['U_avg_hist'][0,mu],'k--',lw=2,label='True network average')
        style(axs[mu],f'({chr(97+mu)}) '+['Velocity','Acceleration','Jerk'][mu]+' reconstruction',['m/s','m/s²','m/s³'][mu])
    axs[0].legend(fontsize=7,ncol=2)
    for mu in range(3):
        axs[3].semilogy(t,np.maximum(h['redcho_order_err_svf'][mu],1e-12),color=colors[mu],label=f'SVF order {mu}')
        axs[3].semilogy(t,np.maximum(h['redcho_order_err_levant'][mu],1e-12),'--',color=colors[mu],label=f'Levant order {mu}')
    style(axs[3],'(d) REDCHO error: SVF vs Levant','Mean reconstruction error');axs[3].legend(fontsize=7,ncol=2)
    finish(fig,'figure_3_redcho.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8),num='Figure 4: Safety and feasibility');axs=axs.ravel()
    axs[0].plot(t,h['min_dist_hist'],'k',label='Minimum pair distance')
    axs[0].axhline(meta['interagent_safety_distance'],color='r',ls='--',label=f"Required {meta['interagent_safety_distance']:.2f} m");axs[0].axhline(meta['interagent_design_distance'],color='b',ls=':',label=f"Design {meta['interagent_design_distance']:.2f} m")
    style(axs[0],'(a) Inter-agent separation','Distance [m]');axs[0].legend(fontsize=8)
    for o in range(2):axs[1].plot(t,h['min_phys_obs_dist'][o],label=f'Obstacle {o+1}')
    axs[1].axhline(meta['obstacle_surface_clearance'],color='m',ls='--',label=f"Required surface clearance {meta['obstacle_surface_clearance']:.2f} m");axs[1].axhline(meta['obstacle_design_clearance'],color='k',ls=':',label=f"Design {meta['obstacle_design_clearance']:.2f} m")
    style(axs[1],'(b) Obstacle clearance','Surface clearance [m]');axs[1].legend(fontsize=8)
    for key,label,reduce in [('row_actuator_margin_hist','Minimum row margin',np.min),('qp_phase1_slack_hist','Maximum Phase-I slack',np.max),('qp_violation_hist','Maximum hard-QP violation',np.max)]:
        axs[2].semilogy(t,np.maximum(reduce(h[key],axis=0),1e-12),label=label)
    axs[2].axhline(1e-7,color='k',ls='-.',label='QP tolerance')
    style(axs[2],'(c) Hard feasibility certificates','Normalized value');axs[2].legend(fontsize=8)
    for key,label in [('accel_utilization_hist','Acceleration box'),('motor_utilization_hist','Motor thrust'),('slew_utilization_hist','Acceleration slew')]:axs[3].plot(t,h[key].max(axis=0),label=label)
    axs[3].axhline(1,color='k',ls=':',label='Hard limit');style(axs[3],'(d) Actuation and plant residual','Maximum utilization')
    ar=axs[3].twinx();ar.plot(t,h['model_mismatch_hist'].max(axis=0),'g',label='Measured SE(3) residual');ar.axhline(.4,color='m',ls='-.',label='Residual bound');ar.set_ylabel('Acceleration residual [m/s²]')
    lines,labels=axs[3].get_legend_handles_labels();lines2,labels2=ar.get_legend_handles_labels();axs[3].legend(lines+lines2,labels+labels2,fontsize=7)
    finish(fig,'figure_4_safety.png')
    for number,key,mean,title in [(5,'redcho_agent_err_hist','redcho_observer_err_hist','REDCHO observer error'),(6,'plant_agent_err_hist','plant_tracking_err_hist','Plant tracking error'),(7,'cohesion_agent_err_hist','vel_consensus_err_hist','Swarm cohesion error')]:
        fig,ax=plt.subplots(figsize=(10,5),num=f'Figure {number}: {title}')
        for i in range(6):ax.plot(t,h[key][i],color=colors[i],label=f'Agent {i+1}')
        ax.plot(t,h[mean],'k',lw=2,label='Swarm mean');style(ax,title,'Error magnitude [m/s]');ax.legend(fontsize=8)
        finish(fig,f'figure_{number}_errors.png')
    fig,axs=plt.subplots(2,2,figsize=(11,8),num='Figure 8: Battery-to-SE(3)');axs=axs.ravel()
    axs[0].plot(t,h['V_batt_hist'][1],'r',label='Faulted Agent 2');axs[0].plot(t,h['V_batt_hist'][[0,2,3,4,5]].mean(axis=0),'b--',label='Healthy-agent mean');axs[0].axhline(3,color='k',ls=':',label='Critical voltage');style(axs[0],'(a) 1-RC Thevenin fault response','Terminal voltage [V]');axs[0].legend(fontsize=8)
    axs[1].plot(t,h['v_safe_hist'][1],'r--',label='Agent 2 local battery speed')
    axs[1].plot(t,np.linalg.norm(h['U_avg_hist'][:,0],axis=0),'k',label='Filtered network average')
    axs[1].plot(t,np.linalg.norm(h['Y_hist'][:,0,1],axis=0),':',label='Agent 2 REDCHO estimate')
    axs[1].plot(t,h['v_mag_hist'][1],'b',label='Agent 2 plant speed');style(axs[1],'(b) Mission rescaling and plant response','Speed [m/s]')
    ar=axs[1].twinx();ar.plot(t,h['qp_correction_hist'][1],color='.5',ls='-.',label='QP correction');ar.set_ylabel('QP correction [m/s²]')
    lines,labels=axs[1].get_legend_handles_labels();lines2,labels2=ar.get_legend_handles_labels();axs[1].legend(lines+lines2,labels+labels2,fontsize=7)
    axs[2].plot(t,h['P_elec_hist'][1],'b',label='Electrical power');style(axs[2],'(c) Propulsion-electrical response','Electrical power [W]')
    ar=axs[2].twinx();ar.plot(t,h['I_batt_hist'][1],'r--',label='Battery current');ar.set_ylabel('Battery current [A]')
    for key,label in [('redcho_observer_err_hist','Mean REDCHO error'),('plant_tracking_err_hist','Mean plant tracking error'),('vel_consensus_err_hist','Mean cohesion error')]:axs[3].semilogy(t,np.maximum(h[key],1e-8),label=label)
    style(axs[3],'(d) Cascaded estimation and tracking errors','Velocity error [m/s]')
    ar=axs[3].twinx();ar.semilogy(t,np.maximum(h['attitude_error_hist'].mean(axis=0),1e-8),'k',label='Mean attitude error');ar.set_ylabel('Mean ||e_R||')
    lines,labels=axs[3].get_legend_handles_labels();lines2,labels2=ar.get_legend_handles_labels();axs[3].legend(lines+lines2,labels+labels2,fontsize=7)
    finish(fig,'figure_8_battery_cascade.png')
    return figs



def playback(h,meta):
    """Figure 1: six-panel trajectory playback and live telemetry (same signals)."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    fig=plt.figure(figsize=(14,8),num='Figure 1: 3D Flight and Real-Time Telemetry')
    ax=fig.add_subplot(2,3,(1,4),projection='3d');av=fig.add_subplot(2,3,2);ae=fig.add_subplot(2,3,3)
    ad=fig.add_subplot(2,3,5);at=fig.add_subplot(2,3,6);at.axis('off')
    ax.set(xlim=(-6,meta['duration']),ylim=(-5,18),zlim=(-4,5),xlabel='X [m]',ylabel='Y [m]',zlabel='Z [m]',title='3D swarm trajectory')
    ax.view_init(28,32)
    phi,theta=np.meshgrid(np.linspace(0,np.pi,20),np.linspace(0,2*np.pi,20))
    for o in range(2):
        center=np.array(meta['obstacle_centers'])[:,o];radius=np.array(meta['obstacle_radii'])[:,o]
        ax.plot_surface(center[0]+radius[0]*np.sin(phi)*np.cos(theta),center[1]+radius[1]*np.sin(phi)*np.sin(theta),center[2]+radius[2]*np.cos(phi),alpha=.35,color=['r','b'][o])
    colors=list(plt.get_cmap('tab10').colors[:6]);colors[1]=(0.9,0.1,0.1); colors[3]=(1.,.6,.1)
    trails=[ax.plot([],[],[],color=colors[i],label=f'CF {i+1}')[0] for i in range(6)]
    agents=[ax.plot([],[],[],'o',color=colors[i])[0] for i in range(6)]
    links=[ax.plot([],[],[],'k:',lw=.8)[0] for _ in range(6)]
    volts=[av.plot([],[],color=colors[i],label=f'Agent {i+1}')[0] for i in range(6)]
    speeds=[ae.plot([],[],color=colors[i])[0] for i in range(6)]
    er=ae.twinx(); errs=[er.plot([],[],ls=ls,color=c,label=label)[0] for ls,c,label in [('-','g','Observer'),('-.','b','Plant'),('--','m','Cohesion')]]
    distance=ad.plot([],[],'k',label='Minimum pair distance')[0];dr=ad.twinx();obstacles=[dr.plot([],[],label=f'Obstacle {o+1}')[0] for o in range(2)]
    text=at.text(.02,.5,'',va='center',fontfamily='monospace',fontsize=9)
    for axes,title,ylabel in [(av,'Battery voltages','Terminal voltage [V]'),(ae,'Consensus and errors','Speed [m/s]'),(ad,'Spacing and obstacle clearances','Pair distance [m]')]:
        axes.set(xlim=(0,meta['duration']),title=title,xlabel='Time [s]',ylabel=ylabel);axes.grid(alpha=.3);axes.axvline(12,color='k',ls='--')
    av.set_ylim(3.1,4.25);av.axhline(3,color='r',ls='--');ae.set_ylim(.2,1.65)
    er.set_ylim(0,max(.01,1.1*max(h[key].max() for key in ['redcho_observer_err_hist','plant_tracking_err_hist','vel_consensus_err_hist'])))
    ad.set_ylim(.7,1.1*h['min_dist_hist'].max());ad.axhline(meta['interagent_safety_distance'],color='r',ls='--')
    dr.set_ylim(0,1.1*h['min_phys_obs_dist'].max());dr.axhline(meta['obstacle_surface_clearance'],color='k',ls=':')
    er.set_ylabel('Velocity error [m/s]');dr.set_ylabel('Obstacle clearance [m]');ax.legend(fontsize=8);av.legend(fontsize=7,ncol=2);er.legend(fontsize=7)
    def update(k):
        ts=h['time'][:k+1]
        for i in range(6):
            xyz=h['p_hist'][:,i,:k+1];trails[i].set_data_3d(*xyz);agents[i].set_data_3d(*h['p_hist'][:,i,k:k+1])
            volts[i].set_data(ts,h['V_batt_hist'][i,:k+1]);speeds[i].set_data(ts,h['v_mag_hist'][i,:k+1])
            j=(i+1)%6;links[i].set_data_3d(*h['p_hist'][:,[i,j],k])
        for line,key in zip(errs,['redcho_observer_err_hist','plant_tracking_err_hist','vel_consensus_err_hist']):line.set_data(ts,h[key][:k+1])
        distance.set_data(ts,h['min_dist_hist'][:k+1])
        for o,line in enumerate(obstacles):line.set_data(ts,h['min_phys_obs_dist'][o,:k+1])
        text.set_text(f't = {h["time"][k]:5.2f} s\nAgent 2 fault: {"ACTIVE" if h["time"][k]>=meta["fault_time"] else "NOMINAL"}\nVoltage: {h["V_batt_hist"][1,k]:.3f} V\nSoC: {100*h["SoC_hist"][1,k]:.1f}%\nAvailable accel: {h["u_max_hist"][1,k]:.3f} m/s²\nMean speed: {h["v_mag_hist"][:,k].mean():.4f} m/s\nObserver error: {h["redcho_observer_err_hist"][k]:.5f}\nPlant error: {h["plant_tracking_err_hist"][k]:.5f}\nCohesion error: {h["vel_consensus_err_hist"][k]:.5f}\nMin pair distance: {h["min_dist_hist"][k]:.4f} m\nObs 1 clearance: {h["min_phys_obs_dist"][0,k]:.4f} m\nObs 2 clearance: {h["min_phys_obs_dist"][1,k]:.4f} m')
        return []
    fig.tight_layout()
    # A retained reference is required to prevent Matplotlib garbage collection.
    animation=FuncAnimation(fig,update,frames=range(0,len(h['time']),4),interval=1,repeat=False,blit=False,cache_frame_data=False)
    fig._swarm_animation=animation
    return animation

