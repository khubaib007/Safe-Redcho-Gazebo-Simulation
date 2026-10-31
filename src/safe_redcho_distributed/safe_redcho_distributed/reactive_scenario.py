"""Startup settings and shared geometry; forward computation is the source algorithm."""
import numpy as np
IMPLEMENTATION_VERSION='redcho-pose-consensus-v2'
DT=.01
GROUND_COM_HEIGHT=.0175
INITIAL_SPACING=1.
INITIAL_POSITIONS=np.array([[-3.5,-5.5+i,GROUND_COM_HEIGHT] for i in range(6)]).T
HOVER_HEIGHT=.8
TAKEOFF_START=3.
TAKEOFF_END=4.8
FORWARD_START=5.
INTERAGENT_DISTANCE=.7
INTERAGENT_BUFFER=.12
INTERAGENT_DESIGN_DISTANCE=INTERAGENT_DISTANCE+INTERAGENT_BUFFER
OBSTACLE_CLEARANCE=.5
OBSTACLE_BUFFER=.25
OBSTACLE_DESIGN_CLEARANCE=OBSTACLE_CLEARANCE+OBSTACLE_BUFFER
OBSTACLE_RADIUS=.5
OBSTACLE_CENTERS=np.array([[0.,6.],[-3.5,2.],[.2,.2]])
HOVER_HEIGHT_TOLERANCE=.05
HOVER_SPEED_TOLERANCE=.05

# Position/height REDCHO estimates the common displacement reference. Individual
# height proposals have mean 0.8 m; their differences make height averaging a
# real distributed operation rather than six identical constant inputs.
HEIGHT_PROPOSALS=np.array([.78,.82,.85,.75,.80,.80])
POSE_REDCHO_L=32.
POSE_REDCHO_THETA=1.2
POSE_REDCHO_EPS=.04
POSE_REFERENCE_BOUND=40.
POSE_ESTIMATION_TOLERANCE=.01
MISSION_DURATION=45.


# Fixed world-frame offsets preserve the original side-by-side shape.
FORMATION_RAMP_TIME=2.
FORMATION_POSITION_GAIN=np.array([2.4,2.4,2.4])
FORMATION_POSITION_ACCEL_CAP=np.array([.8,.8,1.0])
RECOVERY_WINDOW=5.
FORMATION_POSITION_TOLERANCE=.05
FORMATION_VELOCITY_TOLERANCE=.03
FORMATION_HEIGHT_TOLERANCE=.03
# Default-scene progress check only; this never enters the controller.
RECOVERY_CHECK_START=40.
RECOVERY_PASS_X=float(np.max(OBSTACLE_CENTERS[0])+OBSTACLE_RADIUS)

def startup_reference(t):
    s=np.clip((t-TAKEOFF_START)/(TAKEOFF_END-TAKEOFF_START),0.,1.)
    duration=TAKEOFF_END-TAKEOFF_START;dz=HOVER_HEIGHT-GROUND_COM_HEIGHT
    f=10*s**3-15*s**4+6*s**5
    fd=(30*s**2-60*s**3+30*s**4)/duration
    fdd=(60*s-180*s**2+120*s**3)/duration**2
    target=INITIAL_POSITIONS.copy();target[2]=GROUND_COM_HEIGHT+dz*f
    return target,np.array([0.,0.,dz*fd]),np.array([0.,0.,dz*fdd])

def phase(t):
    return 'GROUND_HOLD' if t<TAKEOFF_START else 'TAKEOFF' if t<TAKEOFF_END else 'HOVER' if t<FORWARD_START else 'FORMATION_REACTIVE_FLIGHT'
